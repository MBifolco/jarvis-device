// main.c — stereo capture with separate wake-word and post-wake VAD instances
// Note: Wake instance has AEC disabled to avoid “AEC engine destroyed” errors on destroy.
//       Hold instance keeps AEC enabled for post-wake recordings.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"
#include "driver/i2s.h"
#include "driver/gpio.h"
#include <dirent.h>
#include <sys/stat.h>


#include "esp_nimble_hci.h"
#include "bluetooth.h"
#include "gatt_svc.h"
#include "audio_tone.h"
#include "audio_tx.h"
#include "audio_rx.h"
#include "config.h"

static const char *TAG          = "wakeword";
static const char *TAG_WAV      = "wav_player";
static TaskHandle_t  feed_handle     = NULL;
static TimerHandle_t keepAliveTimer  = NULL;

// Shared between tasks / callback
static volatile bool    s_recording     = false;  // armed on wake
static volatile bool    s_stop_record   = false;  // set by VAD or cap
static volatile bool    s_seen_speech   = false;  // only stop after some speech
static volatile bool    s_keep_alive    = false;  // after send, remain ready
static size_t           s_buf_capacity  = 0;      // in samples
static size_t           s_buf_filled    = 0;      // in samples
static int16_t         *s_mono_buf      = NULL;   // PCM buffer

// Minimum samples to collect before allowing VAD silence to stop
#define MIN_RECORD_SAMPLES (SAMPLE_RATE / 1)  // 1 second at 16 kHz
#define KEEP_ALIVE_MS      (20000)            // 20-second keep-alive window

typedef struct {
    afe_config_t         *cfg_wake;
    afe_config_t         *cfg_hold;
    esp_afe_sr_iface_t   *iface;         // current interface pointer
    esp_afe_sr_data_t    *data;          // current data pointer
    esp_afe_sr_iface_t   *iface_wake;    // wake-word instance
    esp_afe_sr_data_t    *data_wake;
    esp_afe_sr_iface_t   *iface_hold;    // post-wake (hold) instance
    esp_afe_sr_data_t    *data_hold;
    bool                  using_wake;    // true if currently in wake-word mode
} task_info_t;

static task_info_t s_info;

// ─── HARDWARE & TIMING ─────────────────────────────────────────────────────────
#define MIC_BCK_IO        GPIO_NUM_5   // SCK purple
#define MIC_WS_IO         GPIO_NUM_6   // WS green
#define MIC_DATA_IO       GPIO_NUM_4   // SD orange

#define SPK_BCK_IO        GPIO_NUM_10  // BCK purple
#define SPK_WS_IO         GPIO_NUM_11  // LRC green
#define SPK_DATA_IO       GPIO_NUM_9   // DIN orange

#define I2S_MIC_PORT      I2S_NUM_0
#define I2S_SPK_PORT      I2S_NUM_1

#define SAMPLE_RATE       16000
#define POST_WAKE_SECONDS 30           // max record length
// ────────────────────────────────────────────────────────────────────────────────

// ─── WAV SPIFFS + PLAYBACK HELPERS ─────────────────────────────────────────────

// Read 32-bit little-endian from buffer
static uint32_t read_le32(const uint8_t *p) {
    return ((uint32_t)p[0]) |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

// Configure I2S TX for given sample_rate (16-bit mono)
static esp_err_t configure_i2s_for_wav(uint32_t sample_rate) {
    // Uninstall any existing driver on I2S_SPK_PORT
    i2s_driver_uninstall(I2S_SPK_PORT);

    i2s_config_t cfg = {
        .mode                 = I2S_MODE_MASTER | I2S_MODE_TX,
        .sample_rate          = sample_rate,
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,
        .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count        = 2,
        .dma_buf_len          = 1024,
        .use_apll             = false,
        .tx_desc_auto_clear   = true,
        .fixed_mclk           = 0
    };
    esp_err_t err = i2s_driver_install(I2S_SPK_PORT, &cfg, 0, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_WAV, "i2s_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }
    i2s_pin_config_t pins = {
        .bck_io_num   = SPK_BCK_IO,
        .ws_io_num    = SPK_WS_IO,
        .data_out_num = SPK_DATA_IO,
        .data_in_num  = I2S_PIN_NO_CHANGE
    };
    err = i2s_set_pin(I2S_SPK_PORT, &pins);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_WAV, "i2s_set_pin failed: %s", esp_err_to_name(err));
    }
    return err;
}

static void keep_alive_callback(TimerHandle_t xTimer)
{
    task_info_t *info = (task_info_t *)pvTimerGetTimerID(xTimer);
    ESP_LOGI(TAG, "Keep-alive expired, disarming");

    s_keep_alive = false;

    // If currently using hold (post-wake) instance, switch back to wake-word instance
    if (!info->using_wake) {
        // Destroy current hold data (including its AEC engine)
        info->iface_hold->destroy(info->data_hold);
        // Re-create hold data for next cycle
        info->data_hold = info->iface_hold->create_from_config(info->cfg_hold);

        // Switch back to wake-word interface
        info->iface       = info->iface_wake;
        info->data        = info->data_wake;
        info->using_wake  = true;

        ESP_LOGI(TAG, "Switched back to wake-word VAD after keep-alive");
    }
}

static void i2s_mic_init(void)
{
    i2s_config_t cfg = {
        .mode                = I2S_MODE_MASTER | I2S_MODE_RX,
        .sample_rate         = SAMPLE_RATE,
        .bits_per_sample     = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format      = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format= I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags    = 0,
        .dma_buf_count       = 4,
        .dma_buf_len         = 512,
        .use_apll            = false,
        .tx_desc_auto_clear  = false,
        .fixed_mclk          = 0
    };
    i2s_pin_config_t pins = {
        .bck_io_num   = MIC_BCK_IO,
        .ws_io_num    = MIC_WS_IO,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num  = MIC_DATA_IO
    };
    ESP_ERROR_CHECK(i2s_driver_install(I2S_MIC_PORT, &cfg, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_MIC_PORT, &pins));
    ESP_LOGI(TAG, "I2S RX inited");
}

static void i2s_play_init(void)
{
    i2s_config_t cfg = {
        .mode                = I2S_MODE_MASTER | I2S_MODE_TX,
        .sample_rate         = SAMPLE_RATE,
        .bits_per_sample     = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format      = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format= I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags    = 0,
        .dma_buf_count       = 4,
        .dma_buf_len         = 512,
        .use_apll            = false,
        .tx_desc_auto_clear  = true,
        .fixed_mclk          = 0
    };
    i2s_pin_config_t pins = {
        .bck_io_num   = SPK_BCK_IO,
        .ws_io_num    = SPK_WS_IO,
        .data_out_num = SPK_DATA_IO,
        .data_in_num  = I2S_PIN_NO_CHANGE
    };
    ESP_ERROR_CHECK(i2s_driver_install(I2S_SPK_PORT, &cfg, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_SPK_PORT, &pins));
    ESP_LOGI(TAG, "I2S TX inited");
}

static void feed_task(void *arg)
{
    task_info_t *info  = (task_info_t *)arg;
    int  chunksize     = info->iface->get_feed_chunksize(info->data);
    int  channels      = info->iface->get_feed_channel_num(info->data);
    size_t frame_bytes = chunksize * channels * sizeof(int16_t);
    int16_t *i2s_buf   = malloc(frame_bytes);

    s_buf_capacity = POST_WAKE_SECONDS * SAMPLE_RATE;

    ESP_LOGI(TAG, "feed_task started (chunk=%d, channels=%d)", chunksize, channels);
    while (1) {
        if (g_playing_back) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        size_t read_bytes;
        if (i2s_read(I2S_MIC_PORT, i2s_buf, frame_bytes, &read_bytes, portMAX_DELAY) != ESP_OK) {
            ESP_LOGW(TAG, "I2S read failed");
            continue;
        }

        // 1) Feed AFE/VAD (either wake or hold, depending on current mode)
        info->iface->feed(info->data, i2s_buf);

        // 2) If armed or in keep-alive, capture PCM
        if ((s_recording || s_keep_alive) && !s_stop_record && s_mono_buf) {
            size_t frames = read_bytes / (channels * sizeof(int16_t));
            for (size_t i = 0; i < frames && s_buf_filled < s_buf_capacity; i++) {
                // Take only the first channel (mono)
                s_mono_buf[s_buf_filled++] = i2s_buf[i * channels];
            }
        }

        // 3) Stop on VAD or capacity
        if (s_recording && (s_stop_record || s_buf_filled >= s_buf_capacity)) {
            ESP_LOGI(TAG, "Sending %u samples (%.2f s)…",
                     (unsigned)s_buf_filled,
                     s_buf_filled / (float)SAMPLE_RATE);

            audio_tx_send((const uint8_t *)s_mono_buf,
                          s_buf_filled * sizeof(int16_t));

            free(s_mono_buf);
            s_mono_buf    = NULL;
            s_recording   = false;
            s_seen_speech = false;
            s_buf_filled  = 0;
            // start keep-alive
            s_keep_alive = true;
            xTimerReset(keepAliveTimer, 0);
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static void fetch_task(void *arg)
{
    task_info_t *info = (task_info_t *)arg;
    afe_fetch_result_t *res;

    while (1) {
        while ((res = info->iface->fetch(info->data)) != NULL) {
            // 1) Wake word → arm record immediately (only if currently in wake mode)
            if (res->wakeup_state == WAKENET_DETECTED && info->using_wake) {
                ESP_LOGI(TAG, "Wake word detected!");
                xTimerStop(keepAliveTimer, 0);
                s_keep_alive   = false;

                // Pause feed while we play tones & switch AFE instances
                vTaskSuspend(feed_handle);
                tone_play(1000, 100, 50);
                gatt_svc_notify_wake();
                tone_play(1500, 80, 60);

                // Re-create wake data (destroy old AEC-free context, though no AEC here)
                info->iface_wake->destroy(info->data_wake);
                info->data_wake = info->iface_wake->create_from_config(info->cfg_wake);

                // Switch to hold (post-wake) interface, which has AEC enabled
                info->iface       = info->iface_hold;
                info->data        = info->data_hold;
                info->using_wake  = false;

                // Allocate and arm PCM buffer
                s_mono_buf    = malloc(s_buf_capacity * sizeof(int16_t));
                s_buf_filled  = 0;
                s_stop_record = false;
                s_recording   = true;
                s_seen_speech = false;
                ESP_LOGI(TAG, "Recording up to %d seconds…", POST_WAKE_SECONDS);

                vTaskResume(feed_handle);
            }

            // 2) VAD logic (active during record or keep-alive, only in hold mode)
            if ((s_recording || s_keep_alive) && res->wakeup_state != WAKENET_DETECTED) {
                if (res->vad_state) {
                    s_seen_speech = true;
                    // on new speech in keep-alive, re-arm full record
                    if (s_keep_alive && !s_recording) {
                        xTimerStop(keepAliveTimer, 0);
                        s_keep_alive  = false;
                        s_mono_buf    = malloc(s_buf_capacity * sizeof(int16_t));
                        s_buf_filled  = 0;
                        s_stop_record = false;
                        s_recording   = true;
                        s_seen_speech = true;
                        ESP_LOGI(TAG, "Re-armed recording during keep-alive");
                    }
                } else {
                    if (s_recording && s_seen_speech && s_buf_filled > MIN_RECORD_SAMPLES) {
                        ESP_LOGI(TAG, "VAD silence → stopping (samples=%u)", (unsigned)s_buf_filled);
                        s_stop_record = true;
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Init I2S, Bluetooth, AFE & compression");
    i2s_mic_init();
    i2s_play_init();
    tone_set_i2s_port(I2S_SPK_PORT);

    config_init();
    bluetooth_init();
    audio_rx_init();
    audio_tx_compression_init();

    ESP_LOGI(TAG, "Playing tone to indicate startup");
    // ─── PLAY TONE TO INDICATE STARTUP ────────────────────────────────────────────
    tone_play(1000, 100, 50);
    
    // Create keep-alive timer; pass &s_info as timer ID
    keepAliveTimer = xTimerCreate(
        "keepAlive",
        pdMS_TO_TICKS(KEEP_ALIVE_MS),
        pdFALSE,
        (void *)&s_info,
        keep_alive_callback
    );

    // Initialize AFE/WakeNet models
    srmodel_list_t *models = esp_srmodel_init("model");

    //---------------------------------------------------------------
    // A) Wake-word AFE/VAD (fast trigger, high sensitivity, no AEC)
    //---------------------------------------------------------------
    afe_config_t *cfg_wake = afe_config_init("MR", models, AFE_TYPE_SR, AFE_MODE_LOW_COST);
    cfg_wake->wakenet_init     = true;
    cfg_wake->vad_init         = true;
    cfg_wake->aec_init         = false;    // disable AEC for wake instance
    // Make VAD very eager to pick up any speech
    cfg_wake->vad_min_speech_ms  = 100;   // only 0.1 s of speech needed
    cfg_wake->vad_min_noise_ms   = 500;   // 0.5 s of continuous “noise” to exit “speech”
    cfg_wake->vad_mode           = VAD_MODE_0; // most sensitive
    // Leave vad_delay_ms at default (~128 ms)

    esp_afe_sr_iface_t *iface_wake = esp_afe_handle_from_config(cfg_wake);
    esp_afe_sr_data_t   *data_wake  = iface_wake->create_from_config(cfg_wake);

    //---------------------------------------------------------------
    // B) Post-wake AFE/VAD (strict, to avoid false keeps, with AEC)
    //---------------------------------------------------------------
    afe_config_t *cfg_hold = afe_config_init("MR", models, AFE_TYPE_SR, AFE_MODE_LOW_COST);
    cfg_hold->wakenet_init     = false;   // No WakeNet after initial trigger
    cfg_hold->vad_init         = true;
    cfg_hold->aec_init         = true;    // keep AEC enabled for post-wake
    // Only commit to “speech” if user is really talking
    cfg_hold->vad_min_speech_ms  = 500;   // 0.3 s of continuous speech
    cfg_hold->vad_min_noise_ms   = 1500;  // 1.5 s of continuous silence/noise to stop
    cfg_hold->vad_mode           = VAD_MODE_1; // more conservative
    cfg_hold->vad_delay_ms       = 64;    // shorter lookback (62.5 ms)

    esp_afe_sr_iface_t *iface_hold = esp_afe_handle_from_config(cfg_hold);
    esp_afe_sr_data_t   *data_hold  = iface_hold->create_from_config(cfg_hold);

    // Populate global info struct
    s_info.cfg_wake       = cfg_wake;
    s_info.cfg_hold       = cfg_hold;
    s_info.iface_wake     = iface_wake;
    s_info.data_wake      = data_wake;
    s_info.iface_hold     = iface_hold;
    s_info.data_hold      = data_hold;
    s_info.iface          = iface_wake;  // start with wake-word instance
    s_info.data           = data_wake;
    s_info.using_wake     = true;

    // Create tasks, passing &s_info
    xTaskCreate(feed_task,  "feed",  4096, &s_info, 5, &feed_handle);
    xTaskCreate(fetch_task, "fetch", 4096, &s_info, 5, NULL);

    tone_play(1000, 100, 50);

    ESP_LOGI(TAG, "Ready—say the wake word!");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
