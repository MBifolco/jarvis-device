// main.c — stereo capture with post-wake VAD stop and keep-alive
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

#include "esp_nimble_hci.h"
#include "bluetooth.h"
#include "gatt_svc.h"
#include "audio_tone.h"
#include "audio_tx.h"
#include "audio_rx.h"

static const char *TAG = "wakeword";
static TaskHandle_t  feed_handle = NULL;

// Shared between tasks
static volatile bool    s_recording     = false;  // armed on wake
static volatile bool    s_stop_record   = false;  // set by VAD or cap
static volatile bool    s_seen_speech   = false;  // only stop after some speech
static volatile bool    s_keep_alive    = false;  // after send, remain ready
static size_t           s_buf_capacity  = 0;      // in samples
static size_t           s_buf_filled    = 0;      // in samples
static int16_t         *s_mono_buf      = NULL;   // PCM buffer
static TimerHandle_t    keepAliveTimer  = NULL;

// Minimum samples to collect before allowing VAD silence to stop
#define MIN_RECORD_SAMPLES (SAMPLE_RATE / 1)       // 0.2 seconds at 16 kHz citeturn16file0
#define KEEP_ALIVE_MS      (30000)                 // 5-second keep-alive window

typedef struct {
    esp_afe_sr_iface_t *iface;
    esp_afe_sr_data_t   *data;
} task_info_t;

// ─── HARDWARE & TIMING ─────────────────────────────────────────────────────────
#define MIC_BCK_IO        GPIO_NUM_5 // SCK purple
#define MIC_WS_IO         GPIO_NUM_6 // WS green
#define MIC_DATA_IO       GPIO_NUM_4 // SD orange

#define SPK_BCK_IO        GPIO_NUM_10 // BCK purple
#define SPK_WS_IO         GPIO_NUM_11 // LRC green
#define SPK_DATA_IO       GPIO_NUM_9 // DIN orange

#define I2S_MIC_PORT      I2S_NUM_0
#define I2S_SPK_PORT      I2S_NUM_1

#define SAMPLE_RATE       16000
#define POST_WAKE_SECONDS 30         // max record length
// ────────────────────────────────────────────────────────────────────────────────

static void keep_alive_callback(TimerHandle_t xTimer) {
    ESP_LOGI(TAG, "Keep-alive expired, disarming");
    s_keep_alive = false;
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
    task_info_t *info = arg;
    int  chunksize = info->iface->get_feed_chunksize(info->data);
    int  channels  = info->iface->get_feed_channel_num(info->data);
    size_t frame_bytes = chunksize * channels * sizeof(int16_t);
    int16_t *i2s_buf = malloc(frame_bytes);

    s_buf_capacity = POST_WAKE_SECONDS * SAMPLE_RATE;

    ESP_LOGI(TAG, "feed_task started (chunk=%d, channels=%d)", chunksize, channels);
    while (1) {
        if (g_playing_back) {
            continue;
        }
        
        size_t read_bytes;
        if (i2s_read(I2S_MIC_PORT, i2s_buf, frame_bytes, &read_bytes, portMAX_DELAY) != ESP_OK) {
            ESP_LOGW(TAG, "I2S read failed");
            continue;
        }

        // 1) feed AFE for wake/VAD
        info->iface->feed(info->data, i2s_buf);

        // 2) capture if armed or in keep-alive
        if ((s_recording || s_keep_alive) && !s_stop_record && s_mono_buf) {
            size_t frames = read_bytes / (channels * sizeof(int16_t));
            for (size_t i = 0; i < frames && s_buf_filled < s_buf_capacity; i++) {
                s_mono_buf[s_buf_filled++] = i2s_buf[i * channels];
            }
        }

        // 3) stop on VAD or cap
        if (s_recording && (s_stop_record || s_buf_filled >= s_buf_capacity)) {
            ESP_LOGI(TAG, "Sending %u samples (%.2f s)…",
                     (unsigned)s_buf_filled,
                     s_buf_filled / (float)SAMPLE_RATE);

            audio_tx_send((const uint8_t*)s_mono_buf,
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
    }
}

static void fetch_task(void *arg)
{
    task_info_t *info = arg;
    afe_fetch_result_t *res;

    while (1) {
        while ((res = info->iface->fetch(info->data)) != NULL) {
            // 1) Wake word → arm record immediately
            if (res->wakeup_state == WAKENET_DETECTED) {
                ESP_LOGI(TAG, "Wake word detected!");
                xTimerStop(keepAliveTimer, 0);
                s_keep_alive   = false;

                vTaskSuspend(feed_handle);
                ESP_LOGI(TAG, "Tone Play");
                tone_play(1000, 100, 50);
                gatt_svc_notify_wake();
                //tone_play(1500, 80, 60);
                vTaskResume(feed_handle);

                // alloc & arm
                s_mono_buf    = malloc(s_buf_capacity * sizeof(int16_t));
                s_buf_filled  = 0;
                s_stop_record = false;
                s_recording   = true;
                s_seen_speech = false;
                ESP_LOGI(TAG, "Recording up to %d seconds…", POST_WAKE_SECONDS);
            }

            // 2) VAD logic
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
                } else if (!res->vad_state) {
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

    bluetooth_init();
    audio_rx_init();
    audio_tx_compression_init();

    // create keep-alive timer
    keepAliveTimer = xTimerCreate(
        "keepAlive",
        pdMS_TO_TICKS(KEEP_ALIVE_MS),
        pdFALSE,
        NULL,
        keep_alive_callback
    );

    // AFE + WakeNet + VAD
    srmodel_list_t *models = esp_srmodel_init("model");
    afe_config_t   *cfg    = afe_config_init("MR", models, AFE_TYPE_SR, AFE_MODE_LOW_COST);
    cfg->wakenet_init = true;
    cfg->vad_init     = true;

    esp_afe_sr_iface_t *iface = esp_afe_handle_from_config(cfg);
    esp_afe_sr_data_t   *data  = iface->create_from_config(cfg);

    task_info_t info = { .iface = iface, .data = data };
    xTaskCreate(feed_task,  "feed",  4096, &info, 5, &feed_handle);
    xTaskCreate(fetch_task, "fetch", 4096, &info, 5, NULL);

    ESP_LOGI(TAG, "Ready—say the wake word!");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
