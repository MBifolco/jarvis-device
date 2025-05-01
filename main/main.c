// wake_word_engine.c — stereo capture for AFE + mono post-wake playback
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"
#include "driver/i2s.h"
#include "driver/gpio.h"

#include "esp_nimble_hci.h"
#include "bluetooth.h"
#include "gatt_svc.h"

static const char *TAG = "wakeword";
static TaskHandle_t feed_handle = NULL;

typedef struct {
    esp_afe_sr_iface_t *iface;
    esp_afe_sr_data_t   *data;
} task_info_t;

// ─── BOARD-SPECIFIC ──────────────────────────────────────────────────────────
#define MIC_BCK_IO        GPIO_NUM_5
#define MIC_WS_IO         GPIO_NUM_6
#define MIC_DATA_IO       GPIO_NUM_4

#define SPK_BCK_IO        GPIO_NUM_7
#define SPK_WS_IO         GPIO_NUM_8
#define SPK_DATA_IO       GPIO_NUM_9

#define I2S_MIC_PORT      I2S_NUM_0
#define I2S_SPK_PORT      I2S_NUM_1

#define POST_WAKE_SECONDS 5   // seconds to record & then play back
// ─────────────────────────────────────────────────────────────────────────────

static void i2s_mic_init(void)
{
    // Must be stereo so AFE sees [mic, echo] channels
    i2s_config_t cfg = {
        .mode                = I2S_MODE_MASTER | I2S_MODE_RX,
        .sample_rate         = 16000,
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
    ESP_ERROR_CHECK(i2s_set_pin(       I2S_MIC_PORT, &pins));
    ESP_LOGI(TAG, "I2S RX initialized (BCK=%d WS=%d SD=%d)",
             MIC_BCK_IO, MIC_WS_IO, MIC_DATA_IO);
}

static void i2s_play_init(void)
{
    // mono out is fine for playback
    i2s_config_t cfg = {
        .mode                = I2S_MODE_MASTER | I2S_MODE_TX,
        .sample_rate         = 16000,
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
    ESP_ERROR_CHECK(i2s_set_pin(       I2S_SPK_PORT, &pins));
    ESP_LOGI(TAG, "I2S TX initialized (BCK=%d WS=%d DOUT=%d)",
             SPK_BCK_IO, SPK_WS_IO, SPK_DATA_IO);
}

static void feed_task(void *arg)
{
    task_info_t *info = arg;
    int chunksize = info->iface->get_feed_chunksize(info->data);
    int channels  = info->iface->get_feed_channel_num(info->data); // == 2
    size_t buf_bytes = chunksize * channels * sizeof(int16_t);
    int16_t *buf = malloc(buf_bytes);

    ESP_LOGI(TAG, "feed_task: chunk=%d chan=%d", chunksize, channels);
    while (1) {
        size_t br;
        if (i2s_read(I2S_MIC_PORT, buf, buf_bytes, &br, portMAX_DELAY) != ESP_OK) {
            ESP_LOGW(TAG, "I2S read error");
            continue;
        }
        info->iface->feed(info->data, buf);
        vTaskDelay(pdMS_TO_TICKS((chunksize * 1000) / 16000));
    }
}

static void fetch_task(void *arg)
{
    task_info_t *info = arg;
    int chunksize = info->iface->get_feed_chunksize(info->data);
    int channels  = info->iface->get_feed_channel_num(info->data);

    while (1) {
        // drain all pending results
        afe_fetch_result_t *res;
        while ((res = info->iface->fetch(info->data)) != NULL) {
            if (res->wakeup_state == WAKENET_DETECTED) {
                ESP_LOGI(TAG, "Wake word detected (idx=%d)",
                         res->wakenet_model_index);
                    
                vTaskSuspend(feed_handle);

                gatt_svc_notify_wake();
                ESP_LOGI(TAG, "Wake word notification sent");

                // allocate stereo buffer then a mono buffer
                int total_samples = POST_WAKE_SECONDS * 16000;
                size_t stereo_bytes = total_samples * channels * sizeof(int16_t);
                int16_t *stereo = malloc(stereo_bytes);
                int16_t *mono   = malloc(total_samples * sizeof(int16_t));
                if (!stereo || !mono) {
                    ESP_LOGE(TAG, "OOM on post-wake buffers");
                    free(stereo); free(mono);
                    continue;
                }

                // read exactly POST_WAKE_SECONDS worth of FRAMEs
                size_t captured = 0;
                while (captured < (size_t)total_samples) {
                    size_t br2;
                    size_t to_read = chunksize * channels * sizeof(int16_t);
                    if (i2s_read(I2S_MIC_PORT,
                                 stereo + captured * channels,
                                 to_read, &br2, portMAX_DELAY) != ESP_OK) {
                        break;
                    }
                    captured += br2 / (channels * sizeof(int16_t));
                }
                ESP_LOGI(TAG, "Captured %u frames, de-interleaving…", captured);

                // pull out LEFT channel (mic) only
                for (size_t i = 0; i < captured; i++) {
                    mono[i] = stereo[i * channels];
                }
                free(stereo);

                // playback mono buffer
                size_t written;
                esp_err_t err = i2s_write(I2S_SPK_PORT, mono,
                                          captured * sizeof(int16_t),
                                          &written, portMAX_DELAY);
                ESP_LOGI(TAG,
                         "Playback done (frames=%u, bytes=%u, err=%d)",
                         (unsigned)captured, (unsigned)written, err);

                free(mono);

                // ── Now resume feeding so AFE can keep up ──
               vTaskResume(feed_handle);
            }
        }
        // no more results right now
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Init I2S...");
    i2s_mic_init();
    i2s_play_init();

    ESP_LOGI(TAG, "Init Bluetooth...");
    bluetooth_init();

    
    ESP_LOGI(TAG, "Init AFE+WakeNet...");
    srmodel_list_t *models = esp_srmodel_init("model");
    if (!models) {
        ESP_LOGE(TAG, "srmodel_init failed");
        return;
    }

    afe_config_t *cfg = afe_config_init("MR", models,
                                        AFE_TYPE_SR, AFE_MODE_LOW_COST);
    cfg->wakenet_init = true;
    cfg->vad_init     = true;

    esp_afe_sr_iface_t *iface = esp_afe_handle_from_config(cfg);
    esp_afe_sr_data_t   *data  = iface->create_from_config(cfg);
    if (!iface || !data) {
        ESP_LOGE(TAG, "AFE handle/data init failed");
        afe_config_free(cfg);
        esp_srmodel_deinit(models);
        return;
    }

    task_info_t info = { .iface = iface, .data = data };
    xTaskCreate(feed_task,  "feed",  4096, &info, 5, &feed_handle);
    xTaskCreate(fetch_task, "fetch", 4096, &info, 5, NULL);

    ESP_LOGI(TAG, "Running—speak the wake word now!");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
