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
#include "audio_tone.h"

// L2CAP stream API
#include "l2cap_stream.h"

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

#define POST_WAKE_SECONDS 2   // seconds to record & then send
// ─────────────────────────────────────────────────────────────────────────────

static void i2s_mic_init(void)
{
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
    ESP_LOGI(TAG, "I2S RX initialized");
}

static void i2s_play_init(void)
{
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
    ESP_LOGI(TAG, "I2S TX initialized");
}

static void feed_task(void *arg)
{
    task_info_t *info = arg;
    int chunksize = info->iface->get_feed_chunksize(info->data);
    int channels  = info->iface->get_feed_channel_num(info->data); // == 2
    size_t buf_bytes = chunksize * channels * sizeof(int16_t);
    int16_t *buf = malloc(buf_bytes);

    while (1) {
        size_t br;
        if (i2s_read(I2S_MIC_PORT, buf, buf_bytes, &br, portMAX_DELAY) == ESP_OK) {
            info->iface->feed(info->data, buf);
        }
        vTaskDelay(pdMS_TO_TICKS((chunksize * 1000) / 16000));
    }
}

static void fetch_task(void *arg)
{
    task_info_t *info = arg;
    int chunksize = info->iface->get_feed_chunksize(info->data);
    int channels  = info->iface->get_feed_channel_num(info->data);

    while (1) {
        afe_fetch_result_t *res;
        while ((res = info->iface->fetch(info->data)) != NULL) {
            if (res->wakeup_state == WAKENET_DETECTED) {
                ESP_LOGI(TAG, "Wake word detected");

                vTaskSuspend(feed_handle);
                tone_play(1000, 100, 50);

                gatt_svc_notify_wake();
                ESP_LOGI(TAG, "GATT wake notify sent");
                tone_play(1500, 80, 60);

                // Capture
                int total_samples = POST_WAKE_SECONDS * 16000;
                size_t stereo_bytes = total_samples * channels * sizeof(int16_t);
                int16_t *stereo = malloc(stereo_bytes);
                int16_t *mono   = malloc(total_samples * sizeof(int16_t));
                if (!stereo || !mono) {
                    ESP_LOGE(TAG, "OOM");
                    free(stereo); free(mono);
                    vTaskResume(feed_handle);
                    continue;
                }

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

                for (size_t i = 0; i < captured; i++) {
                    mono[i] = stereo[i * channels];
                }
                free(stereo);

                // Wait for L2CAP
                while (!l2cap_stream_is_connected()) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }

                // Send via L2CAP
                size_t byte_len = captured * sizeof(int16_t);
                ESP_LOGI(TAG, "Sending %u bytes over L2CAP", (unsigned)byte_len);
                if (l2cap_stream_send((const uint8_t *)mono, byte_len) != ESP_OK) {
                    ESP_LOGE(TAG, "L2CAP send failed");
                }
                free(mono);

                vTaskResume(feed_handle);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Init I2S...");
    i2s_mic_init();
    i2s_play_init();
    tone_init(I2S_SPK_PORT);

    ESP_LOGI(TAG, "Init Bluetooth...");
    bluetooth_init();

    ESP_LOGI(TAG, "Init AFE...");
    srmodel_list_t *models = esp_srmodel_init("model");
    afe_config_t   *cfg    = afe_config_init("MR", models, AFE_TYPE_SR, AFE_MODE_LOW_COST);
    esp_afe_sr_iface_t *iface = esp_afe_handle_from_config(cfg);
    esp_afe_sr_data_t   *data  = iface->create_from_config(cfg);

    task_info_t info = { .iface = iface, .data = data };
    xTaskCreate(feed_task,  "feed",  4096, &info, 5, &feed_handle);
    xTaskCreate(fetch_task, "fetch", 4096, &info, 5, NULL);

    ESP_LOGI(TAG, "Waiting for wake word...");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
