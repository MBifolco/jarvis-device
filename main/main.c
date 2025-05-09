// main.c — stereo capture for AFE + mono post-wake playback
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
#include "tone_play.h"

// ** Switch to L2CAP stream API **
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

static void i2s_mic_init(void) { /* unchanged */ }
static void i2s_play_init(void) { /* unchanged */ }

static void feed_task(void *arg) { /* unchanged */ }

static void fetch_task(void *arg)
{
    task_info_t *info = arg;
    int chunksize = info->iface->get_feed_chunksize(info->data);
    int channels  = info->iface->get_feed_channel_num(info->data);

    while (1) {
        afe_fetch_result_t *res;
        while ((res = info->iface->fetch(info->data)) != NULL) {
            if (res->wakeup_state == WAKENET_DETECTED) {
                ESP_LOGI(TAG, "Wake word detected (idx=%d)",
                         res->wakenet_model_index);

                // Pause background feed so AFE doesn't overflow
                vTaskSuspend(feed_handle);

                // Audible cue
                tone_play(1000, 100, 50);

                // Notify the app we're awake (via GATT)
                gatt_svc_notify_wake();
                ESP_LOGI(TAG, "Wake word notification sent");

                tone_play(1500, 80, 60);

                // Allocate buffers: stereo capture then mono
                int total_samples = POST_WAKE_SECONDS * 16000;
                size_t stereo_bytes = total_samples * channels * sizeof(int16_t);
                int16_t *stereo = malloc(stereo_bytes);
                int16_t *mono   = malloc(total_samples * sizeof(int16_t));
                if (!stereo || !mono) {
                    ESP_LOGE(TAG, "OOM on post-wake buffers");
                    free(stereo); free(mono);
                    vTaskResume(feed_handle);
                    continue;
                }

                // Capture exactly POST_WAKE_SECONDS worth of frames
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

                // De-interleave: take LEFT channel (mic) only
                for (size_t i = 0; i < captured; i++) {
                    mono[i] = stereo[i * channels];
                }
                free(stereo);

                // ── WAIT FOR L2CAP CHANNEL TO BE READY ──
                ESP_LOGI(TAG, "Waiting for L2CAP channel…");
                while (!l2cap_stream_is_connected()) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }

                // ── SEND VIA L2CAP ──
                size_t byte_len = captured * sizeof(int16_t);
                ESP_LOGI(TAG, "Sending %u bytes over L2CAP…", (unsigned)byte_len);
                esp_err_t err = l2cap_stream_send((const uint8_t *)mono, byte_len);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "L2CAP send failed: %d", err);
                } else {
                    ESP_LOGI(TAG, "L2CAP send complete");
                }
                free(mono);

                // Resume background feed
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

    ESP_LOGI(TAG, "Init AFE+WakeNet...");
    /* AFE init unchanged */

    task_info_t info = { .iface = iface, .data = data };
    xTaskCreate(feed_task,  "feed",  4096, &info, 5, &feed_handle);
    xTaskCreate(fetch_task, "fetch", 4096, &info, 5, NULL);

    ESP_LOGI(TAG, "Running—speak the wake word now!");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
