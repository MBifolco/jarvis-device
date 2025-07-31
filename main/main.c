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
#include "afe_setup.h"
#include "audio_pipeline.h"
#include "keepalive.h"
#include "app_ble.h"
#include "audio_io.h"  // for audio_io_init()
#include "task_info.h"
#include "wakeword_handler.h"
#include "i2s_setup.h"  // for i2s_mic_init(), i2s_play_init()
#include "esp_task_wdt.h"  // for watchdog management

static const char *TAG          = "wakeword";
static const char *TAG_WAV      = "wav_player";
static TaskHandle_t  feed_handle     = NULL;
static TimerHandle_t keepAliveTimer  = NULL;

static task_info_t s_info;

// Keep-alive timeout callback
static void keep_alive_callback(TimerHandle_t xTimer)
{
    task_info_t *info = (task_info_t *)pvTimerGetTimerID(xTimer);

    if (g_playing_back) {
        ESP_LOGI(TAG, "Playback in progress, deferring keep-alive");
        xTimerReset(xTimer, 0);
        return;
    }

    ESP_LOGI(TAG, "Keep-alive expired, disarming");
    keepalive_disable();

    if (!info->using_wake) {
        // 1) Flip over to wake pipeline
        info->iface      = info->iface_wake;
        info->data       = info->data_wake;
        info->using_wake = true;

        // 2) Now safe to tear down & rebuild the hold instance
        info->iface_hold->destroy(info->data_hold);
        info->data_hold = info->iface_hold->create_from_config(info->cfg_hold);
 
        ESP_LOGI(TAG, "Switched back to wake-word VAD after keep-alive");
    }
}


// ────────────────────────────────────────────────────────────────────────────────

static void feed_task(void *arg)
{
    task_info_t *info  = (task_info_t *)arg;
    int  chunksize     = info->iface->get_feed_chunksize(info->data);
    int  channels      = info->iface->get_feed_channel_num(info->data);
    size_t frame_bytes = chunksize * channels * sizeof(int16_t);
    
    // Allocate I2S buffer on stack to avoid memory leak
    int16_t i2s_buf[chunksize * channels];

    ESP_LOGI(TAG, "feed_task started (chunk=%d, channels=%d)", chunksize, channels);
    
    // Subscribe to watchdog - this task should run regularly
    esp_task_wdt_add(NULL);
    while (1) {
        // Feed watchdog once per loop iteration
        esp_task_wdt_reset();
        
        if (g_playing_back) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        size_t read_bytes;
        if (i2s_read(I2S_MIC_PORT, i2s_buf, frame_bytes, &read_bytes, portMAX_DELAY) != ESP_OK) {
            ESP_LOGW(TAG, "I2S read failed");
            continue;
        }

        // 1) Feed, buffer, and send in one call
        audio_pipeline_process_chunk(
            info->iface,
            info->data,
            i2s_buf,
            read_bytes,
            channels,
            keepAliveTimer
        );

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static void fetch_task(void *arg)
{
    task_info_t *info = (task_info_t *)arg;
    afe_fetch_result_t *res;
    
    ESP_LOGI(TAG, "fetch_task started");
    
    // Don't subscribe to watchdog - fetch might legitimately have nothing to process
    // esp_task_wdt_add(NULL) - NOT calling this

    while (1) {
        while ((res = info->iface->fetch(info->data)) != NULL) {
            // 1) Wake word → arm record
            if (res->wakeup_state == WAKENET_DETECTED && info->using_wake) {
                wakeword_handler_handle(info, feed_handle, keepAliveTimer);
            }

            // 2) VAD logic during hold (now in audio_pipeline module)
            audio_pipeline_handle_vad(res, keepAliveTimer);
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
    app_ble_init();
    audio_io_init();

    // ← initialize audio_pipeline once
    audio_pipeline_init(POST_WAKE_SECONDS * SAMPLE_RATE);
    keepalive_init();

    ESP_LOGI(TAG, "Playing tone to indicate startup");
    tone_play(1000, 100, 50);

    keepAliveTimer = xTimerCreate(
        "keepAlive",
        pdMS_TO_TICKS(KEEP_ALIVE_MS),
        pdFALSE,
        (void *)&s_info,
        keep_alive_callback
    );
    
    // Initialize AFE/WakeNet models
    srmodel_list_t *models = esp_srmodel_init("model");

    // A) Wake-word AFE/VAD
    afe_config_t *cfg_wake = create_wake_afe(models);
    esp_afe_sr_iface_t *iface_wake = esp_afe_handle_from_config(cfg_wake);
    esp_afe_sr_data_t   *data_wake  = iface_wake->create_from_config(cfg_wake);

    // B) Post-wake AFE/VAD
    afe_config_t *cfg_hold = create_hold_afe(models);
    esp_afe_sr_iface_t *iface_hold = esp_afe_handle_from_config(cfg_hold);
    esp_afe_sr_data_t   *data_hold  = iface_hold->create_from_config(cfg_hold);

    // Populate global info struct
    s_info.cfg_wake    = cfg_wake;
    s_info.cfg_hold    = cfg_hold;
    s_info.iface_wake  = iface_wake;
    s_info.data_wake   = data_wake;
    s_info.iface_hold  = iface_hold;
    s_info.data_hold   = data_hold;
    s_info.iface       = iface_wake;
    s_info.data        = data_wake;
    s_info.using_wake  = true;

    xTaskCreate(feed_task,  "feed",  8192, &s_info, 5, &feed_handle);
    xTaskCreate(fetch_task, "fetch", 8192, &s_info, 5, NULL);

    tone_play(1000, 100, 50);
    ESP_LOGI(TAG, "Ready—say the wake word!");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
