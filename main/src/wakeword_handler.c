#include "wakeword_handler.h"
#include "keepalive.h"
#include "audio_pipeline.h"
#include "app_ble.h"
#include "audio_tone.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

static const char *TAG = "wakeword_handler";

void wakeword_handler_handle(
    task_info_t *info,
    TaskHandle_t feed_handle,
    TimerHandle_t keepAliveTimer
)
{
    ESP_LOGI(TAG, "Wake word detected!");
    // stop any pending keep-alive
    xTimerStop(keepAliveTimer, 0);
    keepalive_disable();

    // pause feeding while we switch contexts
    vTaskSuspend(feed_handle);
    tone_play(800, 100, 20);   // Lowered pitch from 1000Hz to 800Hz, volume 20%
    app_ble_notify_wake();

    // reset the wake-word AFE instance
    info->iface_wake->destroy(info->data_wake);
    info->data_wake = info->iface_wake->create_from_config(info->cfg_wake);

    // switch to post-wake (hold) AFE
    info->iface       = info->iface_hold;
    info->data        = info->data_hold;
    info->using_wake  = false;

    // arm the recording buffer
    audio_pipeline_alloc_mono_buf(audio_pipeline_get_buf_capacity());
    audio_pipeline_set_buf_filled(0);
    audio_pipeline_set_stop_record(false);
    audio_pipeline_set_recording(true);
    audio_pipeline_set_seen_speech(false);
    ESP_LOGI(TAG, "Recording armed");

    // resume feeding
    vTaskResume(feed_handle);
}
