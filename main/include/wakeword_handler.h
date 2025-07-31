#ifndef WAKEWORD_HANDLER_H
#define WAKEWORD_HANDLER_H

#include "task_info.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

// Handles everything that used to live inline for wake-word detection.
// Pass it your shared info struct, the feed_task handle, and the keepAliveTimer.
void wakeword_handler_handle(
    task_info_t *info,
    TaskHandle_t feed_handle,
    TimerHandle_t keepAliveTimer
);

#endif // WAKEWORD_HANDLER_H
