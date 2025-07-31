#ifndef TASK_INFO_H
#define TASK_INFO_H

#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_afe_sr_iface.h"

// Mirrors the old struct in main.c
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

#endif // TASK_INFO_H
