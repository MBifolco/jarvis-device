#ifndef AFE_SETUP_H
#define AFE_SETUP_H

#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"

// Create & return a configured wake-word AFE/VAD (fast, no AEC)
afe_config_t *create_wake_afe(srmodel_list_t *models);

// Create & return a configured post-wake (hold) AFE/VAD (strict, with AEC)
afe_config_t *create_hold_afe(srmodel_list_t *models);

#endif // AFE_SETUP_H
