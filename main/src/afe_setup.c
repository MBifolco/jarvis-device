#include "afe_setup.h"

afe_config_t *create_wake_afe(srmodel_list_t *models)
{
    // fast trigger, no AEC
    afe_config_t *cfg = afe_config_init("MR", models, AFE_TYPE_SR, AFE_MODE_LOW_COST);
    cfg->wakenet_init     = true;
    cfg->vad_init         = true;
    cfg->aec_init         = false;
    cfg->vad_min_speech_ms = 100;   // 0.1 s speech
    cfg->vad_min_noise_ms  = 500;   // 0.5 s noise→exit
    cfg->vad_mode          = VAD_MODE_0;
    return cfg;
}

afe_config_t *create_hold_afe(srmodel_list_t *models)
{
    // stricter VAD after wake, with AEC
    afe_config_t *cfg = afe_config_init("MR", models, AFE_TYPE_SR, AFE_MODE_LOW_COST);
    cfg->wakenet_init     = false;
    cfg->vad_init         = true;
    cfg->aec_init         = true;
    cfg->vad_min_speech_ms = 500;   // 0.5 s speech
    cfg->vad_min_noise_ms  = 1500;  // 1.5 s silence→exit
    cfg->vad_mode          = VAD_MODE_1;
    cfg->vad_delay_ms      = 64;    // ~62.5 ms lookback
    return cfg;
}
