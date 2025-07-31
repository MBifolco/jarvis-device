#pragma once
#include "driver/i2s.h"
#include "config.h"      // for SAMPLE_RATE, I2S_MIC_PORT, I2S_SPK_PORT, etc.

// Initialize I2S for microphone (RX)
void i2s_mic_init(void);

// Initialize I2S for speaker/playback (TX)
void i2s_play_init(void);
