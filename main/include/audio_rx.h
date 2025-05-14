#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include <stdbool.h>
/// Call once at startup to bring up I2S TX for speaker
void audio_rx_init(void);

void audio_rx_on_write(const uint8_t *data, size_t len);

static bool  g_playing_back = false;