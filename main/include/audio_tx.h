#pragma once
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

// Call once at startup to initialize the ADPCM encoder
esp_err_t audio_tx_compression_init(void);

// Call this after capturing post-wake audio.
// pcm: raw PCM data (16-bit mono at 16kHz)
// len: length in bytes
esp_err_t audio_tx_send(const uint8_t *pcm, size_t len);
