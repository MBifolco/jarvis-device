/* audio_rx.h */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"

/// Global flag set true while playing back the buffered PCM
extern bool g_playing_back;


/// Initialize the audio receiver and I²S peripheral.
void audio_rx_init(void);

/// Feed in each BLE write chunk (with a 4-byte length header + ADPCM payload).
void audio_rx_on_write(const uint8_t *data, size_t len);

/// Once the full ADPCM stream has arrived, play back the buffered PCM.
void audio_rx_playback(void);


