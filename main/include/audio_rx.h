#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"

/// True while the RX task is actively streaming PCM to I2S.
extern bool g_playing_back;

/**
 * @brief  Initialize the audio‐receive subsystem.
 *         - allocates PCM decode buffer
 *         - creates the ADPCM StreamBuffer
 *         - spawns the background RX/decoder task
 *
 * @return ESP_OK on success, or ESP_ERR_NO_MEM if malloc/streambuf fails.
 */
esp_err_t audio_rx_init(void);

/**
 * @brief  Enqueue raw BLE‐received bytes into the ADPCM StreamBuffer.
 *         Blocks up to 100 ms if the buffer is full before dropping.
 *
 * @param  data pointer to incoming bytes (including 4-byte length header)
 * @param  len  number of bytes
 * @return ESP_OK if queued, ESP_ERR_TIMEOUT if dropped
 */
esp_err_t audio_rx_on_write(const uint8_t *data, size_t len);
