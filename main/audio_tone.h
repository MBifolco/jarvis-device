#pragma once

#include "driver/i2s_std.h"  // use the non-deprecated I2S API
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Initialize the tone generator on the given I2S port.
 */
esp_err_t tone_init(i2s_port_t port);

/**
 * @brief   Play a tone of specified frequency (Hz), duration (ms), and volume.
 *          Parameters are 32-bit to avoid any truncation.
 */
esp_err_t tone_play(uint32_t freq_hz, uint32_t duration_ms, uint8_t volume);

#ifdef __cplusplus
}
#endif
