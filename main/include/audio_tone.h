#pragma once
#include "driver/i2s.h"
#include "esp_err.h"

// Tell the tone module which I2S port you already initialized.
void tone_set_i2s_port(i2s_port_t port);

// Play a tone on the previously-set port.
// freq_hz: frequency in Hz
// ms:      duration in milliseconds
// vol:     0–100 volume percentage
esp_err_t tone_play(uint32_t freq_hz, uint32_t ms, uint8_t vol);
