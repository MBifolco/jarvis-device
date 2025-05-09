#include "audio_tone.h"
#include <math.h>
#include <stdlib.h>

static i2s_port_t s_tone_i2s_port = -1;
static const int SAMPLE_RATE = 16000;

void tone_set_i2s_port(i2s_port_t port) {
    s_tone_i2s_port = port;
}

esp_err_t tone_play(uint32_t freq_hz, uint32_t ms, uint8_t vol) {
    if (s_tone_i2s_port < 0) {
        return ESP_ERR_INVALID_STATE;  // you forgot to call tone_set_i2s_port()
    }

    int total_samples = (SAMPLE_RATE * ms) / 1000;
    int16_t *buf = malloc(sizeof(int16_t) * total_samples);
    if (!buf) return ESP_ERR_NO_MEM;

    const double phase_inc = 2.0 * M_PI * freq_hz / SAMPLE_RATE;
    for (int i = 0; i < total_samples; i++) {
        double sample = sin(i * phase_inc);
        buf[i] = (int16_t)(sample * 32767 * (vol / 100.0));
    }

    size_t bytes_written = 0;
    esp_err_t err = i2s_write(
      s_tone_i2s_port,
      buf,
      total_samples * sizeof(int16_t),
      &bytes_written,
      portMAX_DELAY
    );

    free(buf);
    return err;
}
