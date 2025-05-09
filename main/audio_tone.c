#include "audio_tone.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "esp_log.h"

static const char *TAG = "tone";
static i2s_port_t tone_i2s_port;

/**
 * Store the I2S port to use for tone playback.
 */
esp_err_t tone_init(i2s_port_t port) {
    tone_i2s_port = port;
    return ESP_OK;
}

/**
 * Play a simple square wave tone.
 * (This example assumes a pre-filled buffer; you can replace
 *  with your own waveform generation as needed.)
 */
esp_err_t tone_play(uint32_t freq_hz, uint32_t duration_ms, uint8_t volume) {
    // Number of samples for 1 period at 16 kHz
    uint32_t samples_per_period = 16000 / freq_hz;
    uint32_t total_samples = (16000 * duration_ms) / 1000;
    // Allocate a small square wave buffer
    uint16_t *buf = malloc(samples_per_period * sizeof(uint16_t));
    if (!buf) {
        ESP_LOGE(TAG, "OOM allocating tone buffer");
        return ESP_ERR_NO_MEM;
    }
    // Fill half the period high, half low
    for (uint32_t i = 0; i < samples_per_period; i++) {
        buf[i] = (i < (samples_per_period/2)) ? (volume << 8) : 0;
    }

    uint32_t sent = 0;
    while (sent < total_samples) {
        size_t to_send = MIN(samples_per_period, total_samples - sent);
        size_t written;
        esp_err_t err = i2s_write(
            tone_i2s_port,
            buf,
            to_send * sizeof(uint16_t),
            &written,
            portMAX_DELAY
        );
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "i2s_write failed: %d", err);
            free(buf);
            return err;
        }
        sent += to_send;
    }
    free(buf);
    return ESP_OK;
}
