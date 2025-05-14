/* audio_rx.c */
#include "audio_rx.h"
#include <string.h>
#include "esp_log.h"
#include "driver/i2s.h"

static const char *TAG = "audio_rx";

// --- adjust these to match your board wiring ---
#define I2S_SPK_PORT      I2S_NUM_1
#define BITS_PER_SAMPLE   16
#define CHANNELS          1

// --- internal buffer to accumulate one WAV file ---
static uint8_t *s_buf = NULL;
static size_t   s_capacity = 0;
static size_t   s_filled   = 0;
static size_t   s_expected = 0;

void audio_rx_init(void)
{
    // allocate a reasonable max size: 5s @ max expected sample rate (e.g. 48kHz * 2B * 5s)
    s_capacity = 48000 * (BITS_PER_SAMPLE/8) * 5;
    s_buf      = malloc(s_capacity);
    s_filled   = 0;
    s_expected = 0;
}

static uint32_t read_le32(const uint8_t *p) {
    return (uint32_t)p[0]
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

void audio_rx_on_write(const uint8_t *data, size_t len)
{
    if (s_filled + len > s_capacity) {
        ESP_LOGW(TAG, "overflow, resetting buffer");
        s_filled   = 0;
        s_expected = 0;
    }

    // copy chunk into buffer
    memcpy(s_buf + s_filled, data, len);
    s_filled += len;
    ESP_LOGD(TAG, "got chunk %u bytes (filled=%u)", (unsigned)len, (unsigned)s_filled);

    // parse WAV header once we have at least 44 bytes
    if (s_expected == 0 && s_filled >= 44) {
        // extract sample rate from header offset 24
        uint32_t sample_rate = read_le32(s_buf + 24);
        ESP_LOGI(TAG, "WAV header: sample_rate=%u, channels=%u, bits/sample=%u",
                 (unsigned)sample_rate, CHANNELS, BITS_PER_SAMPLE);
        // reconfigure I2S to match incoming stream
        i2s_set_clk(I2S_SPK_PORT, sample_rate, BITS_PER_SAMPLE, CHANNELS);

        // data chunk length at offset 40
        uint32_t data_len = read_le32(s_buf + 40);
        s_expected = 44 + data_len;
        ESP_LOGI(TAG, "expecting WAV total %u bytes", (unsigned)s_expected);
    }

    // once full WAV received → play
    if (s_expected > 0 && s_filled >= s_expected) {
        ESP_LOGI(TAG, "full WAV received (%u bytes), playing…", (unsigned)s_filled);

        // skip 44-byte header
        const uint8_t *pcm    = s_buf + 44;
        size_t        pcm_bytes = s_expected - 44;

        size_t written = 0;
        // blocking write to I2S
        ESP_ERROR_CHECK(i2s_write(I2S_SPK_PORT, pcm, pcm_bytes, &written, portMAX_DELAY));
        ESP_LOGI(TAG, "i2s wrote %u PCM bytes", (unsigned)written);

        // reset for next transmission
        s_filled   = 0;
        s_expected = 0;
    }
}
