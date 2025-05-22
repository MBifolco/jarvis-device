/* audio_rx.c */
#include "audio_rx.h"
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "driver/i2s.h"

static const char *TAG = "audio_rx";

// I2S configuration
#define I2S_SPK_PORT      I2S_NUM_1
#define SAMPLE_RATE       24000
#define BITS_PER_SAMPLE   16
#define CHANNELS          1

// ADPCM decoder state
static int16_t  adpcm_pred;    // predictor (previous PCM sample)
static int      adpcm_index;   // step index

// IMA ADPCM tables
static const int step_table[89] = {
     7, 8, 9, 10, 11, 12, 13, 14, 16, 17,
    19, 21, 23, 25, 28, 31, 34, 37, 41, 45,
    50, 55, 60, 66, 73, 80, 88, 97, 107,118,
   130,143,157,173,190,209,230,253,279,307,
   337,371,408,449,494,544,598,658,724,796,
   876,963,1060,1166,1282,1411,1552,1707,1878,2066,
   2272,2499,2749,3024,3327,3660,4026,4428,4871,5358,
   5894,6484,7132,7845,8630,9493,10442,11487,12635,13899,
   15289,16818,18500,20350,22385,24623,27086,29794,32767
};
static const int index_table[16] = {
   -1, -1, -1, -1, 2, 4, 6, 8,
   -1, -1, -1, -1, 2, 4, 6, 8
};

void audio_rx_init(void)
{
    // initialize ADPCM state
    adpcm_pred  = 0;
    adpcm_index = 0;

    // configure I2S at a default rate (will be updated if needed)
    i2s_zero_dma_buffer(I2S_SPK_PORT);
    i2s_set_clk(I2S_SPK_PORT, SAMPLE_RATE, BITS_PER_SAMPLE, CHANNELS);
}

// simple integer clamp
static inline int clamp_int(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// Decode a single ADPCM block into PCM samples.
// 'in' is adpcm_data of length 'in_bytes'; 'out' must be large enough.
// Returns number of int16_t samples written.
static size_t decode_adpcm_block(const uint8_t *in, size_t in_bytes,
                                 int16_t *out)
{
    size_t out_count = 0;
    size_t i = 0;

    if (in_bytes < 4) return 0;

    // read initial predictor + index
    adpcm_pred  = (int16_t)((in[1] << 8) | in[0]);
    adpcm_index = in[2] & 0x7F;
    i = 4;

    // emit predictor as first sample
    out[out_count++] = adpcm_pred;

    // for each byte, decode two 4-bit samples
    while (i < in_bytes) {
        uint8_t byte = in[i++];
        for (int shift = 0; shift <= 4; shift += 4) {
            int nibble = (byte >> shift) & 0x0F;
            int step   = step_table[adpcm_index];
            int diff   = step >> 3;
            if (nibble & 1) diff += step >> 2;
            if (nibble & 2) diff += step >> 1;
            if (nibble & 4) diff += step;
            if (nibble & 8) diff = -diff;

            // update predictor and clamp to 16-bit range
            adpcm_pred = (int16_t)clamp_int(adpcm_pred + diff, -32768, 32767);

            // update index and clamp to valid table range
            adpcm_index = clamp_int(adpcm_index + index_table[nibble], 0, 88);

            out[out_count++] = adpcm_pred;
        }
    }
    return out_count;
}

void audio_rx_on_write(const uint8_t *data, size_t len)
{
    ESP_LOGD(TAG, "ADPCM chunk in: %u bytes", (unsigned)len);

    // temp buffer for this chunk's PCM
    int16_t *temp = malloc(sizeof(int16_t) * len * 2);
    if (!temp) {
        ESP_LOGE(TAG, "malloc failed");
        return;
    }

    size_t samples = decode_adpcm_block(data, len, temp);
    if (samples > 0 && (pcm_filled + samples) <= MAX_SAMPLES) {
        memcpy(pcm_accum + pcm_filled, temp, samples * sizeof(int16_t));
        pcm_filled += samples;
    } else if (pcm_filled + samples > MAX_SAMPLES) {
        ESP_LOGW(TAG, "PCM buffer overflow – resetting");
        pcm_filled = 0;
    }
    free(temp);
}

/// Call this *once*, after you've sent the last BLE write,
/// to push the entire accumulated PCM out in one go:
void audio_rx_playback(void)
{
    ESP_LOGI(TAG, "Playing back %u samples in one write", (unsigned)pcm_filled);
    if (pcm_filled == 0) return;

    size_t written = 0;
    ESP_ERROR_CHECK(
      i2s_write(
        I2S_SPK_PORT,
        pcm_accum,
        pcm_filled * sizeof(int16_t),
        &written,
        portMAX_DELAY
      )
    );
    ESP_LOGI(TAG, "I2S wrote %u bytes", (unsigned)written);

    // reset for next stream
    pcm_filled = 0;
}