/* SPDX-FileCopyrightText: 2024 …
   SPDX-License-Identifier: Unlicense OR CC0-1.0 */

#include "audio_rx.h"
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>
#include <inttypes.h>
#include "esp_log.h"
#include "driver/i2s.h"

static const char *TAG = "audio_rx";

// I²S configuration — must match your encoding sample rate
#define I2S_SPK_PORT      I2S_NUM_1
#define SAMPLE_RATE       24000
#define BITS_PER_SAMPLE   16
#define CHANNELS          1

// Max duration we expect (seconds) × rate × channels
#define MAX_SECONDS       10
#define MAX_SAMPLES       (SAMPLE_RATE * MAX_SECONDS)

// Global flag set true while playing back the buffered PCM
bool g_playing_back = false;

// ADPCM framing state
static uint32_t adpcm_expected = 0;
static uint32_t adpcm_received = 0;
static bool     header_parsed  = false;

// Buffers
static uint8_t  *adpcm_buffer = NULL;     // incoming ADPCM stream
static int16_t  *pcm_accum    = NULL;     // decoded PCM
static size_t    pcm_filled   = 0;

// IMA-ADPCM tables
static const int step_table[89] = {
     7,   8,   9,  10,  11,  12,  13,  14,  16,  17,  19,  21,  23,  25,  28,  31,
    34,  37,  41,  45,  50,  55,  60,  66,  73,  80,  88,  97, 107, 118, 130, 143,
   157, 173, 190, 209, 230, 253, 279, 307, 337, 371, 408, 449, 494, 544, 598, 658,
   724, 796, 876, 963,1060,1166,1282,1411,1552,1707,1878,2066,2272,2499,2749,3024,
  3327,3660,4026,4428,4871,5358,5894,6484,7132,7845,8630,9493,10442,11487,12635,13899,
 15289,16818,18500,20350,22385,24623,27086,29794,32767
};
static const int index_table[16] = {
   -1, -1, -1, -1, 2, 4, 6, 8,
   -1, -1, -1, -1, 2, 4, 6, 8
};

// Read 32-bit little-endian length header
static uint32_t read_le32(const uint8_t *p) {
    return ((uint32_t)p[0]) |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static inline int clamp_int(int v, int lo, int hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// Decode one 256-byte (or smaller) ADPCM block → PCM samples
static size_t decode_adpcm_block(const uint8_t *in, size_t in_bytes,
                                 int16_t *out)
{
    if (in_bytes < 4) return 0;
    size_t out_cnt = 0;

    const int16_t gate_threshold = 0;  // tweak this up or down

    int16_t pred = (int16_t)((in[1] << 8) | in[0]);
    int     idx  = in[2] & 0x7F;
    out[out_cnt++] = (abs(pred) < gate_threshold) ? 0 : pred;;

    size_t i = 4;
    while (i < in_bytes) {
        uint8_t b = in[i++];
        for (int shift = 0; shift <= 4; shift += 4) {
            int nib  = (b >> shift) & 0x0F;
            int step = step_table[idx];
            int diff = step >> 3;
            if (nib & 1) diff += step >> 2;
            if (nib & 2) diff += step >> 1;
            if (nib & 4) diff += step;
            if (nib & 8) diff = -diff;

            pred = (int16_t)clamp_int(pred + diff, -32768, 32767);
            idx  = clamp_int(idx + index_table[nib], 0, 88);
            out[out_cnt++] = (abs(pred) < gate_threshold) ? 0 : pred;;
        }
    }
    return out_cnt;
}

// Initialize decoder state, buffers and I²S
void audio_rx_init(void)
{
    header_parsed  = false;
    adpcm_expected = 0;
    adpcm_received = 0;
    pcm_filled     = 0;

    free(adpcm_buffer);
    adpcm_buffer = NULL;

    free(pcm_accum);
    pcm_accum = malloc(sizeof(int16_t) * MAX_SAMPLES);

    // configure I²S TX
    i2s_zero_dma_buffer(I2S_SPK_PORT);
    i2s_set_clk(I2S_SPK_PORT, SAMPLE_RATE, BITS_PER_SAMPLE, CHANNELS);
}

// Playback the entire decoded PCM at once
void audio_rx_playback(void)
{
    if (pcm_filled == 0) return;
    g_playing_back = true;


    ESP_LOGI(TAG, "Playing back %zu samples", pcm_filled);
    size_t written = 0;
    i2s_write(I2S_SPK_PORT,
              pcm_accum,
              pcm_filled * sizeof(int16_t),
              &written,
              portMAX_DELAY);
    ESP_LOGI(TAG, "I2S wrote %zu bytes", written);

    // reset state
    pcm_filled     = 0;
    header_parsed  = false;
    adpcm_expected = 0;
    adpcm_received = 0;
    free(adpcm_buffer);
    adpcm_buffer = NULL;

    g_playing_back = false;
}

// Called on each BLE write chunk
void audio_rx_on_write(const uint8_t *data, size_t len)
{
    size_t offset = 0;

    // 1) parse 4-byte length header
    if (!header_parsed) {
        if (len < 4) {
            return;  // wait for full header
        }
        adpcm_expected = read_le32(data);
        header_parsed  = true;
        adpcm_received = 0;
        pcm_filled     = 0;

        free(adpcm_buffer);
        adpcm_buffer = malloc(adpcm_expected);

        offset = 4;
        ESP_LOGI(TAG, "Expecting %" PRIu32 " ADPCM bytes", adpcm_expected);
    }

    // 2) buffer this chunk
    size_t chunk = len - offset;
    if (adpcm_received + chunk > adpcm_expected) {
        chunk = adpcm_expected - adpcm_received;
    }
    memcpy(adpcm_buffer + adpcm_received, data + offset, chunk);
    adpcm_received += chunk;

    ESP_LOGD(TAG,
             "Received %" PRIu32 "/%" PRIu32 " ADPCM bytes",
             adpcm_received, adpcm_expected);

    // 3) once we have it all, decode & play
    if (adpcm_received >= adpcm_expected) {
        size_t in_off = 0;
        while (in_off < adpcm_expected) {
            size_t blk = (adpcm_expected - in_off >= 256) ? 256 : (adpcm_expected - in_off);
            size_t got = decode_adpcm_block(adpcm_buffer + in_off, blk,
                                            pcm_accum + pcm_filled);
            pcm_filled += got;
            in_off    += blk;
        }
        audio_rx_playback();
    }
}
