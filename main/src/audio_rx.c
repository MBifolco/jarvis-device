/* SPDX-FileCopyrightText: 2024 …
   SPDX-License-Identifier: Unlicense OR CC0-1.0 */

#include "audio_rx.h"
#include "esp_log.h"
#include "driver/i2s.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "esp_heap_caps.h"    // for MALLOC_CAP_SPIRAM
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

static const char *TAG = "audio_rx";

/* I2S + ADPCM → PCM parameters */
#define I2S_SPK_PORT      I2S_NUM_1
#define SAMPLE_RATE       24000
#define BITS_PER_SAMPLE   16
#define CHANNELS          1

/* Maximum capture length your app ever sends (seconds) */
#define MAX_SECONDS       5
#define MAX_PCM_SAMPLES   (SAMPLE_RATE * MAX_SECONDS)

/* StreamBuffer must hold entire compressed packet + 4-byte header */
#define MAX_ADPCM_BYTES   (SAMPLE_RATE * CHANNELS * MAX_SECONDS / 2 + 4)
#define SB_SIZE_BYTES     (MAX_ADPCM_BYTES * 2)  // double for safety
#define SB_TRIGGER_LEVEL  1

/* debug flag */
bool g_playing_back = false;

/* the one StreamBuffer for raw ADPCM (incl. header) */
static StreamBufferHandle_t    sb_adpcm          = NULL;
// storage & control for that StreamBuffer, allocated in PSRAM
static uint8_t               *sb_adpcm_mem      = NULL;
static StaticStreamBuffer_t  *sb_adpcm_struct   = NULL;


/* PCM accumulator buffer */
static int16_t *pcm_accum = NULL;

static uint8_t *adpcm_buf;

/* prototype */
static void rx_task(void *arg);
static size_t decode_adpcm_block(const uint8_t *in, size_t in_bytes, int16_t *out);
static void play_pcm_chunk(const int16_t *pcm, size_t nsamps);


// IMA-ADPCM step size table
static const int step_table[89] = {
     7,   8,   9,  10,  11,  12,  13,  14,  16,  17,  19,  21,  23,  25,  28,  31,
    34,  37,  41,  45,  50,  55,  60,  66,  73,  80,  88,  97, 107, 118, 130, 143,
   157, 173, 190, 209, 230, 253, 279, 307, 337, 371, 408, 449, 494, 544, 598, 658,
   724, 796, 876, 963,1060,1166,1282,1411,1552,1707,1878,2066,2272,2499,2749,3024,
  3327,3660,4026,4428,4871,5358,5894,6484,7132,7845,8630,9493,10442,11487,12635,13899,
 15289,16818,18500,20350,22385,24623,27086,29794,32767
};

// IMA-ADPCM index adjustment table
static const int index_table[16] = {
   -1, -1, -1, -1,  2,  4,  6,  8,
   -1, -1, -1, -1,  2,  4,  6,  8
};

/**
 * @brief Set up buffers + spawn the RX task.
 */
esp_err_t audio_rx_init(void)
{
    /* 1) PCM decode buffer — allocate in PSRAM */
    pcm_accum = heap_caps_malloc(
        sizeof(int16_t) * MAX_PCM_SAMPLES,
        MALLOC_CAP_SPIRAM
    );
    if (!pcm_accum) {
        ESP_LOGE(TAG, "PSRAM alloc for pcm_accum failed");
        return ESP_ERR_NO_MEM;
    }

    /* 2) Raw ADPCM StreamBuffer — allocate its RAM & control struct in PSRAM */
    sb_adpcm_mem    = heap_caps_malloc(SB_SIZE_BYTES,        MALLOC_CAP_SPIRAM);
    sb_adpcm_struct = heap_caps_malloc(sizeof(StaticStreamBuffer_t),
                                       MALLOC_CAP_SPIRAM);
    if (!sb_adpcm_mem || !sb_adpcm_struct) {
        ESP_LOGE(TAG, "PSRAM alloc for StreamBuffer failed");
        heap_caps_free(pcm_accum);
        heap_caps_free(sb_adpcm_mem);
        heap_caps_free(sb_adpcm_struct);
        return ESP_ERR_NO_MEM;
    }
    sb_adpcm = xStreamBufferCreateStatic(
        SB_SIZE_BYTES,
        SB_TRIGGER_LEVEL,
        sb_adpcm_mem,
        sb_adpcm_struct
    );
    if (!sb_adpcm) {
        ESP_LOGE(TAG, "xStreamBufferCreateStatic failed");
        heap_caps_free(pcm_accum);
        heap_caps_free(sb_adpcm_mem);
        heap_caps_free(sb_adpcm_struct);
        return ESP_ERR_NO_MEM;
    }

    adpcm_buf = heap_caps_malloc(MAX_ADPCM_BYTES, MALLOC_CAP_SPIRAM);
    if (!adpcm_buf) {
        ESP_LOGE(TAG, "PSRAM alloc for adpcm_buf failed");
        /* cleanup and return ESP_ERR_NO_MEM */
        heap_caps_free(pcm_accum);
        heap_caps_free(sb_adpcm_mem);
        heap_caps_free(sb_adpcm_struct);
        return ESP_ERR_NO_MEM;
    }

    /* 3) Start the background RX/decoder task */
    xTaskCreate(
        rx_task, "audio_rx", 4096,
        NULL, tskIDLE_PRIORITY + 1, NULL
    );

    i2s_zero_dma_buffer(I2S_SPK_PORT);
    i2s_set_clk(
        I2S_SPK_PORT,
        SAMPLE_RATE,      // now 24 kHz again
        BITS_PER_SAMPLE,
        CHANNELS
    );

    return ESP_OK;
}

/**
 * @brief Enqueue BLE‐received bytes. Block up to 100 ms if full.
 */
esp_err_t audio_rx_on_write(const uint8_t *data, size_t len)
{
    size_t sent = xStreamBufferSend(sb_adpcm, data, len,
                                   pdMS_TO_TICKS(100));
    if (sent < len) {
        ESP_LOGW(TAG, "StreamBuffer overflow dropped %u bytes",
                 (unsigned)(len - sent));
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

/**
 * @brief Task that:
 *  - pulls exactly 4 bytes → reads total ADPCM length
 *  - allocates a local buffer, pulls “length” bytes into it
 *  - decodes into pcm_accum[]
 *  - streams small PCM‐chunks to I2S
 */
static void rx_task(void *arg)
{
    uint8_t header[4];

    for (;;) {
        // 1) read 4-byte length header
        xStreamBufferReceive(sb_adpcm, header, 4, portMAX_DELAY);
        uint32_t expected = ((uint32_t)header[0])
                          | ((uint32_t)header[1] << 8)
                          | ((uint32_t)header[2] << 16)
                          | ((uint32_t)header[3] << 24);

        ESP_LOGI(TAG, "RX task: expecting %" PRIu32 " ADPCM bytes", expected);

        // 2) too large? drain and skip
        if (expected > MAX_ADPCM_BYTES) {
            ESP_LOGW(TAG, "Packet too big (%" PRIu32 "), skipping", expected);
            size_t to_drain = expected;
            while (to_drain) {
                uint8_t tmp[256];
                size_t chunk = to_drain > sizeof(tmp) ? sizeof(tmp) : to_drain;
                size_t r = xStreamBufferReceive(sb_adpcm, tmp, chunk, portMAX_DELAY);
                to_drain -= r;
            }
            continue;
        }

        // 3) read the full ADPCM payload into our pre-alloc’d buffer
        size_t rec = 0;
        while (rec < expected) {
            rec += xStreamBufferReceive(
                sb_adpcm,
                adpcm_buf + rec,
                expected - rec,
                portMAX_DELAY
            );
        }

        // 4) DECODE ALL BLOCKS into pcm_accum[]
        size_t pcm_count = 0;
        size_t off = 0;
        while (off < expected) {
            size_t blk = (expected - off > 256) ? 256 : (expected - off);
            size_t got = decode_adpcm_block(
                adpcm_buf + off,
                blk,
                pcm_accum + pcm_count
            );
            pcm_count += got;
            off       += blk;
        }

        // 5) PLAY it back in one go (with internal chunking for DMA)
        if (pcm_count) {
            play_pcm_chunk(pcm_accum, pcm_count);
        }
    }
}

/*———————————————————————————————*
 |  IMA-ADPCM decoder & I2S writer |
 *———————————————————————————————*/

// clamp helper
static inline int clamp_int(int v, int lo, int hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static size_t decode_adpcm_block(const uint8_t *in, size_t in_bytes,
                                 int16_t *out)
{
    if (in_bytes < 4) return 0;
    size_t out_cnt = 0;
    int16_t pred   = (int16_t)((in[1] << 8) | in[0]);
    int     idx    = in[2] & 0x7F;
    out[out_cnt++] = pred;

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
            out[out_cnt++] = pred;
        }
    }
    return out_cnt;
}

static void play_pcm_chunk(const int16_t *pcm, size_t nsamps)
{
    g_playing_back = true;

    const uint8_t *data       = (const uint8_t*)pcm;
    size_t         total_bytes = nsamps * sizeof(int16_t);
    size_t         offset      = 0;
    const size_t   CHUNK       = 1024;  // 1 KB - smaller chunks for better flow control

    while (offset < total_bytes) {
        size_t to_write = (total_bytes - offset > CHUNK)
                            ? CHUNK
                            : (total_bytes - offset);
        size_t written = 0;
        esp_err_t err = i2s_write(
            I2S_SPK_PORT,
            data + offset,
            to_write,
            &written,
            pdMS_TO_TICKS(100)     // longer timeout for DMA buffer to drain
        );
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "i2s_write error: %s", esp_err_to_name(err));
            break;
        }
        if (written < to_write) {
            ESP_LOGW(TAG, "i2s_write wrote only %u/%u bytes",
                     (unsigned)written, (unsigned)to_write);
        }
        offset += written;
        // yield so BLE/VAD tasks don’t starve
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    g_playing_back = false;
}

