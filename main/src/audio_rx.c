/* SPDX-FileCopyrightText: 2024 …
   SPDX-License-Identifier: Unlicense OR CC0-1.0 */

#include "audio_rx.h"
#include "config.h"
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

/* Maximum capture length your app ever sends (seconds) - handle ~17-18 second responses */
#define MAX_SECONDS       18
#define DECODE_CHUNK_SIZE (SAMPLE_RATE * 2)  // 2 seconds of PCM for decode buffer

/* StreamBuffer sizing - much smaller than full audio */
#define MAX_ADPCM_BYTES   (SAMPLE_RATE * CHANNELS * MAX_SECONDS / 2 + 4)  // ~216KB for 18s compressed
#define STREAM_BUFFER_SECONDS 3  // Only need 3 seconds for BLE smoothing
#define STREAM_BUFFER_PCM_BYTES (SAMPLE_RATE * CHANNELS * STREAM_BUFFER_SECONDS * 2)  // 3s of PCM
/* StreamBuffer needs to handle the larger of: full compressed packet OR 3s of uncompressed */
#define SB_SIZE_BYTES     ((MAX_ADPCM_BYTES > STREAM_BUFFER_PCM_BYTES) ? MAX_ADPCM_BYTES * 2 : STREAM_BUFFER_PCM_BYTES * 2)
/* Audio processing buffer only needed for compressed (uncompressed streams directly) */
#define AUDIO_BUF_SIZE    (MAX_ADPCM_BYTES)  // Only for compressed audio buffering
#define SB_TRIGGER_LEVEL  1

/* debug flag */
bool g_playing_back = false;

/* the one StreamBuffer for raw ADPCM (incl. header) */
static StreamBufferHandle_t    sb_adpcm          = NULL;
// storage & control for that StreamBuffer, allocated in PSRAM
static uint8_t               *sb_adpcm_mem      = NULL;
static StaticStreamBuffer_t  *sb_adpcm_struct   = NULL;

/* PCM playback queue - sends decoded PCM chunks to playback task */
static QueueHandle_t pcm_queue = NULL;

/* Pre-allocated chunk pool to avoid fragmentation */
#define CHUNK_POOL_SIZE 12  // 12 seconds of chunks (576KB) - balance between AFE and streaming
static int16_t *chunk_pool[CHUNK_POOL_SIZE];
static bool chunk_pool_used[CHUNK_POOL_SIZE];

/* Dynamic PCM decode buffer - allocated per processing cycle */
static uint8_t *audio_buf;

/* PCM chunk for queue */
typedef struct {
    int16_t *data;
    size_t samples;
} pcm_chunk_t;

/* prototype */
static void rx_task(void *arg);
static void playback_task(void *arg);
static void process_compressed_audio(uint32_t expected);
static void process_uncompressed_audio(uint32_t expected);
static size_t decode_adpcm_block(const uint8_t *in, size_t in_bytes, int16_t *out);
static void play_pcm_chunk(const int16_t *pcm, size_t nsamps);
static int16_t* allocate_chunk(void);
static void free_chunk(int16_t* chunk);


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
    /* 1) No large static PCM buffer needed - we'll allocate dynamically */

    /* 2) Raw ADPCM StreamBuffer — allocate its RAM & control struct in PSRAM */
    sb_adpcm_mem    = heap_caps_malloc(SB_SIZE_BYTES,        MALLOC_CAP_SPIRAM);
    sb_adpcm_struct = heap_caps_malloc(sizeof(StaticStreamBuffer_t),
                                       MALLOC_CAP_SPIRAM);
    if (!sb_adpcm_mem || !sb_adpcm_struct) {
        ESP_LOGE(TAG, "PSRAM alloc for StreamBuffer failed");
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
        heap_caps_free(sb_adpcm_mem);
        heap_caps_free(sb_adpcm_struct);
        return ESP_ERR_NO_MEM;
    }

    audio_buf = heap_caps_malloc(AUDIO_BUF_SIZE, MALLOC_CAP_SPIRAM);
    if (!audio_buf) {
        ESP_LOGE(TAG, "PSRAM alloc for audio_buf failed");
        /* cleanup and return ESP_ERR_NO_MEM */
        heap_caps_free(sb_adpcm_mem);
        heap_caps_free(sb_adpcm_struct);
        return ESP_ERR_NO_MEM;
    }

    /* 3) Initialize chunk pool to avoid fragmentation */
    for (int i = 0; i < CHUNK_POOL_SIZE; i++) {
        chunk_pool[i] = heap_caps_malloc(SAMPLE_RATE * sizeof(int16_t), MALLOC_CAP_SPIRAM);
        if (!chunk_pool[i]) {
            ESP_LOGE(TAG, "Failed to allocate chunk pool entry %d", i);
            // Free already allocated chunks
            for (int j = 0; j < i; j++) {
                heap_caps_free(chunk_pool[j]);
            }
            return ESP_ERR_NO_MEM;
        }
        chunk_pool_used[i] = false;
    }
    ESP_LOGI(TAG, "Allocated chunk pool: %d chunks x %d samples", CHUNK_POOL_SIZE, SAMPLE_RATE);

    /* 4) Create PCM queue for communication between rx and playback tasks */
    pcm_queue = xQueueCreate(16, sizeof(pcm_chunk_t));
    if (!pcm_queue) {
        ESP_LOGE(TAG, "Failed to create PCM queue");
        return ESP_ERR_NO_MEM;
    }

    /* 5) Start the playback task - highest priority for uninterrupted I2S writes */
    xTaskCreate(
        playback_task, "audio_play", 4096,
        NULL, 7, NULL  // Highest priority 7
    );

    /* 6) Start the RX/decoder task - normal priority with larger stack for streaming */
    xTaskCreate(
        rx_task, "audio_rx", 8192,  // Increased from 4096 to 8192 for streaming
        NULL, 5, NULL  // Priority 5 (same as other tasks)
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
 *  - pulls exactly 4 bytes → reads total audio data length
 *  - determines if data is ADPCM (compressed) or PCM (uncompressed)
 *  - decodes ADPCM or directly plays PCM
 *  - streams PCM chunks to I2S
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

        ESP_LOGI(TAG, "RX task: expecting %" PRIu32 " audio bytes", expected);
        
        // Log the header bytes for debugging
        ESP_LOGI(TAG, "Header bytes: [0x%02X, 0x%02X, 0x%02X, 0x%02X]", 
                header[0], header[1], header[2], header[3]);



        // 2) Check config to determine processing method
        bool is_compressed = config_get_compress_incoming();
        
        if (is_compressed) {
            // 3) For compressed audio, check size limit since we buffer it all
            if (expected > MAX_ADPCM_BYTES) {
                ESP_LOGW(TAG, "Compressed packet too big (%" PRIu32 " > %d), skipping", expected, MAX_ADPCM_BYTES);
                size_t to_drain = expected;
                while (to_drain) {
                    uint8_t tmp[256];
                    size_t chunk = to_drain > sizeof(tmp) ? sizeof(tmp) : to_drain;
                    size_t r = xStreamBufferReceive(sb_adpcm, tmp, chunk, portMAX_DELAY);
                    to_drain -= r;
                }
                continue;
            }
            process_compressed_audio(expected);
        } else {
            // 3) For uncompressed audio, no size limit needed since we stream it
            process_uncompressed_audio(expected);
        }
    }
}

/**
 * @brief Process compressed ADPCM audio - receive all data first, then decode
 */
static void process_compressed_audio(uint32_t expected)
{
    ESP_LOGI(TAG, "Processing compressed ADPCM audio");
    
    // Receive all ADPCM data first
    size_t total_received = 0;
    while (total_received < expected) {
        size_t to_read = (expected - total_received > 2048) ? 2048 : (expected - total_received);
        size_t received = xStreamBufferReceive(
            sb_adpcm,
            audio_buf + total_received,
            to_read,
            portMAX_DELAY
        );
        total_received += received;
    }
    
    // Allocate decode buffer for ADPCM processing
    int16_t *decode_buf = heap_caps_malloc(DECODE_CHUNK_SIZE * sizeof(int16_t), MALLOC_CAP_SPIRAM);
    if (!decode_buf) {
        ESP_LOGE(TAG, "Failed to allocate decode buffer");
        return;
    }
    
    // Decode ADPCM and queue PCM chunks
    size_t decode_offset = 0;
    size_t pcm_accumulated = 0;
    int chunk_num = 1;
    const size_t CHUNK_SIZE = SAMPLE_RATE;  // 1 second at 24kHz
    
    while (decode_offset + 256 <= expected) {
        size_t decode_bytes = (expected - decode_offset > 256) ? 256 : (expected - decode_offset);
        if (decode_bytes >= 4 && pcm_accumulated < DECODE_CHUNK_SIZE) {
            size_t pcm_decoded = decode_adpcm_block(
                audio_buf + decode_offset,
                decode_bytes,
                decode_buf + pcm_accumulated
            );
            pcm_accumulated += pcm_decoded;
            decode_offset += decode_bytes;
            
            // If we have 1 second of PCM, queue it
            if (pcm_accumulated >= CHUNK_SIZE) {
                int16_t *chunk_data = heap_caps_malloc(CHUNK_SIZE * sizeof(int16_t), MALLOC_CAP_SPIRAM);
                if (chunk_data) {
                    memcpy(chunk_data, decode_buf, CHUNK_SIZE * sizeof(int16_t));
                    pcm_chunk_t chunk = { .data = chunk_data, .samples = CHUNK_SIZE };
                    
                    if (xQueueSend(pcm_queue, &chunk, pdMS_TO_TICKS(100)) == pdTRUE) {
                        ESP_LOGI(TAG, "Queued ADPCM chunk %d: 1.00 seconds", chunk_num);
                        // Debug first few ADPCM samples for comparison
                        for (int i = 0; i < 5 && i < CHUNK_SIZE; i++) {
                            ESP_LOGI(TAG, "ADPCM sample %d: %d", i+1, chunk_data[i]);
                        }
                        chunk_num++;
                    } else {
                        ESP_LOGW(TAG, "PCM queue full, dropping ADPCM chunk");
                        heap_caps_free(chunk_data);
                        break;
                    }
                    
                    // Shift remaining PCM data
                    pcm_accumulated -= CHUNK_SIZE;
                    if (pcm_accumulated > 0) {
                        memmove(decode_buf, decode_buf + CHUNK_SIZE, pcm_accumulated * sizeof(int16_t));
                    }
                }
            }
        } else {
            break;
        }
    }
    
    // Queue remaining PCM data
    if (pcm_accumulated > 0) {
        int16_t *chunk_data = heap_caps_malloc(pcm_accumulated * sizeof(int16_t), MALLOC_CAP_SPIRAM);
        if (chunk_data) {
            memcpy(chunk_data, decode_buf, pcm_accumulated * sizeof(int16_t));
            pcm_chunk_t chunk = { .data = chunk_data, .samples = pcm_accumulated };
            
            if (xQueueSend(pcm_queue, &chunk, pdMS_TO_TICKS(100)) == pdTRUE) {
                ESP_LOGI(TAG, "Final ADPCM chunk %d: %.2f seconds", 
                        chunk_num, pcm_accumulated / (float)SAMPLE_RATE);
            } else {
                ESP_LOGW(TAG, "PCM queue full, dropping final ADPCM chunk");
                heap_caps_free(chunk_data);
            }
        }
    }
    
    heap_caps_free(decode_buf);
    ESP_LOGI(TAG, "Completed ADPCM decode, total chunks: %d", chunk_num - 1);
}

/**
 * @brief Process uncompressed PCM audio - stream and play chunks as data arrives
 */
static void process_uncompressed_audio(uint32_t expected)
{
    ESP_LOGI(TAG, "Streaming uncompressed PCM audio (%d bytes = %.1f seconds)", 
             (int)expected, expected / (float)(SAMPLE_RATE * 2));
    
    const size_t CHUNK_BYTES = SAMPLE_RATE * 2;  // 1 second = 48KB at 24kHz 16-bit
    const size_t BUFFER_SIZE = CHUNK_BYTES * 2;   // 2-second accumulation buffer
    
    // Allocate accumulation buffer for streaming
    uint8_t *stream_buffer = heap_caps_malloc(BUFFER_SIZE, MALLOC_CAP_SPIRAM);
    if (!stream_buffer) {
        ESP_LOGE(TAG, "Failed to allocate streaming buffer");
        // Drain data to avoid blocking sender
        uint8_t drain[1024];
        size_t total_drained = 0;
        while (total_drained < expected) {
            size_t to_drain = (expected - total_drained > sizeof(drain)) ? sizeof(drain) : (expected - total_drained);
            size_t drained = xStreamBufferReceive(sb_adpcm, drain, to_drain, portMAX_DELAY);
            total_drained += drained;
        }
        return;
    }
    
    size_t total_received = 0;
    size_t buffer_used = 0;
    int chunk_num = 1;
    bool first_chunk_sent = false;
    
    while (total_received < expected) {
        // Receive data into accumulation buffer
        size_t buffer_space = BUFFER_SIZE - buffer_used;
        size_t remaining_data = expected - total_received;
        size_t to_read = (remaining_data > buffer_space) ? buffer_space : remaining_data;
        
        if (to_read > 2048) to_read = 2048;  // Receive in reasonable chunks
        
        size_t received = xStreamBufferReceive(
            sb_adpcm,
            stream_buffer + buffer_used,
            to_read,
            portMAX_DELAY
        );
        
        total_received += received;
        buffer_used += received;
        
        // Yield more frequently for very long audio to prevent watchdog
        if (total_received % (SAMPLE_RATE * 10) == 0) {  // Every ~5 seconds
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        
        // Queue chunks as soon as we have enough data
        while (buffer_used >= CHUNK_BYTES || (total_received >= expected && buffer_used > 0)) {
            size_t chunk_bytes = (buffer_used >= CHUNK_BYTES) ? CHUNK_BYTES : buffer_used;
            size_t chunk_samples = chunk_bytes / sizeof(int16_t);
            
            int16_t *chunk_data = allocate_chunk();
            if (chunk_data && chunk_samples <= SAMPLE_RATE) {
                memcpy(chunk_data, stream_buffer, chunk_bytes);
                pcm_chunk_t chunk = { .data = chunk_data, .samples = chunk_samples };
                
                if (xQueueSend(pcm_queue, &chunk, pdMS_TO_TICKS(100)) == pdTRUE) {
                    ESP_LOGI(TAG, "Streamed chunk %d: %.2f seconds (%d/%d bytes received)", 
                            chunk_num++, chunk_samples / (float)SAMPLE_RATE,
                            (int)total_received, (int)expected);
                    
                    if (!first_chunk_sent) {
                        ESP_LOGI(TAG, "Playback started while still receiving data");
                        first_chunk_sent = true;
                    }
                } else {
                    ESP_LOGW(TAG, "PCM queue full, dropping chunk");
                    heap_caps_free(chunk_data);
                    break;
                }
                
                // Shift remaining data in buffer
                buffer_used -= chunk_bytes;
                if (buffer_used > 0) {
                    memmove(stream_buffer, stream_buffer + chunk_bytes, buffer_used);
                }
                
                // Yield to prevent watchdog timeout
                vTaskDelay(pdMS_TO_TICKS(1));
            } else {
                ESP_LOGW(TAG, "Chunk pool exhausted - waiting for free chunks");
                // Shorter wait to minimize audio gaps
                vTaskDelay(pdMS_TO_TICKS(10));
                // Don't advance buffer - try again with same data
            }
        }
        
        // If buffer is full but we can't make a chunk, we have a problem
        if (buffer_used >= BUFFER_SIZE) {
            ESP_LOGW(TAG, "Buffer overflow, resetting");
            buffer_used = 0;
        }
    }
    
    heap_caps_free(stream_buffer);
    ESP_LOGI(TAG, "Completed streaming %d chunks (%.1f seconds total)", 
             chunk_num - 1, expected / (float)(SAMPLE_RATE * 2));
}

/**
 * @brief Dedicated playback task - highest priority, only does I2S writes
 * This task runs with highest priority to prevent any interruptions during playback
 */
static void playback_task(void *arg)
{
    pcm_chunk_t chunk;
    
    ESP_LOGI(TAG, "Playback task started with highest priority");
    
    for (;;) {
        // Wait for PCM chunks from rx_task
        if (xQueueReceive(pcm_queue, &chunk, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "Playing PCM chunk: %d samples (%.2f seconds)", 
                     chunk.samples, chunk.samples / (float)SAMPLE_RATE);
            
            // Set flag to prevent recording during playback
            g_playing_back = true;
            
            // Play the chunk with dedicated I2S writing
            play_pcm_chunk(chunk.data, chunk.samples);
            
            // Return chunk to pool
            free_chunk(chunk.data);
            
            // Clear flag when done
            g_playing_back = false;
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

/**
 * @brief Allocate a chunk from the pre-allocated pool
 */
static int16_t* allocate_chunk(void)
{
    for (int i = 0; i < CHUNK_POOL_SIZE; i++) {
        if (!chunk_pool_used[i]) {
            chunk_pool_used[i] = true;
            return chunk_pool[i];
        }
    }
    return NULL;  // Pool exhausted
}

/**
 * @brief Return a chunk to the pool
 */
static void free_chunk(int16_t* chunk)
{
    for (int i = 0; i < CHUNK_POOL_SIZE; i++) {
        if (chunk_pool[i] == chunk) {
            chunk_pool_used[i] = false;
            return;
        }
    }
    // Should never happen - log error
    ESP_LOGE(TAG, "Attempted to free unknown chunk %p", chunk);
}

