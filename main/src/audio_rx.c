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

/* StreamBuffer sizing for uncompressed PCM streaming */
#define STREAM_BUFFER_SECONDS 3  // Only need 3 seconds for BLE smoothing
#define STREAM_BUFFER_PCM_BYTES (SAMPLE_RATE * CHANNELS * STREAM_BUFFER_SECONDS * 2)  // 3s of PCM
#define SB_SIZE_BYTES     (STREAM_BUFFER_PCM_BYTES * 2)  // Double buffer for safety
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

// audio_buf removed - no longer needed for compression

/* PCM chunk for queue */
typedef struct {
    int16_t *data;
    size_t samples;
} pcm_chunk_t;

/* prototype */
static void rx_task(void *arg);
static void playback_task(void *arg);
static void process_uncompressed_audio(uint32_t expected);
static void play_pcm_chunk(const int16_t *pcm, size_t nsamps);
static int16_t* allocate_chunk(void);
static void free_chunk(int16_t* chunk);


// ADPCM lookup tables removed - no longer supporting compression

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

    // audio_buf removed - no longer needed for compression

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
        SAMPLE_RATE,      // 24 kHz from OpenAI
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



        // 2) Always process as uncompressed PCM (compression removed)
        process_uncompressed_audio(expected);
    }
}

// process_compressed_audio removed - no longer supporting compression

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
        if (total_received % (SAMPLE_RATE * 10) == 0) {  // Every ~5 seconds at 24kHz
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

// decode_adpcm_block removed - no longer supporting compression

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

