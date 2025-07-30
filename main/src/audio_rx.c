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
    uint32_t expected = 0;
    bool have_header = false;

    static uint32_t wait_count = 0;
    
    for (;;) {
        // If we don't have a header yet, try to read one
        if (!have_header) {
            // 1) Check StreamBuffer has at least 4 bytes available before trying to read header
            size_t available = xStreamBufferBytesAvailable(sb_adpcm);
            if (available < 4) {
                // Only log occasionally to avoid spam during normal idle periods
                wait_count++;
                if (wait_count % 500 == 0) {  // Log every 5 seconds when waiting
                    ESP_LOGI(TAG, "Waiting for audio data... (%u bytes available)", (unsigned)available);
                }
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            
            // Reset wait counter when we have data
            wait_count = 0;
            
            // 2) Read 4-byte length header
            size_t header_received = xStreamBufferReceive(sb_adpcm, header, 4, pdMS_TO_TICKS(100));
            if (header_received != 4) {
                ESP_LOGE(TAG, "Failed to read complete header - got %u bytes", (unsigned)header_received);
                continue;
            }
            
            expected = ((uint32_t)header[0])
                     | ((uint32_t)header[1] << 8)
                     | ((uint32_t)header[2] << 16)
                     | ((uint32_t)header[3] << 24);

            ESP_LOGI(TAG, "RX task: expecting %" PRIu32 " audio bytes", expected);
            
            // Log the header bytes for debugging
            ESP_LOGI(TAG, "Header bytes: [0x%02X, 0x%02X, 0x%02X, 0x%02X]", 
                    header[0], header[1], header[2], header[3]);
            
            have_header = true;  // Mark that we have a valid header
        }

        // Validate data length is reasonable (valid PCM lengths are typically 48KB-480KB)
        if (expected > 500000 || expected == 0) {
            ESP_LOGE(TAG, "Invalid audio length: %" PRIu32 " bytes - header corruption detected!", expected);
            
            // Instead of resetting, try to find a valid header by reading byte by byte
            ESP_LOGI(TAG, "Searching for valid header sync...");
            bool found_sync = false;
            int search_attempts = 0;
            const int MAX_SEARCH_BYTES = 2000; // Don't search forever
            
            while (search_attempts < MAX_SEARCH_BYTES && !found_sync) {
                // Read one byte to advance position
                uint8_t discard;
                size_t discarded = xStreamBufferReceive(sb_adpcm, &discard, 1, pdMS_TO_TICKS(10));
                if (discarded == 0) {
                    ESP_LOGW(TAG, "No more data during sync search");
                    break;
                }
                search_attempts++;
                
                // Check if we have enough bytes for a new header
                if (xStreamBufferBytesAvailable(sb_adpcm) >= 4) {
                    // Peek at the next 4 bytes without consuming them
                    uint8_t peek_header[4];
                    size_t peeked = xStreamBufferReceive(sb_adpcm, peek_header, 4, pdMS_TO_TICKS(10));
                    if (peeked == 4) {
                        uint32_t peek_length = ((uint32_t)peek_header[0])
                                             | ((uint32_t)peek_header[1] << 8)
                                             | ((uint32_t)peek_header[2] << 16)
                                             | ((uint32_t)peek_header[3] << 24);
                        
                        // Check if this looks like a valid header (reasonable audio length)
                        if (peek_length > 1000 && peek_length <= 500000) {
                            ESP_LOGI(TAG, "Found potential valid header: %" PRIu32 " bytes after skipping %d bytes", 
                                    peek_length, search_attempts);
                            // Use this header
                            memcpy(header, peek_header, 4);
                            expected = peek_length;
                            found_sync = true;
                        } else {
                            // Put the header back and continue searching
                            // Reset buffer and put back what we peeked plus what we haven't processed
                            size_t remaining = xStreamBufferBytesAvailable(sb_adpcm);
                            uint8_t *temp_buf = malloc(remaining + 4);
                            if (temp_buf) {
                                memcpy(temp_buf, peek_header, 4);
                                if (remaining > 0) {
                                    xStreamBufferReceive(sb_adpcm, temp_buf + 4, remaining, pdMS_TO_TICKS(100));
                                }
                                xStreamBufferReset(sb_adpcm);
                                xStreamBufferSend(sb_adpcm, temp_buf, remaining + 4, pdMS_TO_TICKS(100));
                                free(temp_buf);
                            }
                        }
                    }
                }
            }
            
            if (!found_sync) {
                ESP_LOGE(TAG, "Could not find valid header sync after %d bytes - resetting buffer", search_attempts);
                xStreamBufferReset(sb_adpcm);
                have_header = false;  // Reset header state
                continue;
            }
        }
        
        // 3) Check that StreamBuffer has enough data for the expected audio
        size_t available = xStreamBufferBytesAvailable(sb_adpcm);
        if (available < expected) {
            ESP_LOGW(TAG, "Not enough data - need %" PRIu32 " bytes, have %u bytes. Waiting...", 
                    expected, (unsigned)available);
            
            // Keep the header but wait for more data
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;  // This will skip reading a new header since have_header is still true
        }

        ESP_LOGI(TAG, "StreamBuffer ready: %" PRIu32 " bytes needed, %u bytes available", 
                expected, (unsigned)available);

        // 4) Process as uncompressed PCM (compression removed)
        ESP_LOGI(TAG, "START processing audio packet: %" PRIu32 " bytes", expected);
        uint32_t start_time = xTaskGetTickCount();
        
        process_uncompressed_audio(expected);
        
        uint32_t end_time = xTaskGetTickCount();
        ESP_LOGI(TAG, "FINISHED processing audio packet: took %u ms", 
                (unsigned)((end_time - start_time) * portTICK_PERIOD_MS));
        
        // Reset header state after processing
        have_header = false;
        ESP_LOGI(TAG, "Ready for next audio packet");
    }
}

// process_compressed_audio removed - no longer supporting compression

/**
 * @brief Process uncompressed PCM audio - ensure exact byte consumption for StreamBuffer alignment
 */
static void process_uncompressed_audio(uint32_t expected)
{
    ESP_LOGI(TAG, "Processing uncompressed PCM audio (%d bytes = %.1f seconds)", 
             (int)expected, expected / (float)(SAMPLE_RATE * 2));
    
    const size_t CHUNK_BYTES = SAMPLE_RATE * 2;  // 1 second = 48KB at 24kHz 16-bit
    
    // Read all data in one go to ensure exact consumption
    uint8_t *audio_data = heap_caps_malloc(expected, MALLOC_CAP_SPIRAM);
    if (!audio_data) {
        ESP_LOGE(TAG, "Failed to allocate buffer for %d bytes", (int)expected);
        // Drain data to avoid blocking sender - must consume exact amount
        uint8_t drain[1024];
        size_t total_drained = 0;
        while (total_drained < expected) {
            size_t to_drain = (expected - total_drained > sizeof(drain)) ? sizeof(drain) : (expected - total_drained);
            size_t drained = xStreamBufferReceive(sb_adpcm, drain, to_drain, portMAX_DELAY);
            total_drained += drained;
            if (drained == 0) {
                ESP_LOGE(TAG, "StreamBuffer empty but expected more data!");
                break;
            }
        }
        ESP_LOGI(TAG, "Drained exactly %d bytes (expected %d)", (int)total_drained, (int)expected);
        return;
    }
    
    // Read exactly the expected number of bytes
    size_t total_received = 0;
    while (total_received < expected) {
        size_t remaining = expected - total_received;
        size_t to_read = (remaining > 4096) ? 4096 : remaining;  // Read in 4KB chunks max
        
        size_t received = xStreamBufferReceive(
            sb_adpcm,
            audio_data + total_received,
            to_read,
            portMAX_DELAY
        );
        
        if (received == 0) {
            ESP_LOGE(TAG, "StreamBuffer empty! Received %d/%d bytes", (int)total_received, (int)expected);
            break;
        }
        
        total_received += received;
        
        // Log progress for long audio
        if (total_received % (SAMPLE_RATE * 4) == 0) {  // Every ~2 seconds
            ESP_LOGI(TAG, "Received %d/%d bytes (%.1f%%)", 
                    (int)total_received, (int)expected, 
                    100.0f * total_received / expected);
        }
    }
    
    ESP_LOGI(TAG, "Read exactly %d bytes from StreamBuffer", (int)total_received);
    
    // Verify we got exactly what we expected
    if (total_received != expected) {
        ESP_LOGE(TAG, "StreamBuffer misalignment! Expected %d bytes, got %d bytes", 
                (int)expected, (int)total_received);
        heap_caps_free(audio_data);
        return;
    }
    
    // Now process the audio data in chunks for streaming playback
    size_t offset = 0;
    int chunk_num = 1;
    
    while (offset < total_received) {
        size_t chunk_bytes = (total_received - offset >= CHUNK_BYTES) ? CHUNK_BYTES : (total_received - offset);
        size_t chunk_samples = chunk_bytes / sizeof(int16_t);
        
        int16_t *chunk_data = allocate_chunk();
        if (chunk_data && chunk_samples <= SAMPLE_RATE) {
            memcpy(chunk_data, audio_data + offset, chunk_bytes);
            pcm_chunk_t chunk = { .data = chunk_data, .samples = chunk_samples };
            
            if (xQueueSend(pcm_queue, &chunk, pdMS_TO_TICKS(200)) == pdTRUE) {
                ESP_LOGI(TAG, "Queued chunk %d: %.2f seconds (%d bytes, offset %d)", 
                        chunk_num++, chunk_samples / (float)SAMPLE_RATE,
                        (int)chunk_bytes, (int)offset);
                offset += chunk_bytes;
            } else {
                ESP_LOGW(TAG, "PCM queue full, waiting for space");
                free_chunk(chunk_data);
                vTaskDelay(pdMS_TO_TICKS(50));  // Wait longer for queue space
                // Don't increment offset - try again
            }
        } else {
            ESP_LOGW(TAG, "Chunk pool exhausted - waiting for free chunks");
            vTaskDelay(pdMS_TO_TICKS(50));
            // Don't increment offset - try again
        }
        
        // Yield to prevent watchdog timeout
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    heap_caps_free(audio_data);
    ESP_LOGI(TAG, "Completed processing %d chunks (%.1f seconds total)", 
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

