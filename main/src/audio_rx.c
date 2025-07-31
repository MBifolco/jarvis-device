/* SPDX-FileCopyrightText: 2024 …
   SPDX-License-Identifier: Unlicense OR CC0-1.0 */

#include "audio_rx.h"
#include "config.h"
#include "esp_log.h"
#include "driver/i2s.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"    // for MALLOC_CAP_SPIRAM
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

static const char *TAG = "audio_rx";

/* I2S + PCM parameters */
#define I2S_SPK_PORT      I2S_NUM_1
#define SAMPLE_RATE       24000
#define BITS_PER_SAMPLE   16
#define CHANNELS          1

/* Chunk size matching app (48KB = 1 second at 24kHz) */
#define CHUNK_SIZE_BYTES  48000
#define CHUNK_SIZE_SAMPLES (CHUNK_SIZE_BYTES / sizeof(int16_t))

/* debug flag */
bool g_playing_back = false;

/* Reception state machine */
typedef enum {
    RX_STATE_WAITING_HEADER,
    RX_STATE_RECEIVING_AUDIO
} rx_state_t;

static struct {
    rx_state_t state;
    int16_t* current_chunk;     // Current chunk being filled by BLE
    size_t bytes_received;      // Bytes received in current chunk
    size_t expected_bytes;      // Expected bytes from header
    uint8_t header_buffer[4];   // Buffer for header bytes
    size_t header_bytes;        // Header bytes received so far
} rx_state = {
    .state = RX_STATE_WAITING_HEADER,
    .current_chunk = NULL,
    .bytes_received = 0,
    .expected_bytes = 0,
    .header_bytes = 0
};

/* PCM playback queue - sends decoded PCM chunks to playback task */
static QueueHandle_t pcm_queue = NULL;

/* Pre-allocated chunk pool to avoid fragmentation */
#define CHUNK_POOL_SIZE 12  // 12 chunks of 48KB each
static int16_t *chunk_pool[CHUNK_POOL_SIZE];
static bool chunk_pool_used[CHUNK_POOL_SIZE];
static SemaphoreHandle_t chunk_pool_mutex = NULL;

/* PCM chunk for queue */
typedef struct {
    int16_t *data;
    size_t samples;
} pcm_chunk_t;

/* prototype */
static void playback_task(void *arg);
static void play_pcm_chunk(const int16_t *pcm, size_t nsamps);
static int16_t* allocate_chunk(void);
static void free_chunk(int16_t* chunk);
static void process_header_byte(uint8_t byte);
static void process_audio_data(const uint8_t *data, size_t len);

/**
 * @brief Set up buffers + spawn the playback task.
 */
esp_err_t audio_rx_init(void)
{
    /* 1) Create mutex for chunk pool access */
    chunk_pool_mutex = xSemaphoreCreateMutex();
    if (!chunk_pool_mutex) {
        ESP_LOGE(TAG, "Failed to create chunk pool mutex");
        return ESP_ERR_NO_MEM;
    }

    /* 2) Initialize chunk pool with 48KB chunks */
    for (int i = 0; i < CHUNK_POOL_SIZE; i++) {
        chunk_pool[i] = heap_caps_malloc(CHUNK_SIZE_BYTES, MALLOC_CAP_SPIRAM);
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
    ESP_LOGI(TAG, "Allocated chunk pool: %d chunks x %d bytes each", CHUNK_POOL_SIZE, CHUNK_SIZE_BYTES);

    /* 3) Create PCM queue for sending chunks to playback task */
    pcm_queue = xQueueCreate(CHUNK_POOL_SIZE, sizeof(pcm_chunk_t));
    if (!pcm_queue) {
        ESP_LOGE(TAG, "Failed to create PCM queue");
        return ESP_ERR_NO_MEM;
    }

    /* 4) Start the playback task - highest priority for uninterrupted I2S writes */
    xTaskCreate(
        playback_task, "audio_play", 4096,
        NULL, 7, NULL  // Highest priority 7
    );

    /* 5) Initialize reception state */
    rx_state.state = RX_STATE_WAITING_HEADER;
    rx_state.current_chunk = NULL;
    rx_state.bytes_received = 0;
    rx_state.expected_bytes = 0;
    rx_state.header_bytes = 0;

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
 * @brief Handle BLE data - state machine processes header/audio directly
 */
esp_err_t audio_rx_on_write(const uint8_t *data, size_t len)
{
    size_t offset = 0;
    
    while (offset < len) {
        if (rx_state.state == RX_STATE_WAITING_HEADER) {
            // Process header bytes one at a time
            while (offset < len && rx_state.header_bytes < 4) {
                process_header_byte(data[offset++]);
            }
        } else if (rx_state.state == RX_STATE_RECEIVING_AUDIO) {
            // Process audio data
            size_t remaining = len - offset;
            process_audio_data(&data[offset], remaining);
            offset += remaining;
        }
    }
    
    return ESP_OK;
}

/**
 * @brief Process a single header byte
 */
static void process_header_byte(uint8_t byte)
{
    rx_state.header_buffer[rx_state.header_bytes++] = byte;
    
    if (rx_state.header_bytes == 4) {
        // Parse complete header
        rx_state.expected_bytes = ((uint32_t)rx_state.header_buffer[0])
                                | ((uint32_t)rx_state.header_buffer[1] << 8)
                                | ((uint32_t)rx_state.header_buffer[2] << 16)
                                | ((uint32_t)rx_state.header_buffer[3] << 24);
        
        ESP_LOGI(TAG, "Received header: [0x%02X, 0x%02X, 0x%02X, 0x%02X] = %u bytes",
                rx_state.header_buffer[0], rx_state.header_buffer[1],
                rx_state.header_buffer[2], rx_state.header_buffer[3],
                (unsigned)rx_state.expected_bytes);
        
        // Validate expected size (48KB chunks)
        if (rx_state.expected_bytes != CHUNK_SIZE_BYTES &&
            (rx_state.expected_bytes < 1000 || rx_state.expected_bytes > 50000)) {
            ESP_LOGE(TAG, "Invalid audio length: %u bytes - expected %d bytes",
                    (unsigned)rx_state.expected_bytes, CHUNK_SIZE_BYTES);
            // Reset state
            rx_state.state = RX_STATE_WAITING_HEADER;
            rx_state.header_bytes = 0;
            return;
        }
        
        // Allocate chunk for audio data
        rx_state.current_chunk = allocate_chunk();
        if (!rx_state.current_chunk) {
            ESP_LOGE(TAG, "No free chunks available - dropping packet");
            rx_state.state = RX_STATE_WAITING_HEADER;
            rx_state.header_bytes = 0;
            return;
        }
        
        // Transition to receiving audio
        rx_state.state = RX_STATE_RECEIVING_AUDIO;
        rx_state.bytes_received = 0;
        ESP_LOGI(TAG, "Ready to receive %u bytes of audio data", (unsigned)rx_state.expected_bytes);
    }
}

/**
 * @brief Process audio data bytes
 */
static void process_audio_data(const uint8_t *data, size_t len)
{
    if (!rx_state.current_chunk) {
        ESP_LOGE(TAG, "No current chunk in audio state!");
        rx_state.state = RX_STATE_WAITING_HEADER;
        rx_state.header_bytes = 0;
        return;
    }
    
    // Calculate how much data we can copy
    size_t remaining_in_chunk = rx_state.expected_bytes - rx_state.bytes_received;
    size_t to_copy = (len < remaining_in_chunk) ? len : remaining_in_chunk;
    
    // Copy data directly to chunk as int16_t samples
    uint8_t *dest = ((uint8_t*)rx_state.current_chunk) + rx_state.bytes_received;
    memcpy(dest, data, to_copy);
    rx_state.bytes_received += to_copy;
    
    ESP_LOGD(TAG, "Received %u bytes, total %u/%u", 
            (unsigned)to_copy, (unsigned)rx_state.bytes_received, (unsigned)rx_state.expected_bytes);
    
    // Check if chunk is complete
    if (rx_state.bytes_received >= rx_state.expected_bytes) {
        ESP_LOGI(TAG, "Audio chunk complete: %u bytes", (unsigned)rx_state.bytes_received);
        
        // Queue chunk for playback
        pcm_chunk_t chunk = {
            .data = rx_state.current_chunk,
            .samples = rx_state.bytes_received / sizeof(int16_t)
        };
        
        if (xQueueSend(pcm_queue, &chunk, pdMS_TO_TICKS(100)) != pdTRUE) {
            ESP_LOGW(TAG, "PCM queue full - dropping chunk");
            free_chunk(rx_state.current_chunk);
        }
        
        // Reset state for next packet
        rx_state.state = RX_STATE_WAITING_HEADER;
        rx_state.current_chunk = NULL;
        rx_state.bytes_received = 0;
        rx_state.expected_bytes = 0;
        rx_state.header_bytes = 0;
    }
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
 |  I2S writer                    |
 *———————————————————————————————*/

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
        // yield so BLE/VAD tasks don't starve
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    g_playing_back = false;
}

/**
 * @brief Allocate a chunk from the pre-allocated pool
 */
static int16_t* allocate_chunk(void)
{
    int16_t* chunk = NULL;
    
    if (xSemaphoreTake(chunk_pool_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        for (int i = 0; i < CHUNK_POOL_SIZE; i++) {
            if (!chunk_pool_used[i]) {
                chunk_pool_used[i] = true;
                chunk = chunk_pool[i];
                break;
            }
        }
        xSemaphoreGive(chunk_pool_mutex);
    }
    
    return chunk;
}

/**
 * @brief Return a chunk to the pool
 */
static void free_chunk(int16_t* chunk)
{
    if (xSemaphoreTake(chunk_pool_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        for (int i = 0; i < CHUNK_POOL_SIZE; i++) {
            if (chunk_pool[i] == chunk) {
                chunk_pool_used[i] = false;
                xSemaphoreGive(chunk_pool_mutex);
                return;
            }
        }
        xSemaphoreGive(chunk_pool_mutex);
        // Should never happen - log error
        ESP_LOGE(TAG, "Attempted to free unknown chunk %p", chunk);
    }
}