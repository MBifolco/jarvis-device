#include "audio_tx.h"
#include "esp_log.h"
#include "host/ble_hs.h"
#include "os/os_mbuf.h"
#include "gatt_svc.h"  // for chr_conn_handle and audio_chr_handle

#define TAG "audio_tx"
#define BLE_CHUNK_SIZE 180
#define WAV_HEADER_SIZE 44

extern uint16_t audio_chr_handle;  // declared in gatt_svc.c
extern int chr_conn_handle;

// Build a standard WAV header (16-bit, mono, 16kHz, PCM)
static void build_wav_header(uint8_t *header, size_t pcm_len_bytes)
{
    uint32_t sample_rate = 16000;
    uint16_t bits_per_sample = 16;
    uint16_t channels = 1;
    uint32_t byte_rate = sample_rate * channels * (bits_per_sample / 8);
    uint16_t block_align = channels * (bits_per_sample / 8);
    uint32_t data_chunk_size = pcm_len_bytes;
    uint32_t fmt_chunk_size = 16;
    uint32_t wav_size = 4 + (8 + fmt_chunk_size) + (8 + data_chunk_size);

    memcpy(header, "RIFF", 4);
    memcpy(header + 4, &wav_size, 4);
    memcpy(header + 8, "WAVE", 4);
    memcpy(header + 12, "fmt ", 4);
    memcpy(header + 16, &fmt_chunk_size, 4);
    uint16_t audio_format = 1;
    memcpy(header + 20, &audio_format, 2);
    memcpy(header + 22, &channels, 2);
    memcpy(header + 24, &sample_rate, 4);
    memcpy(header + 28, &byte_rate, 4);
    memcpy(header + 32, &block_align, 2);
    memcpy(header + 34, &bits_per_sample, 2);
    memcpy(header + 36, "data", 4);
    memcpy(header + 40, &data_chunk_size, 4);
}

esp_err_t audio_tx_send(const uint8_t *pcm, size_t len)
{
    if (!chr_conn_handle || !audio_chr_handle) {
        ESP_LOGW(TAG, "No active BLE connection or audio handle");
        return ESP_FAIL;
    }

    uint8_t header[WAV_HEADER_SIZE];
    build_wav_header(header, len);

    ESP_LOGI(TAG, "Sending WAV header");
    struct os_mbuf *om = ble_hs_mbuf_from_flat(header, WAV_HEADER_SIZE);
    if (!om) {
        ESP_LOGE(TAG, "Failed to allocate BLE buffer for header");
        return ESP_FAIL;
    }

    int rc = ble_gatts_notify_custom(chr_conn_handle, audio_chr_handle, om);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to notify WAV header; rc=%d", rc);
        return ESP_FAIL;
    }

    vTaskDelay(pdMS_TO_TICKS(30));

    ESP_LOGI(TAG, "Sending audio (%d bytes)", (int)len);

    size_t offset = 0;
    while (offset < len) {
        size_t chunk_len = (len - offset > BLE_CHUNK_SIZE)
                               ? BLE_CHUNK_SIZE
                               : (len - offset);

        om = ble_hs_mbuf_from_flat(&pcm[offset], chunk_len);
        if (!om) {
            ESP_LOGE(TAG, "Failed to allocate BLE buffer at offset %d", (int)offset);
            return ESP_FAIL;
        }

        rc = ble_gatts_notify_custom(chr_conn_handle, audio_chr_handle, om);
        if (rc != 0) {
            ESP_LOGE(TAG, "Failed to notify chunk at offset %d; rc=%d", (int)offset, rc);
            return ESP_FAIL;
        }

        offset += chunk_len;
        vTaskDelay(pdMS_TO_TICKS(30));  // moderate throttling
    }

    ESP_LOGI(TAG, "Audio transmission complete (%d bytes + header)", (int)len);
    return ESP_OK;
}
