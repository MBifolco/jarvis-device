#include "audio_tx.h"
#include "esp_log.h"
#include "host/ble_hs.h"
#include "os/os_mbuf.h"
#include "gatt_svc.h"
#include "esp_audio_enc_default.h"
#include "esp_audio_enc_reg.h"
#include "esp_audio_enc.h"
#include "esp_adpcm_enc.h"

#include <string.h>
#include <stdlib.h>

#define TAG "audio_tx"
#define BLE_CHUNK_SIZE 497
#define WAV_HEADER_SIZE 46


static esp_audio_enc_handle_t enc_handle = NULL;
static int in_frame_size = 0;
static int out_frame_size = 0;

// Build a WAV header for ADPCM format
static void build_adpcm_wav_header(uint8_t *header, size_t adpcm_len_bytes)
{
    uint32_t sample_rate = 16000;
    uint16_t channels = 1;
    uint16_t audio_format = 0x11;  // IMA ADPCM
    uint16_t block_align = 256;
    uint16_t bits_per_sample = 4;
    uint16_t extra_size = 2;
    uint32_t byte_rate = (sample_rate * block_align) / 505;
    uint32_t fmt_chunk_size = 20;
    uint32_t wav_size = 36 + adpcm_len_bytes;

    ESP_LOGI(TAG, "ADPCM header: len=%u → bytes @42 = %02x %02x %02x %02x",
        (unsigned)adpcm_len_bytes,
        (unsigned)((adpcm_len_bytes >> 0) & 0xff),
        (unsigned)((adpcm_len_bytes >> 8) & 0xff),
        (unsigned)((adpcm_len_bytes >> 16) & 0xff),
        (unsigned)((adpcm_len_bytes >> 24) & 0xff));

    memcpy(header, "RIFF", 4);
    memcpy(header + 4, &wav_size, 4);
    memcpy(header + 8, "WAVE", 4);
    memcpy(header + 12, "fmt ", 4);
    memcpy(header + 16, &fmt_chunk_size, 4);
    memcpy(header + 20, &audio_format, 2);
    memcpy(header + 22, &channels, 2);
    memcpy(header + 24, &sample_rate, 4);
    memcpy(header + 28, &byte_rate, 4);
    memcpy(header + 32, &block_align, 2);
    memcpy(header + 34, &bits_per_sample, 2);
    memcpy(header + 36, &extra_size, 2);
    memcpy(header + 38, "data", 4);
    memcpy(header + 42, &adpcm_len_bytes, 4);
}

esp_err_t audio_tx_compression_init(void)
{
    esp_err_t err = esp_adpcm_enc_register();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register ADPCM encoder: %s", esp_err_to_name(err));
        return err;
    }

    esp_adpcm_enc_config_t adpcm_cfg = ESP_ADPCM_ENC_CONFIG_DEFAULT();
    adpcm_cfg.sample_rate = 16000;
    adpcm_cfg.channel = 1;
    adpcm_cfg.bits_per_sample = 16;

    esp_audio_enc_config_t enc_cfg = {
        .type = ESP_AUDIO_TYPE_ADPCM,
        .cfg = &adpcm_cfg,
        .cfg_sz = sizeof(adpcm_cfg),
    };

    err = esp_audio_enc_open(&enc_cfg, &enc_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open ADPCM encoder: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_audio_enc_get_frame_size(enc_handle, &in_frame_size, &out_frame_size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get frame size: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "ADPCM encoder initialized (in=%d, out=%d)", in_frame_size, out_frame_size);
    return ESP_OK;
}

esp_err_t audio_tx_send(const uint8_t *pcm, size_t len)
{
    if (!chr_conn_handle || !audio_notify_handle) {
        ESP_LOGW(TAG, "No active BLE connection or audio handle");
        return ESP_FAIL;
    }

    const uint8_t *input_ptr = pcm;
    size_t input_remaining = len;

    uint8_t *inbuf = malloc(in_frame_size);
    uint8_t *outbuf = malloc(out_frame_size);
    if (!inbuf || !outbuf) {
        ESP_LOGE(TAG, "Failed to allocate compression buffers");
        heap_caps_free(inbuf); heap_caps_free(outbuf);
        return ESP_ERR_NO_MEM;
    }

    // Use smaller streaming buffer instead of allocating full size
    const size_t STREAM_BUF_SIZE = 8192;  // 8KB streaming buffer
    uint8_t *adpcm_stream_buf = malloc(STREAM_BUF_SIZE);
    if (!adpcm_stream_buf) {
        ESP_LOGE(TAG, "Failed to allocate ADPCM streaming buffer");
        heap_caps_free(inbuf); heap_caps_free(outbuf);
        return ESP_ERR_NO_MEM;
    }

    // First pass: calculate total ADPCM size for header
    size_t total_adpcm_size = 0;
    const uint8_t *calc_ptr = pcm;
    size_t calc_remaining = len;
    while (calc_remaining >= in_frame_size) {
        total_adpcm_size += out_frame_size;
        calc_ptr += in_frame_size;
        calc_remaining -= in_frame_size;
    }

    // Send header first
    uint8_t header[WAV_HEADER_SIZE] = {0};
    build_adpcm_wav_header(header, total_adpcm_size);
    ESP_LOGI(TAG, "Sending ADPCM WAV header for %d bytes", (int)total_adpcm_size);
    
    struct os_mbuf *om = ble_hs_mbuf_from_flat(header, WAV_HEADER_SIZE);
    if (!om) {
        ESP_LOGE(TAG, "Failed to allocate BLE buffer for header");
        goto cleanup;
    }
    int rc = ble_gatts_notify_custom(chr_conn_handle, audio_notify_handle, om);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to notify WAV header; rc=%d", rc);
        goto cleanup;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    // Stream compression and transmission
    size_t stream_offset = 0;
    size_t total_sent = 0;
    
    while (input_remaining >= in_frame_size) {
        memcpy(inbuf, input_ptr, in_frame_size);
        input_ptr += in_frame_size;
        input_remaining -= in_frame_size;

        esp_audio_enc_in_frame_t in_frame = {
            .buffer = inbuf,
            .len = in_frame_size,
        };

        esp_audio_enc_out_frame_t out_frame = {
            .buffer = outbuf,
            .len = out_frame_size,
        };

        esp_err_t err = esp_audio_enc_process(enc_handle, &in_frame, &out_frame);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "ADPCM encoding failed: %s", esp_err_to_name(err));
            break;
        }

        // Add to streaming buffer
        if (stream_offset + out_frame.len > STREAM_BUF_SIZE) {
            // Send current buffer
            size_t send_offset = 0;
            while (send_offset < stream_offset) {
                size_t chunk_len = (stream_offset - send_offset > BLE_CHUNK_SIZE) 
                                   ? BLE_CHUNK_SIZE : (stream_offset - send_offset);
                
                // Retry BLE buffer allocation with backoff
                int retry_count = 0;
                om = NULL;
                while (!om && retry_count < 5) {
                    om = ble_hs_mbuf_from_flat(adpcm_stream_buf + send_offset, chunk_len);
                    if (!om) {
                        ESP_LOGW(TAG, "BLE buffer allocation failed, retry %d/5", retry_count + 1);
                        vTaskDelay(pdMS_TO_TICKS(50));  // Wait for buffers to free up
                        retry_count++;
                    }
                }
                
                if (!om) {
                    ESP_LOGE(TAG, "Failed to allocate BLE buffer after retries");
                    goto cleanup;
                }
                
                rc = ble_gatts_notify_custom(chr_conn_handle, audio_notify_handle, om);
                if (rc != 0) {
                    ESP_LOGE(TAG, "Failed to notify chunk; rc=%d", rc);
                    goto cleanup;
                }
                
                send_offset += chunk_len;
                total_sent += chunk_len;
                vTaskDelay(pdMS_TO_TICKS(80));  // Increased delay to prevent buffer exhaustion
            }
            stream_offset = 0;  // Reset buffer
        }
        
        memcpy(adpcm_stream_buf + stream_offset, outbuf, out_frame.len);
        stream_offset += out_frame.len;
    }

    // Send remaining data in buffer
    if (stream_offset > 0) {
        size_t send_offset = 0;
        while (send_offset < stream_offset) {
            size_t chunk_len = (stream_offset - send_offset > BLE_CHUNK_SIZE) 
                               ? BLE_CHUNK_SIZE : (stream_offset - send_offset);
            
            // Retry BLE buffer allocation with backoff
            int retry_count = 0;
            om = NULL;
            while (!om && retry_count < 5) {
                om = ble_hs_mbuf_from_flat(adpcm_stream_buf + send_offset, chunk_len);
                if (!om) {
                    ESP_LOGW(TAG, "BLE buffer allocation failed (final), retry %d/5", retry_count + 1);
                    vTaskDelay(pdMS_TO_TICKS(50));  // Wait for buffers to free up
                    retry_count++;
                }
            }
            
            if (!om) {
                ESP_LOGE(TAG, "Failed to allocate BLE buffer for final chunk");
                break;
            }
            
            rc = ble_gatts_notify_custom(chr_conn_handle, audio_notify_handle, om);
            if (rc != 0) {
                ESP_LOGE(TAG, "Failed to notify final chunk; rc=%d", rc);
                break;
            }
            
            send_offset += chunk_len;
            total_sent += chunk_len;
            vTaskDelay(pdMS_TO_TICKS(80));  // Increased delay to prevent buffer exhaustion
        }
    }

    ESP_LOGI(TAG, "Audio transmission complete (ADPCM, %d bytes total)", (int)(total_sent + WAV_HEADER_SIZE));

cleanup:
    heap_caps_free(inbuf);
    heap_caps_free(outbuf);
    heap_caps_free(adpcm_stream_buf);
    return ESP_OK;
}