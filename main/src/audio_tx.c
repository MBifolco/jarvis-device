#include "audio_tx.h"
#include "esp_log.h"
#include "host/ble_hs.h"
#include "os/os_mbuf.h"
#include "gatt_svc.h"  // for chr_conn_handle and audio_chr_handle

#define TAG "audio_tx"
#define BLE_CHUNK_SIZE 180

extern uint16_t audio_chr_handle;  // declared in gatt_svc.c
extern int chr_conn_handle;

esp_err_t audio_tx_send(const uint8_t *pcm, size_t len)
{
    if (!chr_conn_handle || !audio_chr_handle) {
        ESP_LOGW(TAG, "No active BLE connection or wake handle");
        return ESP_FAIL;
    }

    size_t offset = 0;
    int rc = 0;

    while (offset < len) {
        size_t chunk_len = (len - offset > BLE_CHUNK_SIZE) ? BLE_CHUNK_SIZE : (len - offset);

        struct os_mbuf *om = ble_hs_mbuf_from_flat(&pcm[offset], chunk_len);
        if (!om) {
            ESP_LOGE(TAG, "Failed to allocate BLE buffer");
            return ESP_FAIL;
        }

        rc = ble_gatts_notify_custom(chr_conn_handle, audio_chr_handle, om);
        if (rc != 0) {
            ESP_LOGE(TAG, "Failed to notify chunk at offset %d; rc=%d", (int)offset, rc);
            return ESP_FAIL;
        }

        offset += chunk_len;
        vTaskDelay(pdMS_TO_TICKS(30));  // throttle slightly
    }

    ESP_LOGI(TAG, "Audio chunking complete (%d bytes)", (int)len);
    return ESP_OK;
}
