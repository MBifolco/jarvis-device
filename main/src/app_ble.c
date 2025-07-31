#include "app_ble.h"
#include "gatt_svc.h"
#include "audio_tx.h"
#include "esp_log.h"

static const char *TAG = "app_ble";

void app_ble_init(void) {
    ESP_LOGI(TAG, "App BLE wrapper initialized");
}

void app_ble_notify_wake(void) {
    gatt_svc_notify_wake();
    ESP_LOGI(TAG, "Notified wake to central");
}

void app_ble_send_audio(const uint8_t *data, size_t len) {
    audio_tx_send(data, len);
    ESP_LOGI(TAG, "Sent %u bytes of audio", (unsigned)len);
}
