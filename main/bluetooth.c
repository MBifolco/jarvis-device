// bluetooth.c — Jarvis BLE module (ESP-IDF v5.4.1)

#include "common.h"
#include "gap.h"
#include "gatt_svc.h"
#include "l2cap_stream.h"

#include "nvs_flash.h"
#include "esp_nimble_hci.h"
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// Bring in the BLE config-storage initializer
void ble_store_config_init(void);

#undef TAG
#define TAG "bluetooth"

// Called if the NimBLE host resets the stack.
static void on_stack_reset(int reason) {
    ESP_LOGI(TAG, "BLE stack reset, reason=%d", reason);
}

// Called once the host and controller are in sync.
static void on_stack_sync(void) {
    adv_init();
}

// Configure the BLE host stack callbacks and storage.
static void nimble_host_config_init(void) {
    ble_hs_cfg.reset_cb          = on_stack_reset;
    ble_hs_cfg.sync_cb           = on_stack_sync;
    ble_hs_cfg.gatts_register_cb = gatt_svc_register_cb;
    ble_hs_cfg.store_status_cb   = ble_store_util_status_rr;
    ble_store_config_init();
}

// The task that runs the NimBLE host.
static void nimble_host_task(void *param) {
    ESP_LOGI(TAG, "Starting NimBLE host task");
    nimble_port_run();
    vTaskDelete(NULL);
}

void bluetooth_init(void) {
    esp_err_t ret;
    int rc;

    // 1) NVS init
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed: %d", ret);
        return;
    }

    // 2) HCI & Controller init
    esp_nimble_hci_and_controller_init();

    // 3) Host stack init
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %d", ret);
        return;
    }

    // 4) Host config
    nimble_host_config_init();

    // 5) Start host task
    nimble_port_freertos_init(nimble_host_task);

    // 6) GAP (advertising)
    rc = gap_init();
    if (rc) {
        ESP_LOGE(TAG, "gap_init failed: %d", rc);
        return;
    }

    // 7) GATT server
    rc = gatt_svc_init();
    if (rc) {
        ESP_LOGE(TAG, "gatt_svc_init failed: %d", rc);
        return;
