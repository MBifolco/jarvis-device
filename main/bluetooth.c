// bluetooth.c — Jarvis BLE module

#include "bluetooth.h"
#include "esp_log.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "nvs_flash.h"

static const char *TAG = "jarvis-bt";
static uint16_t wake_chr_handle;
static int conn_handle = 0;  // store the current connection


static esp_err_t init_nvs(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    return ret;
}

esp_err_t bluetooth_init(void) {
    ESP_ERROR_CHECK(init_nvs());
    
    esp_nimble_hci_init();
    nimble_port_init();

    //ble_hs_cfg.reset_cb = ble_on_reset;
    //ble_hs_cfg.sync_cb  = ble_on_sync;
    //ble_store_ram_init();

    //ble_gatts_count_cfg(gatt_svr_svcs);
    //ble_gatts_add_svcs  (gatt_svr_svcs);

    //nimble_port_freertos_init(ble_host_task);

    ESP_LOGI(TAG, "BLE initialized");
    return ESP_OK;
}

