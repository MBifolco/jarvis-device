// bluetooth.c — Jarvis BLE module (ESP-IDF v5.4.1)

// [1] Core ESP & NimBLE includes
#include "bluetooth.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"               // for nvs_flash_init()
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

// [2] NimBLE host includes
#include "host/ble_hs.h"
#include "host/util/util.h"          // for ble_store_util_status_rr()
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "jarvis-bt";
//static uint16_t wake_chr_handle;
static int      conn_handle = 0;    // current connection

// Forward-declare the store init function (no header provided in IDF)
extern void ble_store_config_init(void);

static void ble_on_reset(int reason);
static void ble_on_sync(void);
static void ble_host_task(void *param);
static int gap_event_cb(struct ble_gap_event *event, void *arg);
static void adv_init(void);

static esp_err_t init_nvs(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    return ret;
}

esp_err_t bluetooth_init(void)
{
    // 1) NVS for PHY calibration & BLE storage
    ESP_ERROR_CHECK(init_nvs());

    // 2) Initialize controller + host stack
    esp_nimble_hci_init();
    nimble_port_init();

    // 3) Set up host callbacks
    ble_hs_cfg.reset_cb        = ble_on_reset;
    ble_hs_cfg.sync_cb         = ble_on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;    // :contentReference[oaicite:0]{index=0}

    // 4) Configure storage (NVS-backed by default in IDF) 
    ble_store_config_init();                                    // :contentReference[oaicite:1]{index=1}

    // 5) Launch the NimBLE host task
    nimble_port_freertos_init(ble_host_task);

    ESP_LOGI(TAG, "BLE initialized");
    return ESP_OK;
}

static void ble_on_reset(int reason)
{
    ESP_LOGE(TAG, "BLE reset; reason=%d", reason);
}

static void ble_on_sync(void)
{
    adv_init();
}

static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "Client connected; handle=%d", conn_handle);
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Client disconnected; reason=%d",
                 event->disconnect.reason);
        conn_handle = 0;
        adv_init();
        break;
    default:
        break;
    }
    return 0;
}

static void adv_init(void)
{
    int rc;
    rc = ble_svc_gap_device_name_set("Jarvis");
    assert(rc == 0);

    // Ensure we have an address
    uint8_t addr_type;
    ble_hs_util_ensure_addr(0);
    ble_hs_id_infer_auto(0, &addr_type);

    // Build advertising fields (flags, name, TX power, etc.)
    struct ble_hs_adv_fields adv_fields = {0};
    adv_fields.flags     = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    const char *name     = ble_svc_gap_device_name();
    adv_fields.name      = (uint8_t*)name;
    adv_fields.name_len  = strlen(name);
    adv_fields.name_is_complete = 1;
    adv_fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    adv_fields.tx_pwr_lvl_is_present = 1;

    rc = ble_gap_adv_set_fields(&adv_fields);
    if (rc) {
        ESP_LOGE(TAG, "adv_set_fields rc=%d", rc);
        return;
    }

    // Start advertising (undirected, scannable)
    struct ble_gap_adv_params params = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
    };
    rc = ble_gap_adv_start(addr_type, NULL, BLE_HS_FOREVER,
                           &params, gap_event_cb, NULL);
    if (rc) {
        ESP_LOGE(TAG, "adv_start rc=%d", rc);
    } else {
        ESP_LOGI(TAG, "Advertising started");
    }
}

static void ble_host_task(void *param)
{
    ESP_LOGI(TAG, "NimBLE host task running");
    nimble_port_run();
    vTaskDelete(NULL);
}
