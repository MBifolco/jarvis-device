// bluetooth.c — Jarvis BLE module (ESP-IDF v5.4.1)

#include "common.h"
#include "gap.h"
#include "gatt_svc.h"
#include "l2cap_stream.h"

/* Library function declarations */
void ble_store_config_init(void);

/* Private function declarations */
static void on_stack_reset(int reason);
static void on_stack_sync(void);
static void nimble_host_config_init(void);
static void nimble_host_task(void *param);

/* Private functions */
/*
 *  Stack event callback functions
 *      - on_stack_reset is called when host resets BLE stack due to errors
 *      - on_stack_sync is called when host has synced with controller
 */
 static void on_stack_reset(int reason) {
    /* On reset, print reset reason to console */
    ESP_LOGI(TAG, "nimble stack reset, reset reason: %d", reason);
}

static void on_stack_sync(void) {
    /* On stack sync, do advertising initialization */
    adv_init();
}

static void nimble_host_config_init(void) {
    /* Set host callbacks */
    ble_hs_cfg.reset_cb = on_stack_reset;
    ble_hs_cfg.sync_cb = on_stack_sync;
    ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    /* Store host configuration */
    ble_store_config_init();
}

static void nimble_host_task(void *param) {
    /* Task entry log */
    ESP_LOGI(TAG, "nimble host task has been started!");

    /* This function won't return until nimble_port_stop() is executed */
    nimble_port_run();

    /* Clean up at exit */
    vTaskDelete(NULL);
}

void bluetooth_init(void) {
    int rc;
    esp_err_t ret;

    // 1) NVS flash init (BLE needs this)
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

    // 2) NimBLE stack init
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NimBLE init failed: %d", ret);
        return;
    }

    // 3) GAP (advertising) setup
    rc = gap_init();
    if (rc != 0) {
        ESP_LOGE(TAG, "GAP init failed: %d", rc);
        return;
    }

    // 4) GATT server setup (for wake notification)
    rc = gatt_svc_init();
    if (rc != 0) {
        ESP_LOGE(TAG, "GATT svc init failed: %d", rc);
        return;
    }

    // 5) L2CAP CoC server setup (audio streaming)
    {
        struct ble_l2cap_chan_cfg cfg = {0};
        cfg.sc_mtu           = AUDIO_L2CAP_MTU;
        cfg.sc_my_rx.credits      = AUDIO_L2CAP_INITIAL_CREDITS;
        cfg.sc_my_rx.credit_thresh = AUDIO_L2CAP_MTU * 2;
        cfg.sc_my_rx.flow_control  = BLE_L2CC_RX_FLOW_CTRL;

        rc = ble_l2cap_accept_connection(
                AUDIO_L2CAP_PSM,
                &cfg,
                l2cap_rx_cb,
                &l2cap_chan_arg
        );
        if (rc != 0) {
            ESP_LOGE(TAG, "L2CAP accept failed: %d", rc);
            return;
        }
    }

    // 6) Host configuration & task
    nimble_host_config_init();
    xTaskCreate(
        nimble_host_task,
        "NimBLE Host",
        4 * 1024,
        NULL,
        5,
        NULL
    );
}
