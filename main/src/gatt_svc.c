/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* Includes */
#include "gatt_svc.h"
#include "common.h"
#include "os/os_mbuf.h"

/* Private function declarations */
static int chr_access(uint16_t conn_handle, uint16_t attr_handle,
                                 struct ble_gatt_access_ctxt *ctxt, void *arg);


/* Private variables */

static uint16_t chr_val_handle;

static uint16_t chr_conn_handle = 0;
static uint16_t wake_chr_handle;
static bool handle_inited = false;
static bool ind_status = false;

/* GATT services table */
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {

    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = BLE_UUID128_DECLARE(
        0x01,0x23,0x45,0x67,
        0x89,0xAB,0xCD,0xEF,
        0x10,0x32,0x54,0x76,
        0x98,0xBA,0xDC,0xFE
    ),
     .characteristics =
         (struct ble_gatt_chr_def[]){
             {
              .uuid = BLE_UUID128_DECLARE(
                0xFE,0xDC,0xBA,0x98,
                0x76,0x54,0x32,0x10,
                0xEF,0xCD,0xAB,0x89,
                0x67,0x45,0x23,0x01
            ),
              .access_cb = chr_access,
              .flags = BLE_GATT_CHR_F_NOTIFY,
              .val_handle = &chr_val_handle},
             {
                 0, /* No more characteristics in this service. */
             }}},

    {
        0, /* No more services. */
    },
};

/* Private functions */
static int chr_access(uint16_t conn_handle, uint16_t attr_handle,
                                 struct ble_gatt_access_ctxt *ctxt, void *arg) {
    /* Local variables */
    int rc;

    /* Handle access events */
    /* Note:characteristic is read only */
    switch (ctxt->op) {

    /* Read characteristic event */
    case BLE_GATT_ACCESS_OP_READ_CHR:
        /* Verify connection handle */
        if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            ESP_LOGI(TAG, "characteristic read; conn_handle=%d attr_handle=%d",
                     conn_handle, attr_handle);
        } else {
            ESP_LOGI(TAG, "characteristic read by nimble stack; attr_handle=%d",
                     attr_handle);
        }

        /* Verify attribute handle */
        if (attr_handle == chr_val_handle) {
        
            //do whatever
            //return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            return 0;
        }
        goto error;

    /* Unknown event */
    default:
        goto error;
    }

error:
    ESP_LOGE(
        TAG,
        "unexpected access operation characteristic, opcode: %d",
        ctxt->op);
    return BLE_ATT_ERR_UNLIKELY;
}


/*
 *  Handle GATT attribute register events
 *      - Service register event
 *      - Characteristic register event
 *      - Descriptor register event
 */
void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg) {
    /* Local variables */
    char buf[BLE_UUID_STR_LEN];

    /* Handle GATT attributes register events */
    switch (ctxt->op) {

    /* Service register event */
    case BLE_GATT_REGISTER_OP_SVC:
        ESP_LOGD(TAG, "registered service %s with handle=%d",
                 ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                 ctxt->svc.handle);
        break;

    /* Characteristic register event */
    case BLE_GATT_REGISTER_OP_CHR:
        ESP_LOGD(TAG,
                 "registering characteristic %s with "
                 "def_handle=%d val_handle=%d",
                 ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                 ctxt->chr.def_handle, ctxt->chr.val_handle);
        // Match against your known wake-word characteristic UUID
        static const ble_uuid128_t wake_uuid = BLE_UUID128_INIT(
            0xFE,0xDC,0xBA,0x98,
            0x76,0x54,0x32,0x10,
            0xEF,0xCD,0xAB,0x89,
            0x67,0x45,0x23,0x01
        );

        if (ble_uuid_cmp(ctxt->chr.chr_def->uuid, &wake_uuid.u) == 0) {
            wake_chr_handle = ctxt->chr.val_handle;
            ESP_LOGI(TAG, "Saved wake_chr_handle = %d", wake_chr_handle);
        }
        break;

    /* Descriptor register event */
    case BLE_GATT_REGISTER_OP_DSC:
        ESP_LOGD(TAG, "registering descriptor %s with handle=%d",
                 ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                 ctxt->dsc.handle);
        break;

    /* Unknown event */
    default:
        assert(0);
        break;
    }
}

/*
 *  GATT server subscribe event callback
 */

void gatt_svr_subscribe_cb(struct ble_gap_event *event) {
    /* Check connection handle */
    if (event->subscribe.conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        ESP_LOGI(TAG, "subscribe event; conn_handle=%d attr_handle=%d",
                 event->subscribe.conn_handle, event->subscribe.attr_handle);
    } else {
        ESP_LOGI(TAG, "subscribe by nimble stack; attr_handle=%d",
                 event->subscribe.attr_handle);
    }

    /* Check attribute handle */
    if (event->subscribe.attr_handle == chr_val_handle) {
        // do something cause of the event
        chr_conn_handle = event->subscribe.conn_handle;
        handle_inited = true;
        ind_status = event->subscribe.cur_indicate;
    }
}

// Call this when your wake-word engine fires
esp_err_t gatt_svc_notify_wake(void)
{
    if (chr_conn_handle == 0) {
        ESP_LOGW(TAG, "No BLE client connected; skipping notify");
        return ESP_FAIL;
    }

    const char *msg = "awake";
    struct os_mbuf *om = ble_hs_mbuf_from_flat(msg, strlen(msg));
    if (!om) {
        ESP_LOGE(TAG, "Failed to allocate mbuf");
        return ESP_FAIL;
    }

    int rc = ble_gatts_notify_custom(chr_conn_handle, wake_chr_handle, om);
    if (rc != 0) {
        ESP_LOGE(TAG, "Notify failed; rc=%d", rc);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Wake-word notification sent (msg='%s')", msg);
    return ESP_OK;
}


/*
 *  GATT server initialization
 *      1. Initialize GATT service
 *      2. Update NimBLE host GATT services counter
 *      3. Add GATT services to server
 */
int gatt_svc_init(void) {
    /* Local variables */
    int rc;

    /* 1. GATT service initialization */
    ble_svc_gatt_init();

    /* 2. Update GATT services counter */
    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    /* 3. Add GATT services */
    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    return 0;
}