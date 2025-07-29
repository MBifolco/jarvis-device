/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* Includes */
#include "gatt_svc.h"
#include "common.h"
#include "os/os_mbuf.h"
#include "audio_rx.h"
#include "config.h"
#include "l2cap_server.h"
/* Private function declarations */
static int chr_access(uint16_t conn_handle, uint16_t attr_handle,
                                 struct ble_gatt_access_ctxt *ctxt, void *arg);


/* Private variables */

static uint16_t chr_val_handle;

uint16_t chr_conn_handle = 0;
uint16_t wake_chr_handle;
uint16_t audio_notify_handle;
uint16_t audio_write_handle;
uint16_t config_ctrl_handle;
uint16_t l2cap_psm_handle;
static bool handle_inited = false;
static bool ind_status = false;

/* GATT services table */
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID128_DECLARE(
            0x01,0x23,0x45,0x67,
            0x89,0xAB,0xCD,0xEF,
            0x10,0x32,0x54,0x76,
            0x98,0xBA,0xDC,0xFE
        ),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                // Wake-word notify characteristic
                .uuid = BLE_UUID128_DECLARE(
                    0xFE,0xDC,0xBA,0x98,
                    0x76,0x54,0x32,0x10,
                    0xEF,0xCD,0xAB,0x89,
                    0x67,0x45,0x23,0x01
                ),
                .access_cb  = chr_access,  // Or NULL if unused
                .flags      = BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &wake_chr_handle,
            },
            {
                // Audio data stream characteristic
                .uuid = BLE_UUID128_DECLARE(
                    0xAA,0xBB,0xCC,0xDD,
                    0xEE,0xFF,0x00,0x11,
                    0x22,0x33,0x44,0x55,
                    0x66,0x77,0x88,0x99
                ),
                .access_cb  = chr_access,  // Or NULL if unused
                .flags      = BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &audio_notify_handle,
            },
            {
                /* Phone → device (write only) */
                .uuid = BLE_UUID128_DECLARE(
                    0x4E, 0x5F, 0x6A, 0x8B,
                    0xCD, 0x12, 0x4F, 0xAB,
                    0x90, 0xDE, 0x12, 0x34,
                    0x56, 0x78, 0x90, 0xAB
                ),
                .access_cb  = chr_access,
                .flags      = BLE_GATT_CHR_F_WRITE_NO_RSP,
                .val_handle = &audio_write_handle,
            },
            {
                // configuration characteristic
                .uuid = BLE_UUID128_DECLARE(
                    0x79, 0xd4, 0xc3, 0xb2,
                    0x02, 0x0e, 0x67, 0xa5,
                    0x72, 0x43, 0xcc, 0x58, 
                    0x0b, 0xc1, 0x7a, 0xf4
                ),
                .access_cb  = chr_access,
                .flags      = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &config_ctrl_handle,
            },
            {
                // L2CAP PSM characteristic (read-only)
                .uuid = BLE_UUID128_DECLARE(
                    0x12, 0x34, 0x56, 0x78,
                    0x9A, 0xBC, 0xDE, 0xF0,
                    0x11, 0x22, 0x33, 0x44,
                    0x55, 0x66, 0x77, 0x88
                ),
                .access_cb  = chr_access,
                .flags      = BLE_GATT_CHR_F_READ,
                .val_handle = &l2cap_psm_handle,
            },
            {
                0  // End of characteristic list
            }
        }
    },
    {
        0  // End of service list
    }
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
        /* Handle L2CAP PSM read */
        if (attr_handle == l2cap_psm_handle) {
            uint16_t psm = l2cap_server_get_psm();
            rc = os_mbuf_append(ctxt->om, &psm, sizeof(psm));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        return 0;
    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        /* Verify attribute handle */
        if (attr_handle == audio_write_handle) {
            size_t len = OS_MBUF_PKTLEN(ctxt->om);
            uint8_t buf[len];
            if (os_mbuf_copydata(ctxt->om, 0, len, buf) == 0) {
                audio_rx_on_write(buf, len);
                ESP_LOGI(TAG, "Audio Characteristic - Wrote %u bytes from phone", (unsigned)len);
                return 0;
            }
            return 0;
        }else if (attr_handle == config_ctrl_handle) {
            size_t len = OS_MBUF_PKTLEN(ctxt->om);
            uint8_t buf[len];
            if (os_mbuf_copydata(ctxt->om, 0, len, buf) == 0) {
                config_handle_write(buf, len);
                ESP_LOGI(TAG, "Config Characteristic - Wrote %u bytes from phone", (unsigned)len);
                return 0;
            }
            return 0;
        }
        
    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
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

        static const ble_uuid128_t audio_notify_uuid = BLE_UUID128_INIT(
            0xAA,0xBB,0xCC,0xDD,
            0xEE,0xFF,0x00,0x11,
            0x22,0x33,0x44,0x55,
            0x66,0x77,0x88,0x99
        );

        static const ble_uuid128_t audio_write_uuid = BLE_UUID128_INIT(
            0x4E, 0x5F, 0x6A, 0x8B,
            0xCD, 0x12, 0x4F, 0xAB,
            0x90, 0xDE, 0x12, 0x34,
            0x56, 0x78, 0x90, 0xAB
        );

        static const ble_uuid128_t config_ctrl_uuid = BLE_UUID128_INIT(
            0x79, 0xd4, 0xc3, 0xb2,
            0x02, 0x0e, 0x67, 0xa5,
            0x72, 0x43, 0xcc, 0x58,
            0x0b, 0xc1, 0x7a, 0xf4
        );

        static const ble_uuid128_t l2cap_psm_uuid = BLE_UUID128_INIT(
            0x12, 0x34, 0x56, 0x78,
            0x9A, 0xBC, 0xDE, 0xF0,
            0x11, 0x22, 0x33, 0x44,
            0x55, 0x66, 0x77, 0x88
        );

        if (ble_uuid_cmp(ctxt->chr.chr_def->uuid, &wake_uuid.u) == 0) {
            wake_chr_handle = ctxt->chr.val_handle;
            ESP_LOGI(TAG, "Saved wake_chr_handle = %d", wake_chr_handle);
        }
        if (ble_uuid_cmp(ctxt->chr.chr_def->uuid, &audio_notify_uuid.u) == 0) {
            audio_notify_handle = ctxt->chr.val_handle;
            ESP_LOGI(TAG, "Saved audio_notify_handle = %d", audio_notify_handle);
        }
        if (ble_uuid_cmp(ctxt->chr.chr_def->uuid, &audio_write_uuid.u) == 0) {
            audio_write_handle = ctxt->chr.val_handle;
            ESP_LOGI(TAG, "Saved audio_write_handle = %d", audio_write_handle);
        }
        if (ble_uuid_cmp(ctxt->chr.chr_def->uuid, &config_ctrl_uuid.u) == 0) {
            config_ctrl_handle = ctxt->chr.val_handle;
            ESP_LOGI(TAG, "Saved config_ctrl_handle = %d", config_ctrl_handle);
        }
        if (ble_uuid_cmp(ctxt->chr.chr_def->uuid, &l2cap_psm_uuid.u) == 0) {
            l2cap_psm_handle = ctxt->chr.val_handle;
            ESP_LOGI(TAG, "Saved l2cap_psm_handle = %d", l2cap_psm_handle);
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
    } else if (event->subscribe.attr_handle == config_ctrl_handle) {
        // do something cause of the event
        ESP_LOGI(TAG, "Config characteristic subscribed");
        config_init();
    } else if (event->subscribe.attr_handle == audio_notify_handle) {
        // do something cause of the event
        ESP_LOGI(TAG, "Audio notify characteristic subscribed");
    } else if (event->subscribe.attr_handle == wake_chr_handle) {
        // do something cause of the event
        ESP_LOGI(TAG, "Wake-word notify characteristic subscribed");
    } else {
        ESP_LOGW(TAG, "Unknown subscribe attr_handle=%d", event->subscribe.attr_handle);
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

void gatt_svc_gap_event_cb(const struct ble_gap_event *event)
{
    if (event->type == BLE_GAP_EVENT_CONNECT &&
        event->connect.status == 0) {
        chr_conn_handle = event->connect.conn_handle;
        ESP_LOGI(TAG, "Stored conn_handle=%d", chr_conn_handle);
    } else if (event->type == BLE_GAP_EVENT_DISCONNECT) {
        chr_conn_handle = 0;
        ESP_LOGI(TAG, "Cleared conn_handle on disconnect");
    } else if (event->type == BLE_GAP_EVENT_SUBSCRIBE) {
        ESP_LOGI(TAG, "Subscribe: conn=%d attr=%d",
                 event->subscribe.conn_handle,
                 event->subscribe.attr_handle);
    }
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