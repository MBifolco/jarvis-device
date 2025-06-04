// config.c
#include "config.h"
#include "esp_log.h"
#include "os/os_mbuf.h"
#include "gatt_svc.h"    // brings in externs: chr_conn_handle, config_ctrl_handle

static const char *TAG = "config";

//––– Local storage of each setting –––
static bool   s_compress_incoming  = true;
static bool   s_send_debug_drops   = false;
static uint16_t s_led_brightness   = 0;

//––– Helpers to notify the central –––
static void _notify(const uint8_t *buf, size_t len) {
    if (chr_conn_handle == 0) {
        ESP_LOGW(TAG, "No client; skipping notify");
        return;
    }
    struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, len);
    if (!om) {
        ESP_LOGE(TAG, "notify: mbuf alloc failed");
        return;
    }
    int rc = ble_gatts_notify_custom(chr_conn_handle,
                                     config_ctrl_handle,
                                     om);
    if (rc) {
        ESP_LOGE(TAG, "notify failed; rc=%d", rc);
    }
}

void config_notify_compress_incoming(void) {
    uint8_t buf[3] = {
        CFG_COMPRESS_INCOMING,
        1,
        s_compress_incoming ? 1 : 0
    };
    _notify(buf, sizeof(buf));
}

void config_notify_send_debug_drops(void) {
    uint8_t buf[3] = {
        CFG_SEND_DEBUG_DROPS,
        1,
        s_send_debug_drops ? 1 : 0
    };
    _notify(buf, sizeof(buf));
}

void config_notify_led_brightness(void) {
    uint8_t buf[4] = {
        CFG_LED_BRIGHTNESS,
        2,
        (uint8_t)(s_led_brightness & 0xFF),
        (uint8_t)(s_led_brightness >> 8)
    };
    _notify(buf, sizeof(buf));
}

//––– Public getters –––
bool     config_get_compress_incoming(void) { return s_compress_incoming; }
bool     config_get_send_debug_drops(void)  { return s_send_debug_drops; }
uint16_t config_get_led_brightness(void)    { return s_led_brightness; }

//––– Initialization –––
void config_init(void) {
    // log initial values
    ESP_LOGI(TAG, "CFG_COMPRESS_INCOMING  = %s",
             s_compress_incoming ? "ON" : "OFF");
    ESP_LOGI(TAG, "CFG_SEND_DEBUG_DROPS   = %s",
                s_send_debug_drops ? "ON" : "OFF");
    ESP_LOGI(TAG, "CFG_LED_BRIGHTNESS     = %u",
             s_led_brightness);
    // defaults already set above; if you want to push them to the phone:
    
    if (chr_conn_handle != 0) {
        config_notify_compress_incoming();
        config_notify_send_debug_drops();
        config_notify_led_brightness();
    }
}

//––– TLV parser –––
void config_handle_write(const uint8_t *data, size_t len) {
    if (len < 2) {
        ESP_LOGW(TAG, "Invalid config write: too short");
        return;
    }
    uint8_t id      = data[0];
    uint8_t paylen  = data[1];
    if ((size_t)(2 + paylen) > len) {
        ESP_LOGW(TAG, "Invalid config write: truncated payload");
        return;
    }
    const uint8_t *val = data + 2;

    switch (id) {
        case CFG_COMPRESS_INCOMING:
            if (paylen == 1) {
                s_compress_incoming = (val[0] != 0);
                ESP_LOGI(TAG, "CFG_COMPRESS_INCOMING = %s",
                         s_compress_incoming ? "ON" : "OFF");
                // echo back
                config_notify_compress_incoming();
            }
            break;

        case CFG_SEND_DEBUG_DROPS:
            if (paylen == 1) {
                s_send_debug_drops = (val[0] != 0);
                ESP_LOGI(TAG, "CFG_SEND_DEBUG_DROPS = %s",
                         s_send_debug_drops ? "ON" : "OFF");
                config_notify_send_debug_drops();
            }
            break;

        case CFG_LED_BRIGHTNESS:
            if (paylen == 2) {
                s_led_brightness = val[0] | (val[1] << 8);
                ESP_LOGI(TAG, "CFG_LED_BRIGHTNESS = %u",
                         s_led_brightness);
                config_notify_led_brightness();
            }
            break;

        default:
            ESP_LOGW(TAG, "Unknown config ID 0x%02x", id);
            break;
    }
}
