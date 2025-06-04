// config.h
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

//––– Config IDs –––
typedef enum {
    CFG_COMPRESS_INCOMING  = 0x01,
    CFG_SEND_DEBUG_DROPS   = 0x02,
    CFG_LED_BRIGHTNESS     = 0x03,
    CFG_PLAY_ON_DEVICE     = 0x04,
    // add new IDs here…
} config_id_t;

// Call once in app_main (after gatt_svc_init):
void config_init(void);

// Handle an incoming write to the config characteristic:
//   data:  TLV buffer as [ID][LEN][VALUE…]
//   len:   total length of data[]
void config_handle_write(const uint8_t *data, size_t len);

// If you want to read back a value at runtime:
bool        config_get_compress_incoming(void);
bool        config_get_send_debug_drops(void);
bool        config_get_play_on_device(void);
uint16_t    config_get_led_brightness(void);

// Send a notification of a single config change back to the phone.
// (You can call these after updating in handle_write.)
void config_notify_compress_incoming(void);
void config_notify_send_debug_drops(void);
void config_notify_led_brightness(void);
void config_notify_play_on_device(void);

