// config.h
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "driver/gpio.h"   // for GPIO_NUM_x
#include "driver/i2s.h"    // for I2S_NUM_x
//––– Config IDs –––
typedef enum {
    CFG_COMPRESS_INCOMING  = 0x01,
    CFG_SEND_DEBUG_DROPS   = 0x02,
    CFG_LED_BRIGHTNESS     = 0x03,
    CFG_PLAY_ON_DEVICE     = 0x04,
    // add new IDs here…
} config_id_t;

#define SAMPLE_RATE       16000
#define POST_WAKE_SECONDS 30           // max record length
#define MIN_RECORD_SAMPLES (SAMPLE_RATE / 1)  // 1 second at 16 kHz
#define KEEP_ALIVE_MS      (20000)          // 20-second keep-alive window

#define MIC_BCK_IO        GPIO_NUM_8 // SCK
#define MIC_WS_IO         GPIO_NUM_10 // WS
#define MIC_DATA_IO       GPIO_NUM_9 // SD

#define SPK_BCK_IO        GPIO_NUM_6 // BCLK
#define SPK_WS_IO         GPIO_NUM_5 //LRC
#define SPK_DATA_IO       GPIO_NUM_7 // DIN

#define I2S_MIC_PORT      I2S_NUM_0
#define I2S_SPK_PORT      I2S_NUM_1

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

