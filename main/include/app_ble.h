#ifndef APP_BLE_H
#define APP_BLE_H

#include <stdint.h>
#include <stddef.h>

// Initialize BLE transport (call once after bluetooth_init)
void app_ble_init(void);

// Notify central that the wake word was detected
void app_ble_notify_wake(void);

// Send a chunk of audio data over BLE
void app_ble_send_audio(const uint8_t *data, size_t len);

#endif // APP_BLE_H
