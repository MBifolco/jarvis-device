// bluetooth.h

#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <stdint.h>
#include <esp_err.h>

// Call this once from app_main()
esp_err_t bluetooth_init(void);

// Call this inside your fetch_task when the wake word fires.
// Returns ESP_OK if notification was sent.
esp_err_t bluetooth_notify_wake(void);

#endif // BLUETOOTH_H
