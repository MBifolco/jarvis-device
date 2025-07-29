#ifndef L2CAP_STREAM_H
#define L2CAP_STREAM_H

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

// Initialize L2CAP CoC server
esp_err_t l2cap_stream_init(void);

// Send data over the CoC connection
esp_err_t l2cap_stream_send(const uint8_t *data, size_t len);

// Get the L2CAP PSM for GATT advertising
uint16_t l2cap_stream_get_psm(void);

#endif // L2CAP_STREAM_H