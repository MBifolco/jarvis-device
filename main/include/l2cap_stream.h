#ifndef L2CAP_STREAM_H
#define L2CAP_STREAM_H

//#pragma once
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

// Initialize L2CAP CoC server
esp_err_t l2cap_stream_init(void);

// Send data over the CoC connection
esp_err_t l2cap_stream_send(const uint8_t *data, size_t len);

#endif // L2CAP_STREAM_H