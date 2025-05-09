#pragma once
#include "esp_err.h"

int l2cap_stream_init(void);
esp_err_t l2cap_stream_send(const uint8_t *data, size_t len);
