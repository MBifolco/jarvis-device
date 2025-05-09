#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

// Defined in common.h
// AUDIO_L2CAP_PSM
// AUDIO_L2CAP_MTU

/**
 * @brief   Register as an L2CAP CoC server on AUDIO_L2CAP_PSM.
 */
esp_err_t    l2cap_stream_init(void);

/**
 * @brief   Returns true if a peer has opened the CoC channel.
 */
bool         l2cap_stream_is_connected(void);

/**
 * @brief   Send a blob of data, fragmented to AUDIO_L2CAP_MTU bytes.
 */
esp_err_t    l2cap_stream_send(const uint8_t *data, size_t len);

/**
 * @brief   Disconnect and clear the channel.
 */
void         l2cap_stream_close(void);
