/**
 * L2CAP Connection-Oriented Channel (COC) Server
 */

#ifndef L2CAP_SERVER_H
#define L2CAP_SERVER_H

#include <stdbool.h>
#include <stdint.h>

/**
 * Initialize L2CAP server
 * @return 0 on success, error code on failure
 */
int l2cap_server_init(void);

/**
 * Send text message over L2CAP channel
 * @param text Text to send
 * @return 0 on success, error code on failure
 */
int l2cap_server_send_text(const char *text);

/**
 * Get L2CAP PSM (Protocol Service Multiplexer)
 * @return PSM value
 */
uint16_t l2cap_server_get_psm(void);

/**
 * Check if L2CAP channel is connected
 * @return true if connected, false otherwise
 */
bool l2cap_server_is_connected(void);

#endif /* L2CAP_SERVER_H */