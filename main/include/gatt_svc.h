/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#ifndef GATT_SVR_H
#define GATT_SVR_H

/* Includes */
/* NimBLE GATT APIs */
#include "host/ble_gatt.h"
#include "services/gatt/ble_svc_gatt.h"

/* NimBLE GAP APIs */
#include "host/ble_gap.h"

/** Currently connected peer (0 when none) */
extern uint16_t chr_conn_handle;

/** Handle of our notify characteristic (audio → phone) */
extern uint16_t audio_notify_handle;

/** Handle of our write characteristic (phone → device) */
extern uint16_t audio_write_handle;

/* Public function declarations */
void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg);
void gatt_svr_subscribe_cb(struct ble_gap_event *event);
void gatt_svc_gap_event_cb(const struct ble_gap_event *event);
esp_err_t gatt_svc_notify_wake(void);
int gatt_svc_init(void);

#endif // GATT_SVR_H