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
#include <time.h>
#include <sys/time.h>

/* NimBLE GAP APIs */
#include "host/ble_gap.h"

/* Public function declarations */
void send_heart_rate_indication(void);
void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg);
void gatt_svr_subscribe_cb(struct ble_gap_event *event);
void gatt_svr_reset_heart_rate_subscription(void);
int gatt_svc_init(void);

/* Function prototypes for getting value handles */
uint16_t* get_bat_val_handle(void);
uint16_t* get_lux_val_handle(void);
uint16_t* get_pir_val_handle(void);
uint16_t* get_mmwave_val_handle(void);
uint16_t* get_amb_int_val_handle(void);
uint16_t* get_rgb_val_handle(void);
uint16_t* get_ir_val_handle(void);

//getters for global variables
char* get_ssid(void);
char* get_password(void);
char* get_ver(void);
char* get_server_ip(void);
char* get_sessionFolder(void);
bool get_deviceStatus(void);
bool get_otaUpdateAvailable(void);
bool get_deviceConnected(void);
bool get_sendRgbFlag(void);
bool get_sendIrFlag(void);
bool* get_stream_wifi(void);
bool* get_wifi_connect(void);
bool get_sessionInitialized(void);
uint32_t* get_packetsLogged(void);

#endif // GATT_SVR_H
