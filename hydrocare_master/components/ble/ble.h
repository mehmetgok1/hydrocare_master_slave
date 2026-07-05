#ifndef BLE_H
#define BLE_H
#include "config.h"
#include "measurement.h"
#include "ui.h"
#include "communication.h"
#include "sd.h"
#include "wifi_stream.h"
#include "ota.h"
#include "common.h"
#include "gap.h"
#include "gatt_svc.h"
#include "timer.h"

// --- Function Declarations ---
void init_ble();
void notifyAll(uint8_t bat, uint16_t lux, uint8_t pir, uint8_t mmwave, uint16_t amb_int, 
               uint16_t *rgb_frame, uint16_t rgb_len, 
               uint16_t *ir_frame, uint16_t ir_len);

#endif // BLE_H