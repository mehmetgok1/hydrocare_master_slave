#ifndef BLE_H
#define BLE_H

#include "common.h"
#include "gap.h"
#include "gatt_svc.h"

// --- Function Declarations ---
void init_ble();
//void notifyAll();
//void processBLETasks();

// --- Service UUID ---
#define SERVICE_UUID        "11111111-1111-1111-1111-111111111110"
// --- Characteristic UUIDs ---
#define UUID_BATTERY        "11111111-1111-1111-2222-111111111112"
#define UUID_LUX            "11111111-1111-1111-2222-111111111113"
#define UUID_PIR            "11111111-1111-1111-2222-111111111114"
#define UUID_MMWAVE         "11111111-1111-1111-2222-111111111115"
#define UUID_ACTION         "11111111-1111-1111-2222-111111111116"
#define UUID_VERSION        "11111111-1111-1111-2222-111111111117"
#define UUID_AMB_INT "11111111-1111-1111-2222-111111111118"
#define UUID_RGB "c2a969f6-16e9-4e08-99e7-5e6086f6a546" // Custom UUID for RGB Frame
#define UUID_IR  "d3b969f6-16e9-4e08-99e7-5e6086f6a547" // Custom UUID for IR Frame


#endif // BLE_H