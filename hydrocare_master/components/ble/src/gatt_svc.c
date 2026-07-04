/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* Includes */
#include "gatt_svc.h"
#include "sd.h"
#include "measurement/measurement.h"
#include "communication/communication.h"
#include <sys/time.h>
#include "common.h"


//// --- Service UUID ---
//#define SERVICE_UUID        "11111111-1111-1111-1111-111111111110"
//// --- Characteristic UUIDs ---
//#define UUID_BATTERY        "11111111-1111-1111-2222-111111111112"
//#define UUID_LUX            "11111111-1111-1111-2222-111111111113"
//#define UUID_PIR            "11111111-1111-1111-2222-111111111114"
//#define UUID_MMWAVE         "11111111-1111-1111-2222-111111111115"
//#define UUID_ACTION         "11111111-1111-1111-2222-111111111116"
//#define UUID_VERSION        "11111111-1111-1111-2222-111111111117"
//#define UUID_AMB_INT        "11111111-1111-1111-2222-111111111118"
//#define UUID_RGB            "c2a969f6-16e9-4e08-99e7-5e6086f6a546" // Custom UUID for RGB Frame
//#define UUID_IR             "d3b969f6-16e9-4e08-99e7-5e6086f6a547" // Custom UUID for IR Frame

/* Private function declarations */
static int gatt_svr_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg);
static void handle_action_write(struct os_mbuf *om);

/* Extern variables from other parts of the application */
extern bool deviceConnected;
extern bool sendRgbFlag;
extern bool sendIrFlag;
extern bool stream_wifi;
extern char ssid[32], password[64], ver[16], server_ip[16];
extern bool wifi_connect;
extern bool sessionInitialized;
extern uint32_t packetsLogged;
extern int otaUpdateAvailable;
extern uint8_t deviceStatus;

/* Private variables */

/* Custom Service UUID */
static const ble_uuid128_t gatt_svc_uuid =
    BLE_UUID128_INIT(0x10, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11,
                     0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11);

/* Characteristic UUIDs */
static const ble_uuid128_t gatt_bat_uuid =
    BLE_UUID128_INIT(0x12, 0x11, 0x11, 0x11, 0x11, 0x11, 0x22, 0x22,
                     0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11);
static const ble_uuid128_t gatt_lux_uuid =
    BLE_UUID128_INIT(0x13, 0x11, 0x11, 0x11, 0x11, 0x11, 0x22, 0x22,
                     0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11);
static const ble_uuid128_t gatt_pir_uuid =
    BLE_UUID128_INIT(0x14, 0x11, 0x11, 0x11, 0x11, 0x11, 0x22, 0x22,
                     0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11);
static const ble_uuid128_t gatt_mmwave_uuid =
    BLE_UUID128_INIT(0x15, 0x11, 0x11, 0x11, 0x11, 0x11, 0x22, 0x22,
                     0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11);
static const ble_uuid128_t gatt_action_uuid =
    BLE_UUID128_INIT(0x16, 0x11, 0x11, 0x11, 0x11, 0x11, 0x22, 0x22,
                     0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11);
static const ble_uuid128_t gatt_version_uuid =
    BLE_UUID128_INIT(0x17, 0x11, 0x11, 0x11, 0x11, 0x11, 0x22, 0x22,
                     0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11);
static const ble_uuid128_t gatt_amb_int_uuid =
    BLE_UUID128_INIT(0x18, 0x11, 0x11, 0x11, 0x11, 0x11, 0x22, 0x22,
                     0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11);
static const ble_uuid128_t gatt_rgb_uuid =
    BLE_UUID128_INIT(0x46, 0xa5, 0xf6, 0x86, 0x60, 0x5e, 0xe7, 0x99,
                     0x08, 0x4e, 0xe9, 0x16, 0xf6, 0x69, 0xa9, 0xc2);
static const ble_uuid128_t gatt_ir_uuid =
    BLE_UUID128_INIT(0x47, 0xa5, 0xf6, 0x86, 0x60, 0x5e, 0xe7, 0x99,
                     0x08, 0x4e, 0xe9, 0x16, 0xf6, 0x69, 0xb9, 0xd3);


/* GATT services table */
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    /* Custom Service */
    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = &gatt_svc_uuid.u,
     .characteristics =
         (struct ble_gatt_chr_def[]){
             {.uuid = &gatt_version_uuid.u,
              .access_cb = gatt_svr_chr_access,
              .flags = BLE_GATT_CHR_F_READ},
             {.uuid = &gatt_bat_uuid.u,
              .access_cb = gatt_svr_chr_access,
              .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY},
             {.uuid = &gatt_lux_uuid.u,
              .access_cb = gatt_svr_chr_access,
              .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY},
             {.uuid = &gatt_pir_uuid.u,
              .access_cb = gatt_svr_chr_access,
              .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY},
             {.uuid = &gatt_mmwave_uuid.u,
              .access_cb = gatt_svr_chr_access,
              .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY},
             {.uuid = &gatt_amb_int_uuid.u,
              .access_cb = gatt_svr_chr_access,
              .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY},
             {.uuid = &gatt_action_uuid.u,
              .access_cb = gatt_svr_chr_access,
              .flags = BLE_GATT_CHR_F_WRITE},
             {.uuid = &gatt_rgb_uuid.u,
              .access_cb = gatt_svr_chr_access,
              .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY},
             {.uuid = &gatt_ir_uuid.u,
              .access_cb = gatt_svr_chr_access,
              .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY},
             {0} /* No more characteristics in this service */
         }}
};

/* Command handler for writes to the action characteristic */
static void handle_action_write(struct os_mbuf *om) {
    char command[256];
    size_t len;

    len = os_mbuf_len(om);
    if (len >= sizeof(command)) {
        ESP_LOGE(TAG, "Command too long");
        return;
    }

    if (os_mbuf_copydata(om, 0, len, command) != 0) {
        return;
    }
    command[len] = '\0';

    ESP_LOGI(TAG, "[BLE] Command Received: %s", command);

    if (strncmp(command, "Com;OTA", 7) == 0) {
        char *p1 = strchr(command, ';');
        if (p1) {
            char *p2 = strchr(p1 + 1, ';');
            if (p2) {
                char *p3 = strchr(p2 + 1, ';');
                if (p3) {
                    char *p4 = strchr(p3 + 1, ';');
                    if (p4) {
                        strncpy(ver, p2 + 1, p3 - (p2 + 1));
                        ver[p3 - (p2 + 1)] = '\0';
                        strncpy(ssid, p3 + 1, p4 - (p3 + 1));
                        ssid[p4 - (p3 + 1)] = '\0';
                        strcpy(password, p4 + 1);

                        ESP_LOGI(TAG, "[OTA] Starting Update...");
                        ESP_LOGI(TAG, "SSID: %s, Ver: %s", ssid, ver);
                        otaUpdateAvailable = 1;
                    }
                }
            }
        }
    } else if (strncmp(command, "Com;Start", 9) == 0) {
        // char* label = "Default";
        // char* p1 = strchr(command, ';');
        // if (p1) {
        //     char* p2 = strchr(p1 + 1, ';');
        //     if (p2 && strlen(p2 + 1) > 0) {
        //         label = p2 + 1;
        //         // trim whitespace if necessary
        //     }
        // }
        deviceStatus = 1;
        ESP_LOGI(TAG, "\n[BLE] Logging started - Creating new session folder...");
        // initSessionFolder(); // This function needs to be available
        packetsLogged = 0;
        sessionInitialized = true;
        ESP_LOGI(TAG, "[BLE] Session files ready for logging");
    } else if (strncmp(command, "Com;Stop", 8) == 0) {
        deviceStatus = 0;
        sessionInitialized = false;
        ESP_LOGI(TAG, "[SD] Stop Logging. files will be transmitted over TCP socket.");
        stream_wifi = true;
    } else if (strncmp(command, "Com;WiFi", 8) == 0) {
        char *p1 = strchr(command, ';');
        if (p1) {
            char *p2 = strchr(p1 + 1, ';');
            if (p2) {
                char *p3 = strchr(p2 + 1, ';');
                if (p3) {
                    char *p4 = strchr(p3 + 1, ';');
                    if (p4) {
                        strncpy(ssid, p2 + 1, p3 - (p2 + 1));
                        ssid[p3 - (p2 + 1)] = '\0';
                        strncpy(password, p3 + 1, p4 - (p3 + 1));
                        password[p4 - (p3 + 1)] = '\0';
                        strcpy(server_ip, p4 + 1);
                        ESP_LOGI(TAG, "[BLE] WiFi Config updated: %s, Server: %s:8080", ssid, server_ip);
                        wifi_connect = true;
                    }
                }
            }
        }
    } else if (strncmp(command, "Com;Control", 11) == 0) {
        char *p1 = strchr(command, ';');
        if (p1) {
            char *p2 = strchr(p1 + 1, ';');
            if (p2) {
                char *p3 = strchr(p2 + 1, ';');
                if (p3) {
                    char type[16];
                    strncpy(type, p2 + 1, p3 - (p2 + 1));
                    type[p3 - (p2 + 1)] = '\0';
                    int val = atoi(p3 + 1);

                    ESP_LOGI(TAG, "[BLE] Control %s: %d", type, val);
                    if (strcmp(type, "IR") == 0) {
                        ESP_LOGI(TAG, "IR LED status is %d", val);
                        sendIRLED(val);
                    }
                    if (strcmp(type, "LED") == 0) {
                        ESP_LOGI(TAG, "Power LED brightness is %d", val);
                        sendBrightness(val);
                    }
                }
            }
        }
    } else if (strncmp(command, "Com;RGB", 7) == 0) {
        ESP_LOGI(TAG, "[BLE] Command RGB received. Queueing frame transmission...");
        sendRgbFlag = true;
    } else if (strncmp(command, "Com;Frames", 10) == 0) {
        ESP_LOGI(TAG, "[BLE] Command Frames received. Queueing RGB and IR frame transmission...");
        sendRgbFlag = true;
        sendIrFlag = true;
    } else if (strncmp(command, "Com;SetTime", 11) == 0) {
        char *p1 = strchr(command, ';');
        if (p1) {
            char *p2 = strchr(p1 + 1, ';');
            if (p2) {
                time_t timestamp = atol(p2 + 1);
                if (timestamp > 0) {
                    struct timeval tv = {.tv_sec = timestamp, .tv_usec = 0};
                    settimeofday(&tv, NULL);

                    time_t now = time(NULL);
                    char timeStr[30];
                    strftime(timeStr, sizeof(timeStr), "%Y%m%d_%H%M%S", localtime(&now));
                    ESP_LOGI(TAG, "[BLE] System time set to: %s", timeStr);
                } else {
                    ESP_LOGE(TAG, "[BLE] Invalid timestamp");
                }
            }
        }
    }
}

/* Private functions */
static int gatt_svr_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg) {
    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        ESP_LOGI(TAG, "Characteristic read; conn_handle=%d, attr_handle=%d",
                 conn_handle, attr_handle);
        // Here you would typically fetch the current value of the characteristic
        // and append it to ctxt->om. For now, this is a placeholder.
        // For example, for the version characteristic:
        // if (ble_uuid_cmp(&ctxt->chr->uuid, &gatt_version_uuid.u) == 0) {
        //     os_mbuf_append(ctxt->om, "1.0.0", strlen("1.0.0"));
        // }
        return 0;
    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        ESP_LOGI(TAG, "Characteristic write; conn_handle=%d, attr_handle=%d",
                 conn_handle, attr_handle);
        if (ble_uuid_cmp(ctxt->chr->uuid, &gatt_action_uuid.u) == 0) {
            handle_action_write(ctxt->om);
        }
        return 0;
    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
}

/*
 *  Handle GATT attribute register events
 *      - Service register event
 *      - Characteristic register event
 *      - Descriptor register event
 */
void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg) {
    /* Local variables */
    char buf[BLE_UUID_STR_LEN];

    /* Handle GATT attributes register events */
    switch (ctxt->op) {

    /* Service register event */
    case BLE_GATT_REGISTER_OP_SVC:
        ESP_LOGD(TAG, "registered service %s with handle=%d",
                 ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                 ctxt->svc.handle);
        break;

    /* Characteristic register event */
    case BLE_GATT_REGISTER_OP_CHR:
        ESP_LOGD(TAG,
                 "registering characteristic %s with "
                 "def_handle=%d val_handle=%d",
                 ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                 ctxt->chr.def_handle, ctxt->chr.val_handle);
        break;

    /* Descriptor register event */
    case BLE_GATT_REGISTER_OP_DSC:
        ESP_LOGD(TAG, "registering descriptor %s with handle=%d",
                 ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                 ctxt->dsc.handle);
        break;

    /* Unknown event */
    default:
        assert(0);
        break;
    }
}

/*
 *  GATT server subscribe event callback
 *      1. Update heart rate subscription status
 */

void gatt_svr_subscribe_cb(struct ble_gap_event *event)
{
    ESP_LOGI(TAG, "subscribe event; conn_handle=%d attr_handle=%d "
             "reason=%d prevn=%d curn=%d previ=%d curi=%d",
             event->subscribe.conn_handle, event->subscribe.attr_handle,
             event->subscribe.reason, event->subscribe.prev_notify,
             event->subscribe.cur_notify, event->subscribe.prev_indicate,
             event->subscribe.cur_indicate);
}

/*
 *  GATT server initialization
 *      1. Initialize GATT service
 *      2. Update NimBLE host GATT services counter
 *      3. Add GATT services to server
 */
int gatt_svc_init(void) {
    /* Local variables */
    int rc = 0;

    /* 1. GATT service initialization */
    ble_svc_gatt_init();

    /* 2. Update GATT services counter */
    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    /* 3. Add GATT services */
    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    return 0;
}
