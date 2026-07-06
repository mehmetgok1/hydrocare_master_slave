#include "ble.h"

/* Private function declarations */
static int gatt_svr_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg);
static void handle_action_write(struct os_mbuf *om);



/* Characteristic value handles (used for sending notifications) */
uint16_t bat_val_handle;
uint16_t lux_val_handle;
uint16_t pir_val_handle;
uint16_t mmwave_val_handle;
uint16_t amb_int_val_handle;
uint16_t rgb_val_handle;
uint16_t ir_val_handle;

/* credentials */
char* ssid;
char* password;
char* ver;
char* server_ip;
char sessionFolder[64] = "session_default";
// --- Globals ---
bool deviceStatus = false; // false = stopped, true = logging
bool otaUpdateAvailable = false;
bool sendRgbFlag = false;
bool sendIrFlag = false;
bool stream_wifi =false ; 
bool wifi_connect = false;  // Flag to trigger WiFi connection in main loop
bool sessionInitialized = false;  // Set to true when folder/files are ready
uint32_t packetsLogged = 0;   // Bring in the packet counter so we can reset it

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
              .access_cb = gatt_svr_chr_access, .flags = BLE_GATT_CHR_F_READ},
             {.uuid = &gatt_bat_uuid.u,
              .access_cb = gatt_svr_chr_access,
              .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
              .val_handle = &bat_val_handle},
             {.uuid = &gatt_lux_uuid.u,
              .access_cb = gatt_svr_chr_access,
              .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
              .val_handle = &lux_val_handle},
             {.uuid = &gatt_pir_uuid.u,
              .access_cb = gatt_svr_chr_access,
              .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
              .val_handle = &pir_val_handle},
             {.uuid = &gatt_mmwave_uuid.u,
              .access_cb = gatt_svr_chr_access,
              .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
              .val_handle = &mmwave_val_handle},
             {.uuid = &gatt_amb_int_uuid.u,
              .access_cb = gatt_svr_chr_access,
              .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
              .val_handle = &amb_int_val_handle},
             {.uuid = &gatt_action_uuid.u,
              .access_cb = gatt_svr_chr_access,
              .flags = BLE_GATT_CHR_F_WRITE},
             {.uuid = &gatt_rgb_uuid.u,
              .access_cb = gatt_svr_chr_access,
              .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
              .val_handle = &rgb_val_handle},
             {.uuid = &gatt_ir_uuid.u,
              .access_cb = gatt_svr_chr_access,
              .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
              .val_handle = &ir_val_handle},
             {0} /* No more characteristics in this service */
         }
    },
    {0} /* No more services in this table */
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
                        otaUpdateAvailable = true;
                    }
                }
            }
        }
    } else if (strncmp(command, "Com;Start", 9) == 0) {
        deviceStatus = true;
        ESP_LOGI(TAG, "\n[BLE] Logging started - Creating new session folder...");
        time_t now;
        struct tm timeinfo;
        // 1. Get current time
        time(&now);
        // 2. Convert to local time
        localtime_r(&now, &timeinfo);
        strftime(sessionFolder, sizeof(sessionFolder), "%y%m", &timeinfo);
        packetsLogged = 0;
        sessionInitialized = true;
        ESP_LOGI(TAG, "[BLE] Session files ready for logging");
    } else if (strncmp(command, "Com;Stop", 8) == 0) {
        deviceStatus = false;
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
                    size_t type_len = p3 - (p2 + 1);
                    
                    // Prevent buffer overflow by clamping length
                    if (type_len >= sizeof(type)) {
                        type_len = sizeof(type) - 1;
                    }
                    
                    strncpy(type, p2 + 1, type_len);
                    type[type_len] = '\0';
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
        // Note: Implement os_mbuf_append logic here if you want BLE clients 
        // to receive data when they manually read a characteristic.
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
 * Handle GATT attribute register events
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
 * GATT server subscribe event callback
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
 * GATT server initialization
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

    // Allocate memory for credentials
    ssid = malloc(64);
    password = malloc(64);
    ver = malloc(32);
    server_ip = malloc(16);

    return 0;
}

/*handlers getter functions*/
uint16_t* get_bat_val_handle(void) {
    return &bat_val_handle;
}
uint16_t* get_lux_val_handle(void) {
    return &lux_val_handle;
}
uint16_t* get_pir_val_handle(void) {
    return &pir_val_handle;
}
uint16_t* get_mmwave_val_handle(void) {
    return &mmwave_val_handle;
}
uint16_t* get_amb_int_val_handle(void) {
    return &amb_int_val_handle;
}
uint16_t* get_rgb_val_handle(void) {
    return &rgb_val_handle;
}
uint16_t* get_ir_val_handle(void) {
    return &ir_val_handle;
}
char* get_ssid(void) {
    return ssid;
}
char* get_password(void) {
    return password;
}
char* get_ver(void) {
    return ver;
}
char* get_server_ip(void) {
    return server_ip;
}
char* get_sessionFolder(void) {
    return sessionFolder;
}
bool get_deviceStatus(void) {
    return deviceStatus;
}
bool get_otaUpdateAvailable(void) {
    return otaUpdateAvailable;
}
bool get_sendRgbFlag(void) {
    return sendRgbFlag;
}
bool get_sendIrFlag(void) {
    return sendIrFlag;
}
bool* get_stream_wifi(void) {
    return &stream_wifi;
}
bool* get_wifi_connect(void) {
    return &wifi_connect;
}
bool get_sessionInitialized(void) {
    return sessionInitialized;
}
uint32_t* get_packetsLogged(void) {
    return &packetsLogged;
}