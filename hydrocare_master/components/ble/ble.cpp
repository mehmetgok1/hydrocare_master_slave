#include "ble.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#include "measurement/measurement.h"
#include "timer/timer.h"
#include "ota/ota.h"
#include "ui/ui.h"
#include <esp_wifi.h>
#include <sys/time.h>
#include <time.h>
#include "memory/memory.h"
#include "config/config.h"
#include "communication/communication.h"
#include "wifi_stream/wifi_stream.h"

#define GATTS_TAG "GATTS_DEMO"

// Forward declarations
static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

// --- Globals --- (Kept from original logic)
bool deviceConnected = false;
bool sendRgbFlag = false;
bool sendIrFlag = false;
extern bool stream_wifi;
extern char ssid[64], password[64], ver[16], server_ip[16];
extern bool wifi_connect;  // Flag to trigger WiFi connection in main loop
extern bool sessionInitialized;  // Set to true when folder/files are ready
extern uint32_t packetsLogged;   // Bring in the packet counter so we can reset it
extern uint16_t downsampled16x16[256];  // 16x16 downsampled RGB frame
extern uint16_t irFrame16x12[192];      // 16x12 IR thermal frame

#define PROFILE_NUM 1
#define PROFILE_APP_IDX 0
#define ESP_APP_ID 0x55

static uint8_t adv_config_done = 0;

uint16_t heart_rate_handle_table[HRS_IDX_NB];

static uint8_t service_uuid[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //f0000000-0451-4000-b000-000000000000
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xb0, 0x00, 0x40, 0x51, 0x04, 0x00, 0x00, 0x00, 0xf0
};

/* The length of adv data must be less than 31 bytes */
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0012, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x00,
    .manufacturer_len = 0, 
    .p_manufacturer_data =  NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(service_uuid),
    .p_service_uuid = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006,
    .max_interval = 0x0012,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(service_uuid),
    .p_service_uuid = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

static struct gatts_profile_inst profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

/*
 *  GAP HANDLER
 */
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~(1 << 0));
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~(1 << 1));
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TAG, "Advertising start failed");
            } else {
                ESP_LOGI(GATTS_TAG, "Advertising started successfully");
            }
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TAG, "Advertising stop failed");
            } else {
                ESP_LOGI(GATTS_TAG, "Stop adv successfully\n");
            }
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(GATTS_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
            break;
        default:
            break;
    }
}

void handle_ble_command(const uint8_t* value, uint16_t len) {
    // The command parsing logic from the original ActionCallbacks::onWrite
    // is now implemented here in pure C.
    // Note: This is a simplified version. A robust solution would use strtok_r or similar.
    if (len == 0) return;

    char command_buffer[256];
    memcpy(command_buffer, value, len < sizeof(command_buffer) -1 ? len : sizeof(command_buffer) - 1);
    command_buffer[len] = '\0';

    ESP_LOGI(GATTS_TAG, "Command Received: %s", command_buffer);

    if (strncmp(command_buffer, "Com;Start", 9) == 0) {
        deviceStatus = 1;
        ESP_LOGI(GATTS_TAG, "\n[BLE] Logging started - Creating new session folder...");
        initSessionFolder();
        packetsLogged = 0;
        sessionInitialized = true;
        ESP_LOGI(GATTS_TAG, "[BLE] Session files ready for logging");
    } else if (strncmp(command_buffer, "Com;Stop", 8) == 0) {
        deviceStatus = 0;
        sessionInitialized = false;
        ESP_LOGI(GATTS_TAG, "[SD] Stop Logging. files will be transmitted over TCP socket.");
        stream_wifi = true;
    }
    // Add other command parsers here (Com;WiFi, Com;Control, etc.)
}

/*
 *  GATT SERVER HANDLER
 */
static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
        case ESP_GATTS_REG_EVT: {
            char devName[20];
            uint8_t mac[6];
            esp_read_mac(mac, ESP_MAC_WIFI_STA);
            snprintf(devName, sizeof(devName), "Urinfo_%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
            esp_ble_gap_set_device_name(devName);

            esp_ble_gap_config_adv_data(&adv_data);
            adv_config_done |= (1 << 0);
            esp_ble_gap_config_adv_data(&scan_rsp_data);
            adv_config_done |= (1 << 1);

            esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, HRS_IDX_NB, 0);
            break;
        }
        case ESP_GATTS_CREAT_ATTR_TAB_EVT: {
            if (param->add_attr_tab.status != ESP_GATT_OK){
                ESP_LOGE(GATTS_TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
            }
            else if (param->add_attr_tab.num_handle != HRS_IDX_NB){
                ESP_LOGE(GATTS_TAG, "create attribute table abnormally, num_handle (%d) doesn't equal to HRS_IDX_NB(%d)", param->add_attr_tab.num_handle, HRS_IDX_NB);
            }
            else {
                ESP_LOGI(GATTS_TAG, "create attribute table successfully, the number handle = %d\n",param->add_attr_tab.num_handle);
                memcpy(heart_rate_handle_table, param->add_attr_tab.handles, sizeof(heart_rate_handle_table));
                esp_ble_gatts_start_service(heart_rate_handle_table[IDX_SVC]);
            }
            break;
        }
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(GATTS_TAG, "CONNECT, conn_id %d", param->connect.conn_id);
            deviceConnected = true;
            setTimer();
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(GATTS_TAG, "DISCONNECT, reason 0x%x", param->disconnect.reason);
            deviceConnected = false;
            disableTimer();
            esp_ble_gap_start_advertising(&adv_params);
            break;
        case ESP_GATTS_WRITE_EVT:
            if (!param->write.is_prep){
                ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, handle = %d, value len = %d", param->write.handle, param->write.len);
                if (heart_rate_handle_table[IDX_CHAR_CFG_A] == param->write.handle && param->write.len == 2){
                    uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
                    if (descr_value == 0x0001){
                        ESP_LOGI(GATTS_TAG, "notify enable");
                    } else if (descr_value == 0x0000){
                        ESP_LOGI(GATTS_TAG, "notify disable ");
                    }
                } else if (heart_rate_handle_table[IDX_CHAR_VAL_ACTION] == param->write.handle) {
                    handle_ble_command(param->write.value, param->write.len);
                }
            }
            break;
        default:
            break;
    }
}

void initBLE(void)
{
    esp_err_t ret;

    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_profile_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gatts register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gap register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gatts_app_register(PROFILE_APP_IDX);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
        return;
    }

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(GATTS_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }
}

void sendValue(uint16_t handle_index, const char* val) {
    if (deviceConnected) {
        esp_ble_gatts_send_indicate(profile_tab[PROFILE_APP_IDX].gatts_if,
                                    profile_tab[PROFILE_APP_IDX].conn_id,
                                    heart_rate_handle_table[handle_index],
                                    strlen(val), (uint8_t *)val, false);
    }
}

extern bool debug_infos;
void notifyAll() {
    if (!deviceConnected) return;

    char buffer[64];

    // Notify basic sensors
    snprintf(buffer, sizeof(buffer), "%.1f", batteryPercentage);
    sendValue(IDX_CHAR_VAL_BAT, buffer);

    snprintf(buffer, sizeof(buffer), "%.0f", ambLight);
    sendValue(IDX_CHAR_VAL_LUX, buffer);

    snprintf(buffer, sizeof(buffer), "%.0f", PIRValue);
    sendValue(IDX_CHAR_VAL_PIR, buffer);

    snprintf(buffer, sizeof(buffer), "%d", (int)ambLight_Int);
    sendValue(IDX_CHAR_VAL_AMB_INT, buffer);

    // Notify mmWave data
    snprintf(buffer, sizeof(buffer), "%d,%d,%d,%d,%d",
             (int)movingDist, (int)movingEnergy, (int)staticDist, (int)staticEnergy, (int)detectionDist);
    sendValue(IDX_CHAR_VAL_MMWAVE, buffer);

    if (debug_infos) {
        ESP_LOGI(GATTS_TAG, "[BLE] Notifying all sensors...");
    }
    // sendDownsampledImages(); // This function also needs to be converted to use esp_ble_gatts_send_indicate
}