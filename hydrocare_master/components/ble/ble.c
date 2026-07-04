#include "ble.h"


// --- Globals ---
//bool deviceConnected = false;
//bool sendRgbFlag = false;
//bool sendIrFlag = false;
//extern bool stream_wifi;
//extern String ssid, password, ver,server_ip; 
//extern bool wifi_connect;  // Flag to trigger WiFi connection in main loop
//extern bool sessionInitialized;  // Set to true when folder/files are ready
//extern uint32_t packetsLogged;   // Bring in the packet counter so we can reset it
//// External buffers from main.cpp and communication functions
//extern uint16_t downsampled16x16[256];  // 16x16 downsampled RGB frame
//extern uint16_t irFrame16x12[192];      // 16x12 IR thermal frame
/*
// Characteristic Pointers
NimBLECharacteristic *pBatChar, *pLuxChar, *pPirChar, *pMmwaveChar, *pActionChar, *pVerChar, *pAmbIntChar;
NimBLECharacteristic *pRgbChar;
NimBLECharacteristic *pIrChar;

#define UUID_RGB "c2a969f6-16e9-4e08-99e7-5e6086f6a546" // Custom UUID for RGB Frame
#define UUID_IR  "d3b969f6-16e9-4e08-99e7-5e6086f6a547" // Custom UUID for IR Frame

// --- Callbacks ---
class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) override {
        deviceConnected = true;
        Serial.print("[BLE] Connected to: ");
        Serial.println(NimBLEAddress(desc->peer_ota_addr).toString().c_str());
        setTimer();
    }
    void onDisconnect(NimBLEServer* pServer) override {
        deviceConnected = false;
        Serial.println("[BLE] Disconnected - Restarting Advertising");
        NimBLEDevice::startAdvertising();
        disableTimer();
    }
};
class ActionCallbacks: public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pCharacteristic) override {
        std::string value = pCharacteristic->getValue();

        if (value.length() > 0) {
            String command = String(value.c_str());
            Serial.print("[BLE] Command Received: ");
            Serial.println(command);
            if (command.startsWith("Com;OTA")) {
                int firstSemi  = command.indexOf(';');
                int secondSemi = command.indexOf(';', firstSemi + 1);
                int thirdSemi  = command.indexOf(';', secondSemi + 1);
                int fourthSemi = command.indexOf(';', thirdSemi + 1);

                if (secondSemi != -1 && thirdSemi != -1 && fourthSemi != -1) {
                    ver = command.substring(secondSemi + 1, thirdSemi);
                    ssid = command.substring(thirdSemi + 1, fourthSemi);
                    password  = command.substring(fourthSemi + 1);

                    Serial.println("[OTA] Starting Update...");
                    Serial.printf("SSID: %s, Ver: %s\n", ssid.c_str(), ver.c_str());

                    otaUpdateAvailable = 1;
                }
            }
            // --- START PARSER (with Label) ---
            else if(command.startsWith("Com;Start")){
                // command is "Com;Start;session001"
                int firstSemi  = command.indexOf(';');          // Position of 1st ';'
                int secondSemi = command.indexOf(';', firstSemi + 1); // Position of 2nd ';'
                String label = "Default";
                // If there is a semicolon after "Start", the label starts at secondSemi + 1
                if (secondSemi != -1 && command.length() > secondSemi + 1) {
                    label = command.substring(secondSemi + 1);
                    label.trim(); // Remove any hidden newline or carriage return characters
                }

                deviceStatus = 1;
                Serial.println("\n[BLE] Logging started - Creating new session folder...");
                initSessionFolder();     
                packetsLogged = 0;          // Reset file numbering to _part_0.bin
                sessionInitialized = true;  // NOW safe to log
                Serial.println("[BLE] Session files ready for logging");
            }
            
            // --- STOP PARSER ---
            else if(command.startsWith("Com;Stop")){
                deviceStatus = 0;
                sessionInitialized = false;  // Reset for next session
                Serial.println("[SD] Stop Logging. files will be transmitted over TCP socket.");
                stream_wifi=true;
            }
            // --- WiFi PARSER (with Label) ---
            else if (command.startsWith("Com;WiFi")) {
                // Format: Com;WiFi;SSID;PASSWORD;IP
                int firstSemi  = command.indexOf(';');
                int secondSemi = command.indexOf(';', firstSemi + 1);
                int thirdSemi  = command.indexOf(';', secondSemi + 1);
                int fourthSemi = command.indexOf(';', thirdSemi + 1);
                if (secondSemi != -1 && thirdSemi != -1 && fourthSemi != -1) {
                    String ssid_pre     = command.substring(secondSemi + 1, thirdSemi);
                    String password_pre = command.substring(thirdSemi + 1, fourthSemi);
                    String ipAddr_pre   = command.substring(fourthSemi + 1);
                    
                    // Update your globals
                    ssid = ssid_pre;
                    password = password_pre;
                    server_ip = ipAddr_pre; 
                    Serial.printf("[BLE] WiFi Config updated: %s, Server: %s:8080\n", 
                                ssid.c_str(), server_ip.c_str());
                }
                wifi_connect = true;  // Trigger WiFi connection in main loop
            }
            // --- CONTROL PARSER ---
            else if (command.startsWith("Com;Control")) {
                int firstSemi = command.indexOf(';');
                int secondSemi = command.indexOf(';', firstSemi + 1);
                int thirdSemi = command.indexOf(';', secondSemi + 1);

                if (secondSemi != -1 && thirdSemi != -1) {
                    String type = command.substring(secondSemi + 1, thirdSemi);
                    String valStr = command.substring(thirdSemi + 1);
                    int val = valStr.toInt();

                    Serial.printf("[BLE] Control %s: %d\n", type.c_str(), val);
                    // Add hardware control logic here
                    // e.g., if (type == "IR") { ... }
                    if(type == "IR"){
                        Serial.print("IR LED status is ");
                        Serial.println(val);
                        sendIRLED(val);
                    }
                    if(type == "LED"){
                        Serial.print("Power LED brightness is ");
                        Serial.println(val);
                        sendBrightness(val);
                    }
                }
            }
            // --- RGB REQUEST PARSER ---
            else if (command.startsWith("Com;RGB")) {
                Serial.println("[BLE] Command RGB received. Queueing frame transmission...");
                sendRgbFlag = true;
            }
            // --- FRAMES REQUEST PARSER ---
            else if (command.startsWith("Com;Frames")) {
                Serial.println("[BLE] Command Frames received. Queueing RGB and IR frame transmission...");
                sendRgbFlag = true;
                sendIrFlag = true;
            }
            // --- SET TIME PARSER ---
            else if (command.startsWith("Com;SetTime")) {
                int firstSemi = command.indexOf(';');
                int secondSemi = command.indexOf(';', firstSemi + 1);
                
                if (secondSemi != -1) {
                    String timestampStr = command.substring(secondSemi + 1);
                    timestampStr.trim();
                    time_t timestamp = timestampStr.toInt();
                    
                    if (timestamp > 0) {
                        struct timeval tv;
                        tv.tv_sec = timestamp;
                        tv.tv_usec = 0;
                        settimeofday(&tv, NULL);
                        
                        // Show confirmation
                        time_t now = time(nullptr);
                        char timeStr[30];
                        strftime(timeStr, sizeof(timeStr), "%Y%m%d_%H%M%S", localtime(&now));
                        Serial.printf("[BLE] System time set to: %s\n", timeStr);
                    } else {
                        Serial.println("[BLE] Invalid timestamp");
                    }
                }
            }
        }
    }
};*/
/* Library function declarations */
void ble_store_config_init(void);

/* Private functions */
static void on_stack_reset(int reason) {
    /* On reset, print reset reason to console */
    ESP_LOGI(TAG, "nimble stack reset, reset reason: %d", reason);
}

static void on_stack_sync(void) {
    /* On stack sync, do advertising initialization */
    adv_init();
}

static void nimble_host_config_init(void) {
    /* Set host callbacks */
    ble_hs_cfg.reset_cb = on_stack_reset;
    ble_hs_cfg.sync_cb = on_stack_sync;
    ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    /* Store host configuration */
    ble_store_config_init();
}

static void nimble_host_task(void *param) {
    /* Task entry log */
    ESP_LOGI(TAG, "nimble host task has been started!");

    /* This function won't return until nimble_port_stop() is executed */
    nimble_port_run();

    /* Clean up at exit */
    vTaskDelete(NULL);
}


void init_ble() {
    BaseType_t rc = 0;
    esp_err_t ret;
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to initialize nvs flash, error code: %d ", ret);
        return;
    }
    /* NimBLE stack initialization */
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to initialize nimble stack, error code: %d ",
                 ret);
        return;
    }
    /* GAP service initialization */
    rc = gap_init();
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to initialize GAP service, error code: %d", rc);
        return;
    }
    /* GATT server initialization */
    rc = gatt_svc_init();
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to initialize GATT server, error code: %d", rc);
        return;
    }
    /* NimBLE host configuration initialization */
    nimble_host_config_init();

    /* Start NimBLE host task thread and return */
    rc = xTaskCreate(nimble_host_task, "NimBLE Host", 4 * 1024, NULL,
                                5, NULL);
    if (rc != pdPASS) {
        ESP_LOGE(TAG, "failed to create NimBLE host task");
        return;
    }

}