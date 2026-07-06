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
        nimble_port_deinit(); // Add cleanup
        return;
    }
    /* GATT server initialization */
    rc = gatt_svc_init();
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to initialize GATT server, error code: %d", rc);
        nimble_port_deinit();
        return;
    }
    /* NimBLE host configuration initialization */
    nimble_host_config_init();

    /* Start NimBLE host task thread and return */
    nimble_port_freertos_init(nimble_host_task);

}

static void send_single_notification(uint16_t char_val_handle, const void *data, uint16_t len) {
    // Allocate memory for the BLE packet
    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
    if (!om) {
        ESP_LOGE("BLE", "Failed to allocate memory for notification");
        return;
    }

    // Send it! (Ignore the BLE_HS_EDONE error, it just means the client hasn't subscribed yet)
    int rc = ble_gatts_notify_custom(*get_connection_handle(), char_val_handle, om);
    if (rc != 0 && rc != BLE_HS_EDONE) {
        ESP_LOGE("BLE", "Error sending notification: %d", rc);
    }
}
// --- The Main notifyAll Function ---
void notifyAll(uint8_t bat, uint16_t lux, uint8_t pir, uint8_t mmwave, uint16_t amb_int, 
               uint16_t *rgb_frame, uint16_t rgb_len, 
               uint16_t *ir_frame, uint16_t ir_len) 
{
    // 1. Notify simple sensors
    send_single_notification(*get_bat_val_handle(), &bat, sizeof(bat));
    send_single_notification(*get_lux_val_handle(), &lux, sizeof(lux));
    send_single_notification(*get_pir_val_handle(), &pir, sizeof(pir));
    send_single_notification(*get_mmwave_val_handle(), &mmwave, sizeof(mmwave));
    send_single_notification(*get_amb_int_val_handle(), &amb_int, sizeof(amb_int));

    // 2. Notify large frames (only if pointers are valid)
    if (rgb_frame != NULL && rgb_len > 0) {
        send_single_notification(*get_rgb_val_handle(), rgb_frame, rgb_len);
    }
    
    if (ir_frame != NULL && ir_len > 0) {
        send_single_notification(*get_ir_val_handle(), ir_frame, ir_len);
    }
}