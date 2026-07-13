#include "ota.h"

// --- Configuration ---
#define FIRMWARE_VER   "v1.0.1" // Example version

static const char *TAG = "OTA_UPDATER";

// Event group for WiFi state
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

// --- WiFi Event Handler ---
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi disconnected, retrying...");
        wifi_event_sta_disconnected_t* event = (wifi_event_sta_disconnected_t*) event_data;
        ESP_LOGE("WIFI_REASON", "Reason: %d", event->reason);
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// --- WiFi Initialization ---
void wifi_init_sta(char* ssid, char* password) {
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    // Disable PMF for better compatibility with some routers.
    // A disconnect reason of 204 (ASSOC_LEAVE) can sometimes be caused by PMF negotiation issues.
    wifi_config.sta.pmf_cfg.capable = false;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Waiting for WiFi connection...");
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
}

// --- OTA Update Logic ---
void perform_ota_update(const char* version) {
    char firmware_url[256];
    
    // Construct the URL safely replacing Arduino String manipulation
    snprintf(firmware_url, sizeof(firmware_url), 
             "https://github.com/mehmetgok1/hydrocare_master_slave/releases/download/%s/firmware.bin", version);
             
             
    ESP_LOGI(TAG, "Firmware URL: %s", firmware_url);

    esp_http_client_config_t http_config = {
        .url = firmware_url,
        .buffer_size = 8192,                   // Increased from 4096 to handle GitHub/AWS headers
        .buffer_size_tx = 2048,                // Good practice for secure connections
        .crt_bundle_attach = esp_crt_bundle_attach, 
        .keep_alive_enable = true, 
        .timeout_ms = 10000,
    };

    esp_https_ota_config_t ota_config = {
        .http_config = &http_config,
    };

    ESP_LOGI(TAG, "Starting OTA update...");
    esp_err_t ret = esp_https_ota(&ota_config);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "OTA Complete! Rebooting in 2 seconds...");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        esp_restart();
    } else {
        ESP_LOGE(TAG, "OTA failed! Error: %s", esp_err_to_name(ret));
    }
}
