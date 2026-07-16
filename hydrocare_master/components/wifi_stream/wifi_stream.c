#include "wifi_stream.h"
#include "cJSON.h"
#include <sys/stat.h>

const char *TAG = "wifi_stream";

void stream_folder_to_tcp(const char* folder_name) {
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!netif || !esp_netif_is_netif_up(netif)){
        ESP_LOGI(TAG, "Wi-Fi is not connected or has no IP! Aborting S3 upload.");
        return;
    }
    char *chunk_buffer = malloc(2048);
    char *json_buffer = malloc(2048); 

    if (!chunk_buffer || !json_buffer) {
        ESP_LOGE(TAG, "Failed to allocate memory");
        return;
    }

    // --- 1. CALCULATE TOTAL SIZE ---
    size_t total_size = 0;
    int max_index = 0;
    
    for (int i = 0; i < 10000; i += 50) {
        char file_path[256];
        snprintf(file_path, sizeof(file_path), "%s/%s/%s_part_%d.bin", 
                 MOUNT_POINT, folder_name, folder_name, i);
        
        struct stat st;
        if (stat(file_path, &st) == 0) {
            total_size += st.st_size;
            max_index = i;
        } else {
            break; // No more files
        }
    }

    if (total_size == 0) {
        ESP_LOGE(TAG, "No files found to upload!");
        free(chunk_buffer);
        free(json_buffer);
        return;
    }
    
    ESP_LOGI(TAG, "Total session size to upload: %zu bytes", total_size);

    // --- 2. GET THE PRESIGNED URL (JUST ONCE) ---
    char kelvin[8] = "0";      // Default fallback
    char brightness[8] = "0";  // Default fallback

    // Find the positions of 'K' and "Br" in the folder name
    const char *k_ptr = strchr(folder_name, 'K');
    const char *br_ptr = strstr(folder_name, "Br");
    if (k_ptr && br_ptr && (br_ptr > k_ptr)) {
        int k_len = br_ptr - k_ptr - 1; // Length of the kelvin string
        
        // Extract Kelvin
        if (k_len > 0 && k_len < sizeof(kelvin)) {
            strncpy(kelvin, k_ptr + 1, k_len);
            kelvin[k_len] = '\0'; // Ensure null-termination
        }
        
        // Extract Brightness
        strncpy(brightness, br_ptr + 2, sizeof(brightness) - 1);
        brightness[sizeof(brightness) - 1] = '\0'; // Ensure null-termination
    }
    char get_url[256];
    snprintf(get_url, sizeof(get_url), "%s?deviceId=%s&kelvin=%s&brightness=%s", 
             API_GATEWAY_URL, get_device_name(), kelvin, brightness);    
    esp_http_client_config_t config_get = {
        .url = get_url,
        .method = HTTP_METHOD_GET,
        .crt_bundle_attach = esp_crt_bundle_attach,           
    };
    esp_http_client_handle_t client_get = esp_http_client_init(&config_get);
    esp_http_client_set_header(client_get, "x-api-key", API_KEY);
    
    // Open the connection manually (0 means we aren't sending a body)
    if (esp_http_client_open(client_get, 0) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open HTTP connection");
        esp_http_client_cleanup(client_get);
        free(chunk_buffer);
        free(json_buffer);
        return; 
    }

    // Fetch the headers so the ESP32 knows how big the incoming JSON is
    esp_http_client_fetch_headers(client_get);
    int status_code = esp_http_client_get_status_code(client_get);
    ESP_LOGI(TAG, "HTTP GET Status Code: %d", status_code);

    // NOW read the response!
    memset(json_buffer, 0, 2048);
    int read_len = esp_http_client_read_response(client_get, json_buffer, 2048);
    
    ESP_LOGI(TAG, "Bytes read from server: %d", read_len);
    ESP_LOGI(TAG, "Raw response from server: %s", json_buffer);

    esp_http_client_cleanup(client_get);

    // --- PARSE JSON ---
    cJSON *json = cJSON_Parse(json_buffer);
    if (json == NULL) {
        ESP_LOGE(TAG, "Failed to parse JSON string");
        free(chunk_buffer);
        free(json_buffer);
        return;
    }

    cJSON *uploadUrlObj = cJSON_GetObjectItem(json, "uploadUrl");
    if (!uploadUrlObj || !uploadUrlObj->valuestring) {
        ESP_LOGE(TAG, "Failed to find uploadUrl in JSON");
        cJSON_Delete(json);
        free(chunk_buffer);
        free(json_buffer);
        return;
    }

    // --- 3. UPLOAD ALL PARTS AS ONE CONTINUOUS FILE ---
    esp_http_client_config_t config_put = {
        .url = uploadUrlObj->valuestring,
        .method = HTTP_METHOD_PUT,
        .timeout_ms = 60000, 
        .crt_bundle_attach = esp_crt_bundle_attach,
        .buffer_size = 2048,      // Increase RX buffer just in case
        .buffer_size_tx = 4096,   // Massive TX buffer for the giant AWS URL!
    };
    esp_http_client_handle_t client_put = esp_http_client_init(&config_put);
    esp_http_client_set_header(client_put, "Content-Type", "application/octet-stream");

    // Open the connection with the TOTAL size
    if (esp_http_client_open(client_put, total_size) == ESP_OK) {
        
        // Loop through the files again and push them into the open connection
        for (int i = 0; i <= max_index; i += 50) {
            char file_path[256];
            snprintf(file_path, sizeof(file_path), "%s/%s/%s_part_%d.bin", 
                     MOUNT_POINT, folder_name, folder_name, i);
            
            FILE* f = fopen(file_path, "rb");
            if (f) {
                size_t read_bytes;
                while ((read_bytes = fread(chunk_buffer, 1, 2048, f)) > 0) {
                    esp_http_client_write(client_put, chunk_buffer, read_bytes);
                }
                fclose(f);
                ESP_LOGI(TAG, "Streamed part %d...", i);
            }
            vTaskDelay(pdMS_TO_TICKS(10)); 
        }
        
        // Finish the upload
        esp_http_client_fetch_headers(client_put);
        int status = esp_http_client_get_status_code(client_put);
        
        if (status == 200) {
            ESP_LOGI(TAG, "S3 Upload SUCCESS! Entire session merged.");
        } else {
            ESP_LOGE(TAG, "S3 Upload FAILED. HTTP Status: %d", status);
        }
    } else {
        ESP_LOGE(TAG, "Failed to open PUT connection to S3");
    }

    esp_http_client_cleanup(client_put);
    cJSON_Delete(json);
    free(chunk_buffer);
    free(json_buffer);
    
    ESP_LOGI(TAG, "Total upload routine finished!");
}