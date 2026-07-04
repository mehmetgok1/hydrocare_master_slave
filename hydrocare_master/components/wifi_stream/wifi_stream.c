
#include "wifi_stream.h"
// Note: Replace this extern with a proper configuration getter in a real app

static const char *TAG = "HTTP_STREAM";
static const char *DEVICE_ID = "ESP32S3_UNIT_01";

// Ensure your SD card was mounted to this base path in your app_main()

void stream_folder_to_tcp(const char* folder_name,char* server_ip) {
    int64_t task_start = esp_timer_get_time();
    char server_url[256];
    
    // Construct the backend URL
    snprintf(server_url, sizeof(server_url), "http://%s:8000/upload", server_ip);

    for (int i = 0; i < 10000; i += 50) {
        char file_path[256];
        snprintf(file_path, sizeof(file_path), "%s/%s/%s_part_%d.bin", 
                 MOUNT_POINT, folder_name, folder_name, i);

        // Open the file using standard C library functions mapping to ESP-IDF VFS
        FILE* f = fopen(file_path, "rb");
        if (f == NULL) {
            ESP_LOGI(TAG, "No more files found at index %d. Ending transmission.", i);
            break;
        }

        // Get file size
        struct stat st;
        stat(file_path, &st);
        size_t file_size = st.st_size;

        ESP_LOGI(TAG, "Preparing payload for: %s (%zu bytes)", file_path, file_size);

        // Configure HTTP Client
        esp_http_client_config_t config = {
            .url = server_url,
            .method = HTTP_METHOD_POST,
            .timeout_ms = 10000,
        };
        esp_http_client_handle_t client = esp_http_client_init(&config);

        // Set Headers
        esp_http_client_set_header(client, "Content-Type", "application/octet-stream");
        esp_http_client_set_header(client, "X-Device-ID", DEVICE_ID);
        esp_http_client_set_header(client, "X-Folder-Name", folder_name);
        
        char index_str[16];
        snprintf(index_str, sizeof(index_str), "%d", i);
        esp_http_client_set_header(client, "X-Part-Index", index_str);

        // Start the HTTP connection with the known file size
        esp_err_t err = esp_http_client_open(client, file_size);
        if (err == ESP_OK) {
            // Buffer to stream data from SD to Network
            char buffer[2048]; 
            size_t read_bytes;
            
            // Read from SD and write to HTTP iteratively (prevents memory exhaustion)
            while ((read_bytes = fread(buffer, 1, sizeof(buffer), f)) > 0) {
                int written = esp_http_client_write(client, buffer, read_bytes);
                if (written < 0) {
                    ESP_LOGE(TAG, "Failed to write data to HTTP client");
                    break;
                }
            }

            // Finish the request and check the server response
            esp_http_client_fetch_headers(client);
            int status_code = esp_http_client_get_status_code(client);
            
            if (status_code == 200) {
                ESP_LOGI(TAG, "Successfully uploaded part %d", i);
            } else {
                ESP_LOGE(TAG, "Upload failed for part %d, HTTP Status: %d", i, status_code);
            }
        } else {
            ESP_LOGE(TAG, "Unable to connect to backend server. Error: %s", esp_err_to_name(err));
        }

        // Clean up resources for this iteration
        esp_http_client_cleanup(client);
        fclose(f);
        
        // Yield to allow watchdog and other tasks to run
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }

    int64_t task_duration_ms = (esp_timer_get_time() - task_start) / 1000;
    ESP_LOGI(TAG, "Total upload routine finished in: %lld ms", task_duration_ms);
}