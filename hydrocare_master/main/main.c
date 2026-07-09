#include "ble.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "gatt_svc.h"
#include <inttypes.h> // Include for portable format specifiers like PRIu32

const char *TAG2 = "MAIN";
bool debug_infos = false; // Set to true to enable detailed debug prints*/

volatile bool timerStream = false;
uint16_t downsampled16x16[256];  // Shared with BLE for transmission
uint16_t irFrame16x12[192];      // Shared with BLE for transmission
#pragma pack(1)
typedef struct {
  float batteryLevel;             
  float batteryPercentage;        
  float ambLight;                 
  uint16_t ambLight_Int;          
  float PIRValue;                 
  uint16_t movingDist;            
  uint8_t movingEnergy;           
  uint16_t staticDist;            
  uint8_t staticEnergy;           
  uint16_t detectionDist;         
  SensorDataPacket slaveData;     
} CombinedDataPacket;
#pragma pack()

TaskHandle_t sdTaskHandle = NULL;
#define NUM_BUFFERS 3
QueueHandle_t dataQueue = NULL;
QueueHandle_t emptyQueue = NULL;
CombinedDataPacket* packetBuffers[NUM_BUFFERS] = {NULL, NULL, NULL};

void sdCardLoggingTask(void *parameter);
void loop();

//intermediate values
uint16_t master_batteryLevel = 0;
float    master_batteryPercentage = 0.0f;
uint16_t master_ambLight = 0;
uint16_t master_PIRValue = 0;
uint16_t master_movingDist = 0;
uint8_t  master_movingEnergy = 0;
uint16_t master_staticDist = 0;
uint8_t  master_staticEnergy = 0;
uint16_t master_detectionDist = 0;


void app_main(void) {
  // Suppress informational logs from the NimBLE stack to clean up the console
  esp_log_level_set("NimBLE", ESP_LOG_WARN);

  initPeripherals();
  init_sd();
  init_ble();
  initTimer(&timerStream);  // Pass the address of timerStream to the timer
  setTimer();               // Start the timer to activate data acquisition
  //disableTimer();
  initSPIComm();
  vTaskDelay(pdMS_TO_TICKS(50));  // Allow tasks to initialize
  dataQueue = xQueueCreate(NUM_BUFFERS, sizeof(CombinedDataPacket*));
  emptyQueue = xQueueCreate(NUM_BUFFERS, sizeof(CombinedDataPacket*));
  
  // Create SD logging task (lower priority, gets 16KB stack - sufficient for CombinedDataPacket)
  xTaskCreatePinnedToCore(
    sdCardLoggingTask,      // Task function
    "SDLoggingTask",        // Task name
    16384,                   // Stack size (Reduced to 4KB since CombinedDataPacket is now static)
    NULL,                   // Parameter
    1,                      // Priority (lower than main)
    &sdTaskHandle,          // Task handle
    0                       // Core 0 
  );
  
  ESP_LOGI(TAG2, "=== HYDROCARE MASTER - SPI SENSOR ACQUISITION ===");

  while (1) {
    loop();
    vTaskDelay(pdMS_TO_TICKS(10));  // Small delay to yield to other tasks
  }

}

// ==================== COMBINED DATA PACKET ====================


// Creates new binary file every 50 packets (e.g., part_0.bin, part_50.bin, part_100.bin)
void sdCardLoggingTask(void *parameter) {
  ESP_LOGI(TAG2, "[SD-TASK] SD logging task started (BINARY mode - 50-packet rotation)");
  FILE *df = NULL; // Native C file pointer
  uint32_t currentFileIndex = 0xFFFFFFFF;
  // Sector buffer to strictly align SD writes across packets
  uint8_t sectorBuffer[4096];
  size_t sectorOffset = 0;
  while(1) {
    CombinedDataPacket* packetToWrite = NULL;
    // Wait with a 100ms timeout
    if (xQueueReceive(dataQueue, &packetToWrite, pdMS_TO_TICKS(100)) == pdTRUE) {
      int64_t taskStart = esp_timer_get_time();  // Start timing (microseconds)
      if (packetToWrite->slaveData.temperature == 0.0 || packetToWrite->slaveData.sequence == 0xFFFF) {
        ESP_LOGE(TAG2, "[SD-TASK-ERR] Dropped invalid packet (seq=%u, temp=%.1f)", 
                packetToWrite->slaveData.sequence, packetToWrite->slaveData.temperature);
        xQueueSend(emptyQueue, &packetToWrite, 0); 
        continue;  
      }  
      // Calculate which part file to use (every 50 packets = new file)
      unsigned long fileIndex = (*get_packetsLogged() / 50) * 50;
      // Only open a new file if we crossed the 50-packet boundary or starting fresh
      if (fileIndex != currentFileIndex || df == NULL) {
        if (df != NULL) {
          // Flush any remaining unaligned bytes to the current file before rotating
          if (sectorOffset > 0) {
            fwrite(sectorBuffer, 1, sectorOffset, df);
            sectorOffset = 0;
          }
          fflush(df); // Force write of any remaining buffered data and metadata
          fclose(df); // Close the previous file safely
          df = NULL;
        }
        char dataFile[128];
        // Note: Incorporating MOUNT_POINT from your sd.h
        snprintf(dataFile, sizeof(dataFile), "%s/%s/%s_part_%lu.bin",
                MOUNT_POINT, get_sessionFolder(), get_sessionFolder(), fileIndex);   
        // "ab" = Append Binary mode
        df = fopen(dataFile, "ab");
        if (df == NULL) {
          ESP_LOGE(TAG2, "[SD-TASK-ERR] Failed to open binary file: %s", dataFile);
          xQueueSend(emptyQueue, &packetToWrite, 0); 
          continue;
        }
        currentFileIndex = fileIndex;
      }
      int64_t writeStart = esp_timer_get_time();  
      size_t totalWritten = 0;
      const uint8_t* pData = (const uint8_t*)packetToWrite;
      size_t remaining = sizeof(CombinedDataPacket);
      int writeErrors = 0;
      while (remaining > 0) {
        // Calculate how much data we can safely fit into our 4KB sector buffer
        size_t spaceLeft = 4096 - sectorOffset;
        size_t chunk = (remaining < spaceLeft) ? remaining : spaceLeft;
        // Copy data to our perfectly-aligned sector buffer
        memcpy(&sectorBuffer[sectorOffset], pData, chunk);
        sectorOffset += chunk;
        pData += chunk;
        remaining -= chunk;
        totalWritten += chunk;
        // Only trigger a physical SD write when we have exactly 4096 bytes
        if (sectorOffset == 4096) {
          // fwrite returns the number of items written (in this case, bytes since size is 1)
          size_t writtenChunk = fwrite(sectorBuffer, 1, 4096, df);
          if (writtenChunk < 4096) {
            writeErrors++;
            if (writeErrors > 15) {
              ESP_LOGE(TAG2, "[SD-TASK-ERR] Max SD write errors reached.");
              break;
            }
            ESP_LOGW(TAG2, "[SD-TASK-WARN] SD stall. Recovering FATFS state (retry %d/15)...", writeErrors);
            fclose(df);
            vTaskDelay(pdMS_TO_TICKS(100)); // Let the SD card finish GC
            char recFile[128];
            snprintf(recFile, sizeof(recFile), "%s/%s/%s_part_%" PRIu32 ".bin",
                    MOUNT_POINT, get_sessionFolder(), get_sessionFolder(), currentFileIndex);
            df = fopen(recFile, "ab");
            if (df == NULL) {
              ESP_LOGE(TAG2, "[SD-TASK-ERR] Failed to reopen file during recovery.");
              break;
            }
            // Rewind state to retry the block
            pData -= chunk;
            remaining += chunk;
            totalWritten -= chunk;
            sectorOffset -= chunk;    
            continue; 
          }       
          writeErrors = 0; 
          sectorOffset = 0; // Empty the buffer
          // Yield to FreeRTOS (was delay(2) in Arduino)
          vTaskDelay(pdMS_TO_TICKS(2)); 
        }
      }
      int64_t writeTime = esp_timer_get_time() - writeStart;
      if (totalWritten != sizeof(CombinedDataPacket)) {
        ESP_LOGE(TAG2, "[SD-TASK-ERR] Incomplete write! Expected %u bytes, wrote %u bytes", 
        sizeof(CombinedDataPacket), totalWritten);
      }
      *get_packetsLogged()=*get_packetsLogged()+1;
      int64_t taskDuration = (esp_timer_get_time() - taskStart) / 1000; // Convert to milliseconds
      if(debug_infos) {
          ESP_LOGI(TAG2, "[SD-TASK] Packet #%" PRIu32 " | Write: %lld us | Total: %lld ms",
                  *get_packetsLogged(), writeTime, taskDuration);
      }
      xQueueSend(emptyQueue, &packetToWrite, 0);
    } else {
        // Timeout hit, check if we need to close the file because logging stopped
        if (get_deviceStatus() == 0 && df != NULL) {
            if (sectorOffset > 0) {
                fwrite(sectorBuffer, 1, sectorOffset, df);
                sectorOffset = 0;
            }
            fflush(df); // IMPORTANT: Flush all buffers to the physical card
            fsync(fileno(df));
            fclose(df);
            df = NULL; // Crucial: clear pointer so it reopens next time
            currentFileIndex = 0xFFFFFFFF; 
            ESP_LOGI(TAG2, "[SD-TASK] Logging stopped, file safely closed.");
         }
    }
  }
}

void loop() {
  
  //checkUSB();
  if (get_otaUpdateAvailable()) {
    uiOTAStarted(NULL,NULL,NULL);
    wifi_init_sta(get_ssid(), get_password());
    perform_ota_update(get_ver());
  }
  if(*get_wifi_connect()) {
    wifi_init_sta(get_ssid(), get_password());;
    *get_wifi_connect() = false;  // Reset flag after connecting
  }
  if(get_deviceStatus()==false && *get_stream_wifi()==true){
    ESP_LOGI(TAG2, "[MAIN] Stopping logging, waiting for SD card to finish writing...");
    
    vTaskDelay(pdMS_TO_TICKS(150)); // Ensure SD task hits its 100ms timeout and safely closes the final file
    
    // 1. Calculate how many buffers we actually allocated
    int allocatedBuffers = 0;
    for(int i=0; i<NUM_BUFFERS; i++) {
      if(packetBuffers[i] != NULL) allocatedBuffers++;
    }
    
    // 2. Wait for all active buffers to return to the emptyQueue (meaning SD task is fully done)
    int waitTime = 0;
    while(uxQueueMessagesWaiting(emptyQueue) < allocatedBuffers && waitTime < 300) {
       vTaskDelay(pdMS_TO_TICKS(10)); // Wait up to 3 seconds
       waitTime++;
    }
    
    // 3. Clear queues and free up the ~72 KB of RAM for the Wi-Fi Stack!
    xQueueReset(dataQueue);
    xQueueReset(emptyQueue);
    
    for(int i=0; i<NUM_BUFFERS; i++) {
      if(packetBuffers[i] != NULL) {
        free(packetBuffers[i]);
        packetBuffers[i] = NULL;
      }
    }
    
    ESP_LOGI(TAG2, "[MAIN] Buffers fully freed. Starting Wi-Fi TCP stream.");
    deallocateSPIBuffer();
    stream_folder_to_tcp(get_sessionFolder(),get_server_ip());
    *get_stream_wifi() = false;
  }
  if (get_deviceConnected() && timerStream == 1 && get_deviceStatus() == 1 && get_sessionInitialized()) {
    int64_t loopStart = esp_timer_get_time();
    // ==================== DYNAMICALLY ALLOCATE BUFFERS ====================
    for(int i=0; i<NUM_BUFFERS; i++) {
      if(packetBuffers[i] == NULL) {
        packetBuffers[i] = (CombinedDataPacket*)malloc(sizeof(CombinedDataPacket));
        if(packetBuffers[i] != NULL) xQueueSend(emptyQueue, &packetBuffers[i], 0);
        else ESP_LOGE(TAG2, "[MAIN-ERR] Failed to allocate heap buffer!");
      }
    }
    allocateSPIBuffer();

    // This large ~24KB packet is declared static to allocate it in .bss once at compile-time,
    // preventing repeated, costly stack allocations and reducing the risk of stack overflow.
    static SensorDataPacket slaveData;
    // ==================== FETCH SLAVE DATA FIRST ====================
    // Now a stack-allocated object, not a pointer
    bool slaveDataValid = readSlaveData(&slaveData);
    
    // ==================== MEASURE MASTER SENSORS ====================
    measureBatteryLevel(&master_batteryLevel, &master_batteryPercentage);
    measureAmbLight(&master_ambLight);
    measurePIR(&master_PIRValue);
    measuremmWave(&master_movingDist, &master_movingEnergy, &master_staticDist, &master_staticEnergy, &master_detectionDist);
    //ESP_LOGI(TAG2, "[MAIN] Master Sensors: Battery=%u (%.1f%%), AmbLight=%u, PIR=%u, mmWave: movingDist=%u, movingEnergy=%u, staticDist=%u, staticEnergy=%u, detectionDist=%u",
    //        master_batteryLevel, master_batteryPercentage, master_ambLight, master_PIRValue,
    //        master_movingDist, master_movingEnergy, master_staticDist, master_staticEnergy, master_detectionDist);// ==================== COMBINE AND PUSH TO QUEUE ====================
    if (slaveDataValid) {
      
      CombinedDataPacket* currentPacket = NULL;
      
      // Try to get a free buffer from the queue (0 block time)
      if (xQueueReceive(emptyQueue, &currentPacket, 0) == pdTRUE) {
        
        currentPacket->batteryLevel = master_batteryLevel;
        currentPacket->batteryPercentage = master_batteryPercentage;
        currentPacket->ambLight = master_ambLight;
        currentPacket->ambLight_Int = master_ambLight;
        currentPacket->PIRValue = master_PIRValue;
        currentPacket->movingDist = master_movingDist;
        currentPacket->movingEnergy = master_movingEnergy;
        currentPacket->staticDist = master_staticDist;
        currentPacket->staticEnergy = master_staticEnergy;
        currentPacket->detectionDist = master_detectionDist;
        
        memcpy(&currentPacket->slaveData, &slaveData, sizeof(SensorDataPacket));
        
        // Pass the filled pointer to the SD task
        xQueueSend(dataQueue, &currentPacket, 0);
      } else {
        ESP_LOGE(TAG2, "[MAIN-ERR] All 3 buffers full! SD card is too slow. Dropping packet.");
      }
    }
    
    // ==================== BLE NOTIFICATION PHASE ====================
    if (slaveDataValid) {
      downsampleRGBFrame(slaveData.rgbFrame, downsampled16x16);
      memcpy(irFrame16x12, slaveData.irFrame, sizeof(irFrame16x12));
    }
    notifyAll(master_batteryPercentage, slaveData.ambientLight, master_PIRValue,master_ambLight, master_movingDist, 
             master_movingEnergy, master_staticDist, master_staticEnergy, master_detectionDist,
                downsampled16x16, sizeof(downsampled16x16), 
                 irFrame16x12, sizeof(irFrame16x12));

    // Total loop execution time
    if(debug_infos) {
        ESP_LOGI(TAG2, "[LOOP] Cycle: %llu ms (< 200 ms timer)", (esp_timer_get_time() - loopStart) / 1000);
        if (slaveDataValid) { // Add this guard to prevent dereferencing a NULL pointer
            ESP_LOGI(TAG2, "[slaveData] accelX: %d | accelY: %d | accelZ: %d | temperature: %.1f | humdity: %.1f | ambienlight: %u | sequence: %u", 
                    slaveData.accelX, slaveData.accelY, slaveData.accelZ, slaveData.temperature, slaveData.humidity, slaveData.ambientLight, slaveData.sequence);
        }
    }else {
        if (slaveDataValid) {
            //printf("%d ", slaveData.sequence);
        }
    }
    timerStream = 0;
  }
}
