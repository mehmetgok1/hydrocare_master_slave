#include "communication.h"

// DMA-capable buffers
uint8_t *rxBuf;
uint8_t *txBuf;
static const char *TAG = "COMM";

// Global SPI transaction (initialized once, reused for all transactions)
static spi_slave_transaction_t slaveSpiTransaction = {};
static uint32_t transaction_count = 0;
bool debug_code=false;
// ============ High-Speed Sampling Ring Buffers (2kHz) ============
// 5000-sample buffer = 2.5 seconds of continuous data @ 2kHz
// Large buffer prevents race condition: sampler index always moves far ahead of read position
#define RING_BUFFER_SIZE 5000  // 5 seconds of 1kHz data
int16_t accelX_ring[RING_BUFFER_SIZE] = {0};
int16_t accelY_ring[RING_BUFFER_SIZE] = {0};
int16_t accelZ_ring[RING_BUFFER_SIZE] = {0};
uint16_t microphone_ring[RING_BUFFER_SIZE] = {0};
volatile int ringBufferIndex = 0;  // Index for next write (no mutex needed - single writer)

// Global sensor data
static SensorDataPacket currentData = {0};
static uint16_t sequenceNumber = 0;

// State machine - simplified for new protocol
typedef enum {
  STATE_IDLE = 0,           // No measurement active
  STATE_MEASURING = 1,      // Measurement in progress
  STATE_READY_TRANSFER = 2, // Measurement complete, buffers locked, ready to send
  STATE_ERROR = 3
} SlaveState;

static SlaveState slaveState = STATE_IDLE;
static uint32_t measurementStartTime = 0;
static uint32_t lockStartTime = 0;        // Track when lock was set (for 5-sec timeout)
#define LOCK_TIMEOUT_MS  5000;  // 5 seconds - auto-reset stale locks

// ============ FreeRTOS Synchronization ============
static EventGroupHandle_t spiEventGroup = NULL;
static SemaphoreHandle_t currentDataMutex = NULL;
static TaskHandle_t measurementTaskHandle = NULL;

// Event group bits
#define EVENT_TRIGGER_RECEIVED (1 << 0)    // Trigger command received

// ============ Cached Sensor Data (Double Buffering) ============
// IR cache structure
typedef struct  {
  uint16_t irFrame[192];
  float avgTemp;
  float Ta;
  uint32_t timestamp;
}IRCache;

// BME cache structure
typedef struct {
  float temp;
  float humidity;
  float pressure;
  float gas;
  uint32_t timestamp;
} BMECache;

static IRCache irCache[2] = {0};          // Double buffer (avoid race conditions while reading)
static BMECache bmeCache[2] = {0};        // Double buffer
static volatile int irWriteIdx = 0;       // Background task writes to this index
static volatile int bmeWriteIdx = 0;      // Background task writes to this index
static SemaphoreHandle_t irCacheMutex = NULL;
static SemaphoreHandle_t bmeCacheMutex = NULL;
static TaskHandle_t irTaskHandle = NULL;
static TaskHandle_t bmeTaskHandle = NULL;

// ============ Initialization ============
void initSPIComm() {
  // Allocate DMA buffers
  rxBuf = (uint8_t*) heap_caps_malloc(SPI_BUFFER_SIZE, MALLOC_CAP_DMA);
  txBuf = (uint8_t*) heap_caps_malloc(SPI_BUFFER_SIZE, MALLOC_CAP_DMA);
  
  if (!rxBuf || !txBuf) {
    ESP_LOGE(TAG, "DMA allocation failed!");
    while(1) vTaskDelay(pdMS_TO_TICKS(1000));
  }
  
  // Clear initial buffers
  memset(rxBuf, 0, SPI_BUFFER_SIZE);
  memset(txBuf, 0, SPI_BUFFER_SIZE);
  memset(&currentData, 0, sizeof(SensorDataPacket));
  
  spi_bus_config_t buscfg = {
    .mosi_io_num     = SPI_MOSI,
    .miso_io_num     = SPI_MISO,
    .sclk_io_num     = SPI_SCK,
    .quadwp_io_num   = -1,
    .quadhd_io_num   = -1,
    .max_transfer_sz = SPI_BUFFER_SIZE,  // Supports full sensor packet with DMA
  };

  spi_slave_interface_config_t slvcfg = {
    .spics_io_num  = SPI_CS,
    .flags         = 0,
    .queue_size    = 1,
    .mode          = 0,  // SPI_MODE0
    .post_setup_cb = NULL,
    .post_trans_cb = NULL,
  };

  esp_err_t ret = spi_slave_initialize(SPI2_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "SPI slave init failed: %s", esp_err_to_name(ret));
    while(1) vTaskDelay(pdMS_TO_TICKS(1000));
  }
  
  // Create FreeRTOS synchronization primitives
  spiEventGroup = xEventGroupCreate();
  currentDataMutex = xSemaphoreCreateMutex();
  irCacheMutex = xSemaphoreCreateMutex();
  bmeCacheMutex = xSemaphoreCreateMutex();
  
  if (!spiEventGroup || !currentDataMutex || !irCacheMutex || !bmeCacheMutex) {
    ESP_LOGE(TAG, "Mutex/EventGroup creation failed!");
    while(1) vTaskDelay(pdMS_TO_TICKS(1000));
  }
  
  // Initialize global SPI transaction structure (done once, reused for all transactions)
  memset(&slaveSpiTransaction, 0, sizeof(spi_slave_transaction_t));
  slaveSpiTransaction.length    = SPI_BUFFER_SIZE * 8;
  slaveSpiTransaction.rx_buffer = rxBuf;
  slaveSpiTransaction.tx_buffer = txBuf;
  
  // Pre-fill permanent parts of txBuf
  txBuf[0] = 0x00;  // Dummy byte (always 0x00)
  txBuf[1] = 0x00;  // Status byte (updated on state changes)
  
  ESP_LOGI(TAG, "SPI Ready - Concurrent architecture with background measurement task");
}

// ============ Measurement Collection (runs in background task) ============
void collectMeasurementData() {
  uint64_t totalStartTime = esp_timer_get_time() / 1000;
  uint32_t stepTime = 0;
  
  // Acquire mutex before modifying currentData
  xSemaphoreTake(currentDataMutex, portMAX_DELAY);
  
  // 1. Ambient light (fast, ~1ms)
  stepTime = esp_timer_get_time() / 1000;
  currentData.ambientLight = *measureAmbLight();
  // 2. Grab high-speed accel + mic samples from ring buffer (last 2000 @ 2kHz = 1 second of data)
  // Calculate start index (2000 samples back from current write position)
  // Capture endIdx FIRST - even if sampler advances during copy, we use this snapshot
  stepTime = esp_timer_get_time() / 1000;
  int endIdx = ringBufferIndex;
  int startIdx = (endIdx - 2000 + RING_BUFFER_SIZE) % RING_BUFFER_SIZE;
  
  // Copy 2000 consecutive samples into packet
  // RACE CONDITION SAFE: 5000-sample buffer is large enough that sampler can't catch up
  // While copying (40ms max), sampler advances only ~80 positions (2000 samples = 1 second at 2kHz)
  // With 5000 total slots, there's always massive separation. No overwrite risk!
  for (int i = 0; i < 2000; i++) {
    int srcIdx = (startIdx + i) % RING_BUFFER_SIZE;
    currentData.accelX_samples[i] = accelX_ring[srcIdx];
    currentData.accelY_samples[i] = accelY_ring[srcIdx];
    currentData.accelZ_samples[i] = accelZ_ring[srcIdx];
    currentData.microphoneSamples[i] = microphone_ring[srcIdx];
  }
  currentData.accelSampleCount = 2000;  // Full 1 second of 2kHz data
  
  // Also store most recent single values for backward compatibility
  currentData.accelX = accelX_ring[endIdx > 0 ? endIdx - 1 : RING_BUFFER_SIZE - 1];
  currentData.accelY = accelY_ring[endIdx > 0 ? endIdx - 1 : RING_BUFFER_SIZE - 1];
  currentData.accelZ = accelZ_ring[endIdx > 0 ? endIdx - 1 : RING_BUFFER_SIZE - 1];
  currentData.gyroX = 0;
  currentData.gyroY = 0;
  currentData.gyroZ = 0;
  
  // 3. IR temperature frame - GRAB FROM CACHE (background task updates every 200ms)
  stepTime = esp_timer_get_time() / 1000;
  xSemaphoreTake(irCacheMutex, portMAX_DELAY);
  int irReadIdx = 1 - irWriteIdx;  // Read from buffer NOT being written to
  for (int i = 0; i < 192; i++) {
    currentData.irFrame[i] = irCache[irReadIdx].irFrame[i];
  }
  currentData.temperature = irCache[irReadIdx].avgTemp;
  xSemaphoreGive(irCacheMutex);
  
  // 3.5 BME688 Environment Data - GRAB FROM CACHE (background task updates every 200ms)
  xSemaphoreTake(bmeCacheMutex, portMAX_DELAY);
  int bmeReadIdx = 1 - bmeWriteIdx;  // Read from buffer NOT being written to
  currentData.humidity = bmeCache[bmeReadIdx].humidity;
  xSemaphoreGive(bmeCacheMutex);

  // 4. RGB camera frame 
  uint16_t* rgbFramePtr = malloc(sizeof(uint16_t) * 4096);
  get_ov2640_image(rgbFramePtr);
  memcpy(currentData.rgbFrame, rgbFramePtr, sizeof(uint16_t) * 4096);
  free(rgbFramePtr);
  
  // 5. Timestamp
  currentData.timestamp_ms = esp_timer_get_time() / 1000;
  // Release mutex after all updates complete
  xSemaphoreGive(currentDataMutex);
  uint64_t elapsed = (esp_timer_get_time() / 1000) - totalStartTime;
  // ===== MEASUREMENT SUMMARY WITH SENSOR DATA =====
  if(debug_code){
    ESP_LOGI(TAG, "Measurement Complete in %llu ms", elapsed);
    ESP_LOGI(TAG, "RGB[Fst:0x%04X Mid:0x%04X Last:0x%04X]", currentData.rgbFrame[0], currentData.rgbFrame[2048], currentData.rgbFrame[4095]);
    ESP_LOGI(TAG, "IR[Fst:0x%04X Mid:0x%04X Last:0x%04X]", currentData.irFrame[0], currentData.irFrame[96], currentData.irFrame[191]);
    ESP_LOGI(TAG, "Seq:%d RingBufIdx:%d TxBufReady", sequenceNumber, ringBufferIndex);
  }
  // Buffer info
}

// ============ Background Measurement Task ============
static void measurementCollectorTask(void *pvParameters) {
  while (1) {
    // Wait for trigger event from SPI handler
    EventBits_t uxBits = xEventGroupWaitBits(
      spiEventGroup,
      EVENT_TRIGGER_RECEIVED,
      pdTRUE,  // Clear on exit
      pdFALSE, // Don't wait for all bits
      portMAX_DELAY
    );
    if (uxBits & EVENT_TRIGGER_RECEIVED) {
      if(debug_code){
        ESP_LOGI(TAG, "Measurement Task Triggered! Starting data collection...");
      }
      collectMeasurementData();
      txBuf[1] = STATUS_MEASURED;
      if(debug_code){
        ESP_LOGI(TAG, "Measurement Task: Data ready, awaiting LOCK command from master");
      }
    }
  }
}

// ============ IR Sensor Background Task (Every 200ms) ============
// Continuously reads IR sensor and caches result in double buffer
static void irSensorBackgroundTask(void *pvParameters) {
  TickType_t lastWakeTime = xTaskGetTickCount();
  while (1) {
    // Precise 200ms timing
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(500));
    // Read IR sensor (blocking ~134ms)
    float mlx90641Frame[192] = {0};
    float Tamb = 0;
    bool status   = read_thermal_matrix_frame(mlx90641Frame, &Tamb);
    // Calculate average temperature
    if (!status) {
      ESP_LOGE(TAG, "IR Sensor Task: Failed to read thermal frame data");
      continue;  // Skip this iteration if read failed
    }
    else {
      if(debug_code){
        ESP_LOGI(TAG, "IR Sensor Task: Thermal frame read successfully");
      }
    }
    float avgTemp = 0;
    for (int i = 0; i < 192; i++) {
      avgTemp += mlx90641Frame[i];
    }
    avgTemp /= 192;
    // Write to cache with mutex protection
    xSemaphoreTake(irCacheMutex, portMAX_DELAY);
    int writeIdx = irWriteIdx;
    
    // Convert temperatures to fixed-point format (+ 40 offset, *100 scale)
    for (int i = 0; i < 192; i++) {
      irCache[writeIdx].irFrame[i] = (uint16_t)((mlx90641Frame[i] + 40) * 100);
    }
    irCache[writeIdx].avgTemp = avgTemp;
    irCache[writeIdx].Ta = Tamb; // Assuming myIRcam is globally accessible from its component
    irCache[writeIdx].timestamp = esp_timer_get_time() / 1000;
    
    // Swap write index for next iteration (double buffering)
    irWriteIdx = 1 - irWriteIdx;
    
    xSemaphoreGive(irCacheMutex);
    
    taskYIELD();
  }
}

// ============ BME Sensor Background Task (Every 200ms) ============
// Continuously reads BME688 sensor and caches result in double buffer
// Continuously reads BME688 sensor and caches result in double buffer
static void bmeSensorBackgroundTask(void *pvParameters) {
  TickType_t lastWakeTime = xTaskGetTickCount();
  while (1) {
      // Precise 200ms timing
      vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(200));
      // Read BME sensor (blocking ~123ms with optimized settings)
      bme680_values_float_t *bme_results = measureBME680();
      // 1. Safe Guard: Check if the sensor read was successful
      if (bme_results != NULL) {
          // 2. Write to cache with mutex protection
          if (xSemaphoreTake(bmeCacheMutex, portMAX_DELAY) == pdTRUE) {
              int writeIdx = bmeWriteIdx;
              
              bmeCache[writeIdx].temp = bme_results->temperature;
              bmeCache[writeIdx].humidity = bme_results->humidity;
              bmeCache[writeIdx].pressure = bme_results->pressure;
              bmeCache[writeIdx].gas = bme_results->gas_resistance;
              bmeCache[writeIdx].timestamp = esp_timer_get_time() / 1000;
              
              // Swap write index so the reader knows a new frame is ready
              bmeWriteIdx = 1 - bmeWriteIdx;
              
              xSemaphoreGive(bmeCacheMutex);
          }
          
          // 3. Prevent Memory Leak: Free the results if allocated dynamically
          free(bme_results); 
          
      } else {
          ESP_LOGE("BME_TASK", "Failed to measure BME680 data");
      }
    }
}

// ============ High-Speed Sampler Task (2kHz on Core 1) ============
// Precise microsecond-level sampling for DFT analysis
// 2000 samples/second = 500µs intervals
// Results stored in ring buffers (no mutex needed - single writer)
static void highSpeedSamplerTask(void *pvParameters) {
  int64_t lastSampleTime_us = 0;
  int64_t targetInterval_us = 500;  // 2kHz = one sample every 500 microseconds
  
  ESP_LOGI(TAG, "HighSpeedSampler Started - Sampling accel + mic @ 2kHz (precise microsecond timing)");
  
  // Initialize reference time
  lastSampleTime_us = esp_timer_get_time();
  
  while (1) {
    // Get current time
    int64_t now_us = esp_timer_get_time();
    
    // Check if time for next sample (500µs interval)
    if ((now_us - lastSampleTime_us) >= targetInterval_us) {
      lastSampleTime_us = now_us;
      
      // Read acceleration (FSPI - ~200µs)
      lis3dh_float_data_t *accel_results = measureLIS3DH();
      if(accel_results == NULL) {
        ESP_LOGE(TAG, "Failed to read LIS3DH data in high-speed sampler");
        continue;  // Skip this sample if failed
      }
      
      // Read microphone (ADC - very fast ~10µs)
      uint16_t *mic_results = measureMicrophone();
      if(mic_results == NULL) {
        ESP_LOGE(TAG, "Failed to read microphone data in high-speed sampler");
        continue;  // Skip this sample if failed
      }
      
      // Store in ring buffer (atomic write, single writer)
      int idx = ringBufferIndex;
      accelX_ring[idx] = (int16_t)(accel_results->ax * 1000);
      accelY_ring[idx] = (int16_t)(accel_results->ay * 1000);
      accelZ_ring[idx] = (int16_t)(accel_results->az * 1000);
      microphone_ring[idx] = *mic_results;
      // Advance ring buffer index
      ringBufferIndex = (ringBufferIndex + 1) % RING_BUFFER_SIZE;
    }
    
    // Yield to allow other tasks to run (watchdog, SPI handler, etc)
    // This prevents starving the whole system
    taskYIELD();
  }
}

// ============ Main SPI Command Handler - Address-Based Protocol ============
void receiveCommand() {
  // ===== STEP 1: EXECUTE SINGLE TRANSACTION (BLOCKING) =====
  // Global slaveSpiTransaction reused - initialized once in initSPIComm()
  // txBuf[0] = dummy, txBuf[1] = status (both maintained globally)
  // txBuf[2+] = sensor data (prefilled after LOCK)
  esp_err_t ret = spi_slave_transmit(SPI2_HOST, &slaveSpiTransaction, 100);
  
  if (ret != ESP_OK) {
    if (ret == ESP_ERR_TIMEOUT) {
      return;  // Normal - no master activity yet
    } else {
      ESP_LOGE(TAG, "SPI Error: %s", esp_err_to_name(ret));
      return;
    }
  }
  // ===== STEP 2: PROCESS RECEIVED COMMAND =====
  transaction_count++;
  uint8_t cmdByte = rxBuf[0];
  uint8_t isRead = (cmdByte & PROTO_CMD_READ) ? 1 : 0;
  uint8_t address = cmdByte & PROTO_ADDR_MASK;
  
  if (!isRead) {
    uint8_t dataValue = rxBuf[2];  // Data is in byte 2 (protocol: byte 0=cmd, byte 1=0x00, byte 2=data)
    if (address == ADDR_CTRL) {
      // ===== CONTROL REGISTER: Measurement control =====
      if (dataValue == CTRL_TRIGGER_MEASUREMENT) {
        if(debug_code){
          ESP_LOGI(TAG, "WRITE CTRL: TRIGGER_MEASUREMENT");
        }
        // Only accept TRIGGER if: in correct state AND not in MEASURED limbo
        if ((slaveState == STATE_IDLE || slaveState == STATE_READY_TRANSFER) && txBuf[1] != STATUS_MEASURED) {
          slaveState = STATE_MEASURING;
          txBuf[1] = STATUS_MEASURING;
          // Signal background measurement task
          xEventGroupSetBits(spiEventGroup, EVENT_TRIGGER_RECEIVED);
          if(debug_code){
            ESP_LOGI(TAG, "Measurement triggered");
          }
        } else {
          ESP_LOGW(TAG, "TRIGGER ignored in state %d, txBuf[1]=0x%02X", slaveState, txBuf[1]);
        }
      }
      else if (dataValue == CTRL_LOCK_BUFFERS) {
        if(debug_code){
          ESP_LOGI(TAG, "WRITE CTRL: LOCK_BUFFERS");
        }
        // Lock only if measurement is complete (status byte shows MEASURED)
        if (txBuf[1] == STATUS_MEASURED) {
          xSemaphoreTake(currentDataMutex, portMAX_DELAY);
          currentData.sequence = sequenceNumber++;
          currentData.status = STATUS_LOCKED;
          // (Byte 0=dummy, Byte 1=status, Bytes 2+=data)
          txBuf[1] = STATUS_LOCKED;
          memcpy(txBuf + 2, &currentData, sizeof(SensorDataPacket));
          slaveState = STATE_READY_TRANSFER;
          xSemaphoreGive(currentDataMutex);
        } else {
          ESP_LOGW(TAG, "LOCK called but measurement not ready (status=0x%02X)", txBuf[1]);
        }
      }
      else if (dataValue == CTRL_UNLOCK_BUFFERS) {
        if(debug_code){
          ESP_LOGI(TAG, "WRITE CTRL: UNLOCK_BUFFERS");
        }
        // Master is done reading - release the lock
        if (slaveState == STATE_READY_TRANSFER) {
          slaveState = STATE_IDLE;
          txBuf[1] = 0x00;  // Status back to IDLE
        } else {
          ESP_LOGW(TAG, "UNLOCK called in state %d (expected STATE_READY_TRANSFER=2)", slaveState);
          ESP_LOGW(TAG, "txBuf[1] before UNLOCK handler: 0x%02X", txBuf[1]);
        }
      }
      else {
        ESP_LOGE(TAG, "Unknown CTRL value 0x%02X", dataValue);
      }
    }
    else if (address == ADDR_IR_LED) {
      // ===== IR LED CONTROL (0x00=off, 0x01=on) =====
      ESP_LOGI(TAG, "WRITE IR_LED: %s", dataValue ? "ON" : "OFF");
      bool ledStatus = dataValue ? true : false;
      set_ir_led(ledStatus);
      // Status at [1] stays prefilled
    }
    
    else if (address == ADDR_BRIGHTNESS) {
      // ===== LED BRIGHTNESS (0-100) =====
      ESP_LOGI(TAG, "WRITE BRIGHTNESS: %d%%", dataValue);
      set_led_brightness(dataValue);
      // Status at [1] stays prefilled
    }
    
    else {
      ESP_LOGE(TAG, "WRITE to unknown address 0x%02X", address);
    }
  }else {
    if (address == 0x00) {
      if(debug_code){
        ESP_LOGD(TAG, "READ BULK COMMAND ARRIVED: status=0x%02X", txBuf[1]);
      }
    }
  }
}

// ============ Exposed Functions ============

// Start the background measurement task (call from main setup)
void startMeasurementTask() {
  xTaskCreatePinnedToCore(
    measurementCollectorTask,
    "MeasurementCollector",
    28672,          // Stack size (28 KB) - generous headroom for camera + thermal library operations
    NULL,           // Parameters
    2,              // Priority (higher than high-speed sampler)
    &measurementTaskHandle,
    0               // Core 0
  );
  ESP_LOGI(TAG, "Measurement task created on Core 0");
}

// Start the high-speed sampler task (continuous 1kHz accel + mic sampling)
void startHighSpeedSamplerTask() {
  static TaskHandle_t samplerTaskHandle = NULL;
  
  xTaskCreatePinnedToCore(
    highSpeedSamplerTask,
    "HighSpeedSampler",
    8192,           // Stack size (8 KB - safe for SPI + ADC operations)
    NULL,           // Parameters
    1,              // Priority (low - won't interfere with SPI handler)
    &samplerTaskHandle,
    1               // Core 1 (dedicated to sampling, Core 0 free for SPI + measurement)
  );
  ESP_LOGI(TAG, "High-speed sampler task created on Core 1 @ 2kHz");
}

// Start the IR sensor background task (continuous 200ms sampling with caching)
void startIRSensorTask() {
  xTaskCreatePinnedToCore(
    irSensorBackgroundTask,
    "IRSensorTask",
    8192,           // Stack size (8 KB - sufficient for MLX90641 library)
    NULL,           // Parameters
    1,              // Priority (same as sampler, won't interfere with measurement)
    &irTaskHandle,
    1               // Core 1 (background caching task)
  );
  ESP_LOGI(TAG, "IR sensor task created on Core 1 @ 200ms");
}

// Start the BME sensor background task (continuous 200ms sampling with caching)
void startBMESensorTask() {
  xTaskCreatePinnedToCore(
    bmeSensorBackgroundTask,
    "BMESensorTask",
    8192,           // Stack size (8 KB - sufficient for BME688 library)
    NULL,           // Parameters
    1,              // Priority (same as sampler, won't interfere with measurement)
    &bmeTaskHandle,
    1               // Core 1 (background caching task)
  );
  ESP_LOGI(TAG, "BME sensor task created on Core 1 @ 200ms");

}
