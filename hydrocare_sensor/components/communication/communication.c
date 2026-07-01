#include "communication.h"

// DMA-capable buffers
uint8_t *rxBuf;
uint8_t *txBuf;
static const char *TAG = "SPI_COMM";

// Global SPI transaction (initialized once, reused for all transactions)
static spi_slave_transaction_t slaveSpiTransaction = {};
bool debug_code=false;
// Global sensor data
static SensorDataPacket currentData = {0};
static int16_t accelX_ring[RING_BUFFER_SIZE] = {0};
static int16_t accelY_ring[RING_BUFFER_SIZE] = {0};
static int16_t accelZ_ring[RING_BUFFER_SIZE] = {0};
static uint16_t microphone_ring[RING_BUFFER_SIZE] = {0};
volatile int ringBufferIndex = 0;  // Index for next write (no mutex needed - single writer)


static uint16_t sequenceNumber = 0;
static lis3dh_float_data_t accel_results={0};
static bme680_values_float_t bme_results={0};
static uint16_t mic_result=0;
static uint16_t ambLight_result=0;
static float mlx90641Frame[192] = {0};
static float Tamb = 0;
// Double-buffer for IR frame sampling (background task writes, collector reads)
static float mlx_frame_buf[2][192] = { {0} };
static float mlx_frame_temp[2] = {0, 0};
static volatile int mlx_write_idx = 0; // index the writer last wrote to
static portMUX_TYPE mlxMux = portMUX_INITIALIZER_UNLOCKED;
static uint16_t rgbFramePtr[4096] = {0};

static SlaveState slaveState = STATE_IDLE;

// ============ FreeRTOS Synchronization ============
static EventGroupHandle_t spiEventGroup = NULL;
static SemaphoreHandle_t currentDataMutex = NULL;
static portMUX_TYPE samplerMux = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t measurementTaskHandle = NULL;
static TaskHandle_t samplerTaskHandle = NULL;
static TaskHandle_t irTaskHandle = NULL;

// Event group bits
#define EVENT_TRIGGER_RECEIVED (1 << 0)    // Trigger command received

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
  if (!spiEventGroup || !currentDataMutex) {
    ESP_LOGE(TAG, "Mutex/EventGroup creation failed!");
    while(1) vTaskDelay(pdMS_TO_TICKS(1000));
  }
  
  // Initialize global SPI transaction structure (done once, reused for all transactions)
  memset(&slaveSpiTransaction, 0, sizeof(spi_slave_transaction_t));
  slaveSpiTransaction.length    = SPI_BUFFER_SIZE * 8;
  slaveSpiTransaction.rx_buffer = rxBuf;
  slaveSpiTransaction.tx_buffer = txBuf;
  
  ESP_LOGI(TAG, "SPI Ready - Concurrent architecture with background measurement task");
}

// ============ Measurement Collection (runs in background task) ============
void collectMeasurementData() {
  uint64_t totalStartTime = esp_timer_get_time() / 1000;
  // Acquire mutex before modifying currentData
  xSemaphoreTake(currentDataMutex, portMAX_DELAY);
  // 1. Ambient light (fast, ~1ms)
  if(!measureAmbLight(&ambLight_result)) {
    ESP_LOGE(TAG, "Failed to measure ambient light!");
  }else {
    currentData.ambientLight = ambLight_result;
  }
  // 2. Grab high-speed accel + mic samples from ring buffer (last 2000 @ 2kHz = 1 second of data)
  int endIdx = ringBufferIndex;
  int startIdx = (endIdx - 2000 + RING_BUFFER_SIZE) % RING_BUFFER_SIZE;
  // Copy 2000 consecutive samples into packet
  taskENTER_CRITICAL(&samplerMux);
  for (int i = 0; i < 2000; i++) {
    int srcIdx = (startIdx + i) % RING_BUFFER_SIZE;
    currentData.accelX_samples[i] = accelX_ring[srcIdx];
    currentData.accelY_samples[i] = accelY_ring[srcIdx];
    currentData.accelZ_samples[i] = accelZ_ring[srcIdx];
    currentData.microphoneSamples[i] = microphone_ring[srcIdx];
  }
  taskEXIT_CRITICAL(&samplerMux);
  currentData.accelSampleCount = 2000;  // Full 1 second of 2kHz data
  currentData.accelX = accelX_ring[endIdx > 0 ? endIdx - 1 : RING_BUFFER_SIZE - 1];
  currentData.accelY = accelY_ring[endIdx > 0 ? endIdx - 1 : RING_BUFFER_SIZE - 1];
  currentData.accelZ = accelZ_ring[endIdx > 0 ? endIdx - 1 : RING_BUFFER_SIZE - 1];
  currentData.gyroX = 0;
  currentData.gyroY = 0;
  currentData.gyroZ = 0;
  // 3. IR temperature frame - read from double-buffered background sampler
  // Pick the buffer not currently being written to
  taskENTER_CRITICAL(&mlxMux);
  int read_idx = 1 - mlx_write_idx;
  taskEXIT_CRITICAL(&mlxMux);
  for (int i = 0; i < 192; i++) {
    currentData.irFrame[i] = (uint16_t)((mlx_frame_buf[read_idx][i] + 40.0f) * 100.0f);
  }
  currentData.temperature = mlx_frame_temp[read_idx];
  
  // 3.5 BME688 Environment Data - GRAB FROM CACHE (background task updates every 200ms)
  measureBME680(&bme_results);
  currentData.humidity = bme_results.humidity;
  // 4. RGB camera frame 
  get_ov2640_image(rgbFramePtr);
  memcpy(currentData.rgbFrame, rgbFramePtr, sizeof(uint16_t) * 4096);
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
      slaveState = STATE_READY_TRANSFER;
      if(debug_code){
        ESP_LOGI(TAG, "Measurement Task: Data ready, awaiting LOCK command from master");
      }
    }
  }
}

// ============ High-Speed Sampler (Callback + Task) for 2kHz Sampling ============

// ISR triggered by hardware timer
static void IRAM_ATTR timer_callback(void *arg) {
  if (!samplerTaskHandle) {
    return;
  }
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(samplerTaskHandle, &xHigherPriorityTaskWoken);
    // If a higher-priority task was unblocked, request a context switch.
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

void setup_timer() {
    const esp_timer_create_args_t timer_args = {
        .callback = &timer_callback,
        .name = "sampler_timer"
    };

    esp_timer_handle_t timer_handle;
  esp_err_t ret = esp_timer_create(&timer_args, &timer_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create sampler timer: %s", esp_err_to_name(ret));
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
  }
    // Start the timer to trigger every 500 microseconds
  ret = esp_timer_start_periodic(timer_handle, 500);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start sampler timer: %s", esp_err_to_name(ret));
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// ============ IR Sampler Task (double-buffered, runs every 100ms) ============
static void irSamplerTask(void *pvParameters) {
  (void) pvParameters;
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(200);

  // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();
  while (1) {
    int write_idx = 1 - mlx_write_idx; // write to the back buffer
    float temp = 0.0f;
    // Read thermal frame into back buffer
    if (read_thermal_matrix_frame(mlx_frame_buf[write_idx], &temp)) {
      mlx_frame_temp[write_idx] = temp;
      ESP_LOGI(TAG, "IR Sampler: Frame written to buffer %d, Temp=%.2f°C", write_idx, temp);
      // Publish the newly written buffer index atomically
      taskENTER_CRITICAL(&mlxMux);
      mlx_write_idx = write_idx;
      taskEXIT_CRITICAL(&mlxMux);
    } else {
      ESP_LOGW(TAG, "IR Sampler: MLX90641 frame read failed, keeping last good buffer");
    }

    // Wait for the next cycle, ensuring a fixed 100ms period.
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

static void highSpeedSamplerTask(void *pvParameters) {
    ESP_LOGI(TAG, "HighSpeedSampler Task Started");

    while (1) {
        if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY)) {
            /* Fine-grained timing to isolate which stage causes overruns */
            int64_t t0 = esp_timer_get_time();
            measureMicrophone(&mic_result);
            int64_t t1 = esp_timer_get_time();
            measureLIS3DH(&accel_results);
            int64_t t2 = esp_timer_get_time();
            taskENTER_CRITICAL(&samplerMux);
            int idx = ringBufferIndex;
            accelX_ring[idx] = (int16_t)(accel_results.ax * 1000);
            accelY_ring[idx] = (int16_t)(accel_results.ay * 1000);
            accelZ_ring[idx] = (int16_t)(accel_results.az * 1000);
            microphone_ring[idx] = mic_result;
            ringBufferIndex = (ringBufferIndex + 1) % RING_BUFFER_SIZE;
            taskEXIT_CRITICAL(&samplerMux);
            int64_t t3 = esp_timer_get_time();
            int64_t mic_us = t1 - t0;
            int64_t lis_us = t2 - t1;
            int64_t crit_us = t3 - t2;
            int64_t total_us = t3 - t0;
            static uint32_t sampler_overrun_count = 0;
            static uint32_t sampler_sample_count = 0;
            sampler_sample_count++;
            if (total_us > 500) {
              sampler_overrun_count++;
              if (debug_code || (sampler_overrun_count % 50 == 0)) {
                ESP_LOGW(TAG, "HighSpeedSampler overrun: %lld us (mic=%lld, lis=%lld, crit=%lld) total_overruns=%u",
                     total_us, mic_us, lis_us, crit_us, sampler_overrun_count);
              }
            }
        }
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
        // Only accept TRIGGER when the previous packet has been fully released.
        if (slaveState == STATE_IDLE) {
          slaveState = STATE_MEASURING;
          txBuf[1] = STATUS_MEASURING;
          // Signal background measurement task
          xEventGroupSetBits(spiEventGroup, EVENT_TRIGGER_RECEIVED);
          if(debug_code){
            ESP_LOGI(TAG, "Measurement triggered");
          }
        } else {
          ESP_LOGW(TAG, "TRIGGER ignored in state %d, txBuf[1]=0x%02X (waiting for LOCK/UNLOCK to finish)", slaveState, txBuf[1]);
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
    8192,         // Stack size (8 KB) - camera, thermal, and BME calls need more headroom
    NULL,           // Parameters
    2,              // Priority (higher than high-speed sampler)
    &measurementTaskHandle,
    0               // Core 0
  );
  ESP_LOGI(TAG, "Measurement task created on Core 0");
}

// Start the high-speed sampler task (continuous 2kHz accel + mic sampling)
void startHighSpeedSamplerTask() {
  // 1. Create the high-priority task that will perform the sampling
  xTaskCreatePinnedToCore(
    highSpeedSamplerTask,
    "HighSpeedSampler",
    8192,          // Stack size (8 KB - safe for SPI + ADC operations)
    NULL,           // Parameters
    0,            // High priority to ensure timely sampling
    &samplerTaskHandle,
    1               // Core 1 (dedicated to sampling, Core 0 free for SPI + measurement)
  );
}
void initIRSamplerTask() {
    // Start IR sampler task to keep thermal frame cache fresh
  // The task will be created if not already running
  if (!irTaskHandle) {
    xTaskCreatePinnedToCore(
      irSamplerTask,
      "IRSampler",
      4096,          // Stack size
      NULL,
      1,
      &irTaskHandle,
      0);
    ESP_LOGI(TAG, "IR sampler task started on Core 0");
  }
}
