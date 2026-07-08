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
volatile int accelRingBufferIndex = 0;

static uint16_t microphone_ring[RING_BUFFER_SIZE] = {0};
volatile int micRingBufferIndex = 0;

static uint16_t sequenceNumber = 0;
static lis3dh_float_data_fifo_t accel_results_fifo;
static uint16_t ambLight_result=0;
// Double-buffer for IR frame sampling (background task writes, collector reads)
static float mlx_frame_buf[2][192] = { {0} };
static float mlx_frame_temp[2] = {0, 0};
static volatile int mlx_write_idx = 0; // index the writer last wrote to
static portMUX_TYPE mlxMux = portMUX_INITIALIZER_UNLOCKED;
// Double-buffer for BME680 sampling (background task writes, collector reads)
static bme680_values_float_t bme_cache_buf[2] = { {0} };
static volatile int bme_write_idx = 0;
static bool bme_cache_valid = false;
static portMUX_TYPE bmeMux = portMUX_INITIALIZER_UNLOCKED;
static uint16_t rgbFramePtr[4096] = {0};

static SlaveState slaveState = STATE_IDLE;

// ============ FreeRTOS Synchronization ============
static EventGroupHandle_t spiEventGroup = NULL;
static SemaphoreHandle_t currentDataMutex = NULL;
static portMUX_TYPE samplerMux = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t measurementTaskHandle = NULL;
static TaskHandle_t spiCommandHandlerTaskHandle = NULL;
static TaskHandle_t samplerTaskHandle = NULL;
static TaskHandle_t lis3dhSamplerTaskHandle = NULL;
static TaskHandle_t irTaskHandle = NULL;
static TaskHandle_t bmeTaskHandle = NULL;
static TaskHandle_t s_task_handle = NULL;

// Event group bits
#define EVENT_TRIGGER_RECEIVED (1 << 0)    // Trigger command received

// =========== SPI Interrupt Handling ===========
static SemaphoreHandle_t spiCommandSemaphore = NULL;

// ISR called after each SPI transaction
static void IRAM_ATTR post_trans_cb(spi_slave_transaction_t *trans) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(spiCommandSemaphore, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

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
    .post_trans_cb = post_trans_cb,
  };

  esp_err_t ret = spi_slave_initialize(SPI2_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "SPI slave init failed: %s", esp_err_to_name(ret));
    while(1) vTaskDelay(pdMS_TO_TICKS(1000));
  }
  
  // Create FreeRTOS synchronization primitives
  spiEventGroup = xEventGroupCreate();
  spiCommandSemaphore = xSemaphoreCreateBinary();
  currentDataMutex = xSemaphoreCreateMutex();
  if (!spiEventGroup || !currentDataMutex || !spiCommandSemaphore) {
    ESP_LOGE(TAG, "Mutex/EventGroup creation failed!");
    while(1) vTaskDelay(pdMS_TO_TICKS(1000));
  }
  
  // Initialize global SPI transaction structure (done once, reused for all transactions)
  memset(&slaveSpiTransaction, 0, sizeof(spi_slave_transaction_t));
  slaveSpiTransaction.length    = SPI_BUFFER_SIZE * 8;
  slaveSpiTransaction.rx_buffer = rxBuf;
  slaveSpiTransaction.tx_buffer = txBuf;

  // Queue the first transaction to be ready for the master
  ret = spi_slave_queue_trans(SPI2_HOST, &slaveSpiTransaction, portMAX_DELAY);
  if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to queue initial SPI transaction: %s", esp_err_to_name(ret));
  }
  
  ESP_LOGI(TAG, "SPI Ready - Concurrent architecture with background measurement task");
}

// ============ Measurement Collection (runs in background task) ============
void collectMeasurementData() {
  uint64_t totalStartTime = esp_timer_get_time() / 1000;
  int64_t tAmbientStart = esp_timer_get_time();
  // Acquire mutex before modifying currentData
  xSemaphoreTake(currentDataMutex, portMAX_DELAY);

  // 1. Ambient light (read from ring buffer, updated by ADC task)
  currentData.ambientLight = ambLight_result;

  int64_t tAmbientEnd = esp_timer_get_time();
  int64_t tRingStart = esp_timer_get_time();
  // --- Critical section to copy from ring buffers ---
  taskENTER_CRITICAL(&samplerMux);
  int micEndIdx = micRingBufferIndex;
  int accelEndIdx = accelRingBufferIndex;
  int micStartIdx = (micEndIdx - 400 + RING_BUFFER_SIZE) % RING_BUFFER_SIZE;
  int accelStartIdx = (accelEndIdx - 400 + RING_BUFFER_SIZE) % RING_BUFFER_SIZE;
  for (int i = 0; i < 400; i++) {
    int micSrcIdx = (micStartIdx + i) % RING_BUFFER_SIZE;
    int accelSrcIdx = (accelStartIdx + i) % RING_BUFFER_SIZE;
    currentData.accelX_samples[i] = accelX_ring[accelSrcIdx];
    currentData.accelY_samples[i] = accelY_ring[accelSrcIdx];
    currentData.accelZ_samples[i] = accelZ_ring[accelSrcIdx];
    currentData.microphoneSamples[i] = microphone_ring[micSrcIdx];
  }
  taskEXIT_CRITICAL(&samplerMux);
  int64_t tRingEnd = esp_timer_get_time();
  currentData.accelSampleCount = 400;  // Full 0.2 second of 2kHz data
  currentData.accelX = currentData.accelX_samples[399];
  currentData.accelY = currentData.accelY_samples[399];
  currentData.accelZ = currentData.accelZ_samples[399];
  currentData.gyroX = 0;
  currentData.gyroY = 0;
  currentData.gyroZ = 0;
  // 3. IR temperature frame - read from double-buffered background sampler
  // Pick the buffer not currently being written to
  int64_t tIrStart = esp_timer_get_time();
  taskENTER_CRITICAL(&mlxMux);
  int read_idx = 1 - mlx_write_idx;
  taskEXIT_CRITICAL(&mlxMux);
  for (int i = 0; i < 192; i++) {
    currentData.irFrame[i] = (uint16_t)((mlx_frame_buf[read_idx][i] + 40.0f) * 100.0f);
  }
  currentData.temperature = mlx_frame_temp[read_idx];
  int64_t tIrEnd = esp_timer_get_time();
  
  // 3.5 BME688 Environment Data - GRAB FROM CACHE (background task updates every 250ms)
  int64_t tBmeStart = esp_timer_get_time();
  taskENTER_CRITICAL(&bmeMux);
  int bme_read_idx = bme_write_idx;
  bme680_values_float_t cached_bme = bme_cache_buf[bme_read_idx];
  bool cached_bme_valid = bme_cache_valid;
  taskEXIT_CRITICAL(&bmeMux);
  if (cached_bme_valid) {
    currentData.humidity = cached_bme.humidity;
  } else {
    currentData.humidity = 0.0f;
  }
  int64_t tBmeEnd = esp_timer_get_time();
  // 4. RGB camera frame 
  int64_t tRgbStart = esp_timer_get_time();
  get_ov2640_image(rgbFramePtr);
  memcpy(currentData.rgbFrame, rgbFramePtr, sizeof(uint16_t) * 4096);
  int64_t tRgbEnd = esp_timer_get_time();
  // 5. Timestamp
  currentData.timestamp_ms = esp_timer_get_time() / 1000;
  // Release mutex after all updates complete
  xSemaphoreGive(currentDataMutex);
  uint64_t elapsed = (esp_timer_get_time() / 1000) - totalStartTime;
  // ===== MEASUREMENT SUMMARY WITH SENSOR DATA =====
  if(debug_code){
    ESP_LOGI(TAG, "Measurement Complete in %llu ms", elapsed);
    ESP_LOGI(TAG, "Timing(us): ambient=%lld ring=%lld ir=%lld bme=%lld rgb=%lld total=%lld",
             (long long)(tAmbientEnd - tAmbientStart),
             (long long)(tRingEnd - tRingStart),
             (long long)(tIrEnd - tIrStart),
             (long long)(tBmeEnd - tBmeStart),
             (long long)(tRgbEnd - tRgbStart),
             (long long)((esp_timer_get_time() - tAmbientStart)));
    ESP_LOGI(TAG, "RGB[Fst:0x%04X Mid:0x%04X Last:0x%04X]", currentData.rgbFrame[0], currentData.rgbFrame[2047], currentData.rgbFrame[4095]);
    ESP_LOGI(TAG, "IR[Fst:0x%04X Mid:0x%04X Last:0x%04X]", currentData.irFrame[0], currentData.irFrame[96], currentData.irFrame[191]);
    ESP_LOGI(TAG,"accelX[Fst:%d Mid:%d Last:%d] accelY[Fst:%d Mid:%d Last:%d] accelZ[Fst:%d Mid:%d Last:%d]",
             currentData.accelX_samples[0], currentData.accelX_samples[199], currentData.accelX_samples[399],
             currentData.accelY_samples[0], currentData.accelY_samples[199], currentData.accelY_samples[399],
             currentData.accelZ_samples[0], currentData.accelZ_samples[199], currentData.accelZ_samples[399]);
    ESP_LOGI(TAG, "Seq:%d MicIdx:%d AccelIdx:%d TxBufReady", sequenceNumber, micRingBufferIndex, accelRingBufferIndex);

  }
  // Buffer info
}

// ============ BME680 Sampler Task (double-buffered, runs every 250ms) ============
static void bmeSamplerTask(void *pvParameters) {
  (void) pvParameters;
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(800);
  uint32_t duration = bme680_get_measurement_duration(get_bme_dev_handle());
  xLastWakeTime = xTaskGetTickCount();
  while (1) {
    int write_idx = 1 - bme_write_idx;
    bme680_values_float_t sample = {0};
    int64_t t0 = esp_timer_get_time();
    measureBME680(&sample, &duration);
    int64_t t1 = esp_timer_get_time();

    taskENTER_CRITICAL(&bmeMux);
    bme_cache_buf[write_idx] = sample;
    bme_write_idx = write_idx;
    bme_cache_valid = true;
    taskEXIT_CRITICAL(&bmeMux);

    if (0) {
      ESP_LOGI(TAG, "BME Sampler: cached buffer %d in %lld us (T=%.2f C, H=%.2f %%)",
               write_idx, (long long)(t1 - t0), sample.temperature, sample.humidity);
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
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
      collectMeasurementData();
      txBuf[1] = STATUS_MEASURED;
      slaveState = STATE_READY_TRANSFER;
      if(debug_code){
        ESP_LOGI(TAG, "Measurement Task: Data ready, awaiting LOCK command from master");
      }
    }
  }
}

// ============ IR Sampler Task (double-buffered, runs every 100ms) ============
static void irSamplerTask(void *pvParameters) {
  (void) pvParameters;
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(100);

  // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();
  while (1) {
    int write_idx = 1 - mlx_write_idx; // write to the back buffer
    float temp = 0.0f;
    // Read thermal frame into back buffer
    if (read_thermal_matrix_frame(mlx_frame_buf[write_idx], &temp)) {
      mlx_frame_temp[write_idx] = temp;
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

static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    //Notify that ADC continuous driver has done enough number of conversions
    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);

    return (mustYield == pdTRUE);
}

static void continuous_adc_task(void *pvParameters)
{
    esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[EXAMPLE_READ_LEN] = {0};
    adc_continuous_data_t parsed_result[256]; // Buffer to hold parsed ADC data
    int decimation_count = 0;

    s_task_handle = xTaskGetCurrentTaskHandle();

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(*get_adc_cont_handle(), &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(*get_adc_cont_handle()));

    while(1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        while (1) {
            ret = adc_continuous_read(*get_adc_cont_handle(), result, EXAMPLE_READ_LEN, &ret_num, 0);
            if (ret == ESP_OK) {
                uint32_t parsed_num = 0;
                // Use the official parsing function to correctly interpret the ADC data
                esp_err_t parse_ret = adc_continuous_parse_data(*get_adc_cont_handle(), result, ret_num, parsed_result, &parsed_num);
                if (parse_ret == ESP_OK) {
                  for (uint32_t i = 0; i < parsed_num; i+=2) { // We get 2 channels of data at a time
                    if (parsed_result[i].valid) {
                        // Downsample: 20kHz -> 2kHz means we take 1 of every 10 samples
                      if (decimation_count++ % 10 == 0) {
                        // Assuming channel 0 is Ambient Light and channel 1 is Microphone based on config
                        ambLight_result = parsed_result[i].raw_data;
                        if (i + 1 < parsed_num && parsed_result[i+1].valid) {
                          taskENTER_CRITICAL(&samplerMux);
                          microphone_ring[micRingBufferIndex] = parsed_result[i+1].raw_data;
                          micRingBufferIndex = (micRingBufferIndex + 1) % RING_BUFFER_SIZE;
                          taskEXIT_CRITICAL(&samplerMux);
                        }
                      }
                    }
                  }
                }
            } else if (ret == ESP_ERR_TIMEOUT) {
                break; // No more data to read
            }
        }
    }
}

// ============ High-Speed Sampler (Callback + Task) for 2kHz LIS3DH Sampling ============

// ISR triggered by hardware timer
static void IRAM_ATTR timer_callback(void *arg) {
  if (!lis3dhSamplerTaskHandle) {
    return;
  }
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(lis3dhSamplerTaskHandle, &xHigherPriorityTaskWoken);
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
    // Start the timer to trigger every 6400 microseconds (6.4ms), the time to fill the 32-sample FIFO at 5kHz.
  ret = esp_timer_start_periodic(timer_handle, 6400);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start sampler timer: %s", esp_err_to_name(ret));
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
        /*read_samples=measureLIS3DH_FIFO(accel_results_fifo);
        if(read_samples > 0){
          ESP_LOGI(TAG, "LIS3DH : Samples read from FIFO: %d", read_samples);
          ESP_LOGI(TAG, "LIS3DH : ax=%.3f ay=%.3f az=%.3f", accel_results_fifo[read_samples-1].ax, accel_results_fifo[read_samples-1].ay, accel_results_fifo[read_samples-1].az);
          ESP_LOGI(TAG, "LIS3DH : ax=%.3f ay=%.3f az=%.3f", accel_results_fifo[0].ax, accel_results_fifo[0].ay, accel_results_fifo[0].az);
        }else {
          ESP_LOGW(TAG, "LIS3DH : No samples read from FIFO");
        }*/
static void lis3dh_sampler_task(void *pvParameters) {
    ESP_LOGI(TAG, "LIS3DH Sampler Task Started");
    static uint32_t sampler_overrun_count = 0;
    uint8_t samples_read = 0;
    int64_t t0;
    int64_t t1;
    int64_t t2;
    int64_t total_us;
    while (1) {
      if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY)) {
        t0 = esp_timer_get_time();            
        samples_read = measureLIS3DH_FIFO(accel_results_fifo);
        t1 = esp_timer_get_time();
        if (samples_read > 0) {
          // Downsample from 5kHz to 2kHz.
          // In a 6.4ms window, we get 32 samples at 5kHz.
          // To get a 2kHz rate, we need 6.4ms / (1/2000Hz) = 12.8 samples.
          // We will pick 13 samples from the batch by taking every ~2.5th sample.
          taskENTER_CRITICAL(&samplerMux);
          for (int i = 0; i < 13; i++) {
            int sample_index = (int)(i * 2.5f);
            if (sample_index < samples_read) {
                accelX_ring[accelRingBufferIndex] = (int16_t)(accel_results_fifo[sample_index].ax * 1000);
                accelY_ring[accelRingBufferIndex] = (int16_t)(accel_results_fifo[sample_index].ay * 1000);
                accelZ_ring[accelRingBufferIndex] = (int16_t)(accel_results_fifo[sample_index].az * 1000);
                accelRingBufferIndex = (accelRingBufferIndex + 1) % RING_BUFFER_SIZE;
            }
          }
          taskEXIT_CRITICAL(&samplerMux);
        }
        samples_read = 0; // Reset for next iteration
        t2 = esp_timer_get_time();
        total_us = t2 - t0;
        if (total_us > 6400) {
          sampler_overrun_count++;
          ESP_LOGW(TAG, "LIS3DH Sampler overrun: %lld us (lis=%lld, crit=%lld) total_overruns=%u",
                 total_us, (long long)(t1-t0), (long long)(t2-t1), sampler_overrun_count);
        }
      }
    }
}

// ============ Main SPI Command Handler - Address-Based Protocol ============
void process_spi_command() {
  // The transaction has already happened, we just process the data.
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

static void spiCommandHandlerTask(void *pvParameters) {
    ESP_LOGI(TAG, "SPI Command Handler Task started.");
    while(1) {
        // Wait for a transaction to complete (signaled by the ISR)
        if (xSemaphoreTake(spiCommandSemaphore, portMAX_DELAY) == pdTRUE) {
            // Process the received command
            process_spi_command();

            // Immediately queue the next transaction to be ready for the master.
            // This is crucial for the interrupt-driven model.
            esp_err_t ret = spi_slave_queue_trans(SPI2_HOST, &slaveSpiTransaction, portMAX_DELAY);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to queue subsequent SPI transaction: %s", esp_err_to_name(ret));
            }
        }
    }
}
// ============ Exposed Functions ============
void initBMESamplerTask() {
  if (!bmeTaskHandle) {
    xTaskCreatePinnedToCore(
      bmeSamplerTask,
      "BMESampler",
      4096,
      NULL,
      1,
      &bmeTaskHandle,
      0);
    ESP_LOGI(TAG, "BME sampler task started on Core 0");
  }
}
void startSpiCommandHandlerTask() {
    xTaskCreatePinnedToCore(
        spiCommandHandlerTask,
        "SpiCmdHandler",
        4096,
        NULL,
        4, // High priority to quickly handle SPI commands
        &spiCommandHandlerTaskHandle,
        1 // Core 1 for general tasks
    );
    ESP_LOGI(TAG, "SPI command handler task created on Core 1");
}

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
  xTaskCreatePinnedToCore(
    continuous_adc_task,
    "HighSpeedSampler",
    8192,           // Increased stack for ADC parsing buffer
    NULL,           // Parameters
    4,              // Highest priority to ensure timely sampling and prevent overruns
    &samplerTaskHandle,
    0               // Core 0, same as the timer and other tasks for simplicity
  );
  ESP_LOGI(TAG, "Continuous ADC sampler task created on Core 0");
}

void startLis3dhSamplerTask() {
  xTaskCreatePinnedToCore(
    lis3dh_sampler_task,
    "Lis3dhSampler",
    4096,
    NULL,
    5, // Highest priority
    &lis3dhSamplerTaskHandle,
    0);
  ESP_LOGI(TAG, "LIS3DH sampler task created on Core 0");
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
      2,
      &irTaskHandle,
      0);
    ESP_LOGI(TAG, "IR sampler task started on Core 0");
  }
}
