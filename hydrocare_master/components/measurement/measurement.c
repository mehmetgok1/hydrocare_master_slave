#include "measurement.h"
#include "esp_log.h"

static const char *TAG = "measurement";

static uint8_t rxBuf[BUF_SIZE];
static uint16_t rxLen = 0;


void parseAndPrintFrame(const uint8_t* frame, uint16_t len,uint16_t* movingDist, uint8_t* movingEnergy, uint16_t* staticDist, uint8_t* staticEnergy, uint16_t* detectionDist) {
    if (len < 18)          return;
    if (frame[7] != 0xAA)  return;
    if (frame[17] != 0x55) return;

    // Parse data properties exactly as your original architecture did
    *movingDist    = frame[9]  | (frame[10] << 8);
    *movingEnergy  = frame[11];
    *staticDist    = frame[12] | (frame[13] << 8);
    *staticEnergy  = frame[14];
    *detectionDist = frame[15] | (frame[16] << 8);

    //ESP_LOGI(TAG, "%d,%d,%d,%d,%d", 
    //         *movingDist, *movingEnergy, *staticDist, *staticEnergy, *detectionDist);
}
/*// Shared output variables
uint16_t movingDist = 0, staticDist = 0, detectionDist = 0;
uint8_t  movingEnergy = 0, staticEnergy = 0;*/
void measuremmWave(uint16_t* movingDist, uint8_t* movingEnergy, uint16_t* staticDist, uint8_t* staticEnergy, uint16_t* detectionDist) {
    uint8_t single_byte;
    // Read 1 byte at a time from the UART hardware ring buffer.
    // Specifying a 10ms timeout prevents this from blocking your FreeRTOS loop indefinitely.
    while (uart_read_bytes(UART_NUM_1, &single_byte, 1, pdMS_TO_TICKS(10)) > 0) {
        rxBuf[rxLen++] = single_byte;
        if (rxLen > 250) rxLen = 0; // Buffer overflow safety reset
        // Phase 1: Look for the Data Frame Header
        if (rxLen >= 4) {
            uint16_t s = rxLen - 4;
            if (rxBuf[s]   == DATA_FRAME_HEADER[0] && rxBuf[s+1] == DATA_FRAME_HEADER[1] &&
                rxBuf[s+2] == DATA_FRAME_HEADER[2] && rxBuf[s+3] == DATA_FRAME_HEADER[3]) {
                
                // Align the buffer: shift the header to the index 0 position
                memmove(rxBuf, rxBuf + s, 4);
                rxLen = 4;
            }
        }

        // Phase 2: Once header is aligned, check if a complete length packet has arrived
        if (rxLen >= 6 && rxBuf[0] == DATA_FRAME_HEADER[0]) {
            uint16_t dataLen  = rxBuf[4] | (rxBuf[5] << 8);
            uint16_t totalLen = 4 + 2 + dataLen + 4; // Header[4] + LenBytes[2] + Payload[N] + Tail[4]
            
            if (rxLen >= totalLen) {
                // Verify if the tail matches protocol constraints
                if (rxBuf[totalLen-4] == DATA_FRAME_TAIL[0] && rxBuf[totalLen-3] == DATA_FRAME_TAIL[1] &&
                    rxBuf[totalLen-2] == DATA_FRAME_TAIL[2] && rxBuf[totalLen-1] == DATA_FRAME_TAIL[3]) {
                    
                    parseAndPrintFrame(rxBuf, totalLen, movingDist, movingEnergy, staticDist, staticEnergy, detectionDist);
                    rxLen = 0; // Target parsed successfully, wipe index for next frame
                } else {
                    // False alignment: shift left by 1 byte and continue hunting
                    memmove(rxBuf, rxBuf + 1, --rxLen);
                }
            }
        }
    }
}
bool measureBatteryLevel(uint16_t* batteryLevelOut, float* batteryPercentageOut) 
{
  gpio_set_level((gpio_num_t)Batt_EN, 0);
  gpio_set_level((gpio_num_t)CE_En, 1);

  int raw_accum = 0;
  SemaphoreHandle_t adcMutex = get_adc_mutex();
  
  for(int i = 0; i < Batt_Meas_Count; i++){
    if (!adcMutex || xSemaphoreTake(adcMutex, portMAX_DELAY) != pdTRUE) {
      ESP_LOGE("MEASUREMENT", "ADC mutex unavailable for battery read!");
      return false;
    }
    
    int raw = 0;
    esp_err_t err = adc_oneshot_read(get_adc1_handle(), ADC_CHANNEL_1, &raw);
    xSemaphoreGive(adcMutex);
    
    if (err != ESP_OK) {
      // Swapped these two lines so the log actually prints!
      ESP_LOGE("MEASUREMENT", "Failed to read raw ADC1 Channel 1 value!");
      return false; 
    }
    raw_accum += raw;
  }
  
  gpio_set_level((gpio_num_t)Batt_EN, 1);
  gpio_set_level((gpio_num_t)CE_En, 0);

  // 1. Calculate the average raw value
  float avg_raw = (float)raw_accum / Batt_Meas_Count;
  
  // 2. Do the voltage calculation using a local FLOAT
  float voltage = (avg_raw * Batt_VoltDiv_Mult * DigitalSupply) / 4095.0f;
  voltage = voltage * Batt_Const_X - Batt_Const_Y;

  // 3. Apply piecewise logic using the accurate float
  // High clamp
  if (voltage >= 4.20f)
      *batteryPercentageOut = 100.0f;
  // 4.20 → 3.95  (100 → 75)
  else if (voltage < 4.20f && voltage >= 3.95f)
      *batteryPercentageOut = 75.0f + (voltage - 3.95f) * (25.0f / 0.25f);
  // 3.95 → 3.80 (75 → 45)
  else if (voltage < 3.95f && voltage >= 3.80f)
      *batteryPercentageOut = 45.0f + (voltage - 3.80f) * (30.0f / 0.15f);
  // 3.80 → 3.70 (45 → 28)
  else if (voltage < 3.80f && voltage >= 3.70f)
      *batteryPercentageOut = 28.0f + (voltage - 3.70f) * (17.0f / 0.10f);
  // 3.70 → 3.50 (28 → 6)
  else if (voltage < 3.70f && voltage >= 3.50f)
      *batteryPercentageOut = 6.0f + (voltage - 3.50f) * (22.0f / 0.20f);
  // 3.50 → 3.20 (6 → 1)
  else if (voltage < 3.50f && voltage >= 3.20f)
      *batteryPercentageOut = 1.0f + (voltage - 3.20f) * (5.0f / 0.30f);
  // Low clamp
  else if (voltage < 3.20f)
      *batteryPercentageOut = 0.0f;

  // 4. Save the voltage out in millivolts so it safely fits in your uint16_t
  *batteryLevelOut = (uint16_t)(voltage * 1000.0f); 

  return true;
}

bool measurePIR(uint16_t* PIRValue){
  int raw = 0;
  SemaphoreHandle_t adcMutex = get_adc_mutex();
  if (!adcMutex || xSemaphoreTake(adcMutex, portMAX_DELAY) != pdTRUE) {
    ESP_LOGE("MEASUREMENT", "ADC mutex unavailable for ambient light read!");
    return false;
  }
  // 1. Read the raw 12-bit value once
  esp_err_t err = adc_oneshot_read(get_adc1_handle(), ADC_CHANNEL_2, &raw);
  xSemaphoreGive(adcMutex);
  if (err != ESP_OK) {
      ESP_LOGE("MEASUREMENT", "Failed to read raw ADC1 Channel 2 value!");
      return false; // Return false on failure instead of breaking
  }
  //2. Convert raw bits into calibrated Millivolts using the configuration getter
 /* if (is_adc_cali_enabled_chan2()) { 
      adc_cali_raw_to_voltage(get_adc1_cali_handle_chan2(), raw, &voltage_mv);
  }  */
  *PIRValue = (uint16_t)raw;
  //ESP_LOGI(TAG, "PIR Value: %u", *PIRValue);
  return true;
}
bool measureAmbLight(uint16_t* ambLight)
{
  int raw = 0;
  uint16_t voltage_mv = 0;
  SemaphoreHandle_t adcMutex = get_adc_mutex();
  if (!adcMutex || xSemaphoreTake(adcMutex, portMAX_DELAY) != pdTRUE) {
    ESP_LOGE("MEASUREMENT", "ADC mutex unavailable for ambient light read!");
    return false;
  }
    // 1. Read the raw 12-bit value once
  esp_err_t err = adc_oneshot_read(get_adc1_handle(), ADC_CHANNEL_0, &raw);
  xSemaphoreGive(adcMutex);
    if (err != ESP_OK) {
        ESP_LOGE("MEASUREMENT", "Failed to read raw ADC1 Channel 0 value!");
        return false; // Return false on failure instead of breaking
    }
    voltage_mv = (uint16_t)((raw / 4095.0f) * VREF * 1000.0f);
    *ambLight = (uint16_t)((voltage_mv * 1000.0f) / R_LOAD);
    //ESP_LOGI(TAG, "AmbLight Value: %u (from %u mV)", *ambLight, voltage_mv);
    return true;
}

void checkUSB(bool *chargingStatus){

  while(gpio_get_level((gpio_num_t)USB_Voltage) == 1)
  {
    *chargingStatus = true;
    //measureBatteryLevel();
    //uiChargingScenario();
  }
  if(gpio_get_level((gpio_num_t)USB_Voltage) == 0){
    *chargingStatus = false;
  }
}

bool checkButton(){
  if(gpio_get_level((gpio_num_t)Button) == 1){
    vTaskDelay(pdMS_TO_TICKS(25));
    if(gpio_get_level((gpio_num_t)Button) == 1)
      return true;
    else
      return false;
  }
  else
    return false;
}