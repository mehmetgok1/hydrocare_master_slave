#include "measurement.h"



static const char *TAG = "MEASUREMENT";

//ircam

bool measureAmbLight(uint16_t* ambLight)
{
  int raw = 0;
  int voltage_mv = 0;
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

    // 2. Convert raw bits into calibrated Millivolts using the configuration getter
    if (is_adc_cali_enabled_chan0()) { 
        adc_cali_raw_to_voltage(get_adc1_cali_handle_chan0(), raw, &voltage_mv);
    } else {
        // Fallback scaling calculation
        voltage_mv = (int)((raw / 4095.0f) * VREF * 1000.0f);
    }
    *ambLight = (uint16_t)((voltage_mv * 1000.0f) / R_LOAD);
    return true;
}

bool measureMicrophone(uint16_t* mic_result)
{
  int raw2 = 0;
  int voltage_mv2 = 0;
  SemaphoreHandle_t adcMutex = get_adc_mutex();
  if (!adcMutex || xSemaphoreTake(adcMutex, portMAX_DELAY) != pdTRUE) {
    ESP_LOGE("MEASUREMENT", "ADC mutex unavailable for microphone read!");
    return false;
  }

    esp_err_t err = adc_oneshot_read(get_adc1_handle(), ADC_CHANNEL_1, &raw2);
  xSemaphoreGive(adcMutex);
    if (err != ESP_OK) {
        ESP_LOGE("MEASUREMENT", "Failed to read raw ADC1 Channel 1 value!");
        return false; // Return 0 (False) on failure
    }

    if (is_adc_cali_enabled_chan1()) {
        adc_cali_raw_to_voltage(get_adc1_cali_handle_chan1(), raw2, &voltage_mv2);
    } else {
        voltage_mv2 = (int)((raw2 / 4095.0f) * VREF * 1000.0f);
    }

    *mic_result = (uint16_t)voltage_mv2;
    return true; // Return true on success
}
static uint16_t mlx90641FrameData[242]; // MLX90641 raw frame buffer

bool read_thermal_matrix_frame(float* mlx90641Frame, float* Tamb) {
    // EEPROM data for MLX90641
    if (mlx90641Frame == NULL) {
        ESP_LOGE("MEASUREMENT", "Heap allocation failed for thermal frame processing!");
        return false;
    }
    // 3. Fetch raw data from the camera array (Default Address: 0x33)
    int status = MLX90641_GetFrameData(0x33, mlx90641FrameData);
    if (status < 0) {
        ESP_LOGE("MEASUREMENT", "Error getting frame data from MLX90641");
        return false;
    }
    ESP_LOGI("MEASUREMENT", "data read from MLX90641: first pixel=0x%04X, last pixel=0x%04X", mlx90641FrameData[0], mlx90641FrameData[191]);
    // 4. Calculate the ambient temperature of the sensor body first
    *Tamb = MLX90641_GetTa(mlx90641FrameData, get_mlx90641_params()) - 8.0;
    // 5. Calculate the real temperatures for all 192 individual pixels!
    MLX90641_CalculateTo(mlx90641FrameData, get_mlx90641_params(), 0.95, *Tamb, mlx90641Frame);
    return true;
}


void measureLIS3DH(lis3dh_float_data_t* accel_results)
{
    //lis3dh_new_data (get_lis3dh_dev_handle());
    lis3dh_get_float_data (get_lis3dh_dev_handle(), accel_results);
}

void set_led_brightness(uint8_t brightness_pct) {
  // 1. Sanity check/clamp the percentage input
  if (brightness_pct > 100) {
    ESP_LOGW(TAG, "Brightness percentage %d exceeds 100%%. Clamping.", brightness_pct);
    brightness_pct = 100;
  }                                 
  // 2. Map 0-100 to 0-1023 (10-bit duty)
  uint32_t duty_val = (brightness_pct * 1023) / 100;
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty_val));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
}

void set_ir_led(bool status){
    if(status == 0)
      gpio_set_level(ledCntrlIR, 1);
    else if(status == 1)
      gpio_set_level(ledCntrlIR, 0);
}

void get_ov2640_image(uint16_t* currentData)
{
  camera_fb_t *fb = esp_camera_fb_get();
  if (fb) {
    int startX = (fb->width - CROP_SIZE) / 2;
    int startY = (fb->height - CROP_SIZE) / 2;
    int idx = 0;
    for (int row = 0; row < CROP_SIZE; row++) {
      for (int col = 0; col < CROP_SIZE; col++) {
        int src = ((startY + row) * fb->width + (startX + col)) * 2;
        uint16_t pixel = (fb->buf[src] << 8) | fb->buf[src + 1];
        currentData[idx++] = pixel;
      }
    }
    esp_camera_fb_return(fb);
  } else {
    ESP_LOGE(TAG, "Camera buffer NULL!");
  }
}

void measureBME680(bme680_values_float_t* bme680_results)
{
    uint32_t duration = bme680_get_measurement_duration(get_bme_dev_handle());
    if (bme680_force_measurement(get_bme_dev_handle())) {
        vTaskDelay(duration);
        bme680_get_results_float(get_bme_dev_handle(), bme680_results);
    }
}