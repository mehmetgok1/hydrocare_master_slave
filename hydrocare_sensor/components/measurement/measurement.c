#include "measurement.h"



static const char *TAG = "MEASUREMENT";


uint16_t* measureAmbLight(void)
{
    int raw = 0;
    int voltage_mv = 0;
    uint16_t* ambLight = malloc(sizeof(uint16_t));
    // 1. Read the raw 12-bit value once
    esp_err_t err = adc_oneshot_read(get_adc1_handle(), ADC_CHANNEL_0, &raw);
    if (err != ESP_OK) {
        ESP_LOGE("MEASUREMENT", "Failed to read raw ADC1 Channel 0 value!");
        return NULL; // Return previous value on failure instead of breaking
    }

    // 2. Convert raw bits into calibrated Millivolts using the configuration getter
    if (is_adc_cali_enabled_chan0()) { 
        adc_cali_raw_to_voltage(get_adc1_cali_handle_chan0(), raw, &voltage_mv);
    } else {
        // Fallback scaling calculation
        voltage_mv = (int)((raw / 4095.0f) * VREF * 1000.0f);
    }

    // 3. Convert voltage drop over load resistor into Microamps (uA)
    float voltage_v = voltage_mv / 1000.0f;
    float current_amps = voltage_v / R_LOAD;
    float current_uA = current_amps * 1000000.0f;

    *ambLight = (uint16_t)current_uA;
    return ambLight;
}

uint16_t* measureMicrophone(void)
{
    int raw = 0;
    int voltage_mv = 0;
    uint16_t* microphone = malloc(sizeof(uint16_t));

    // 1. Read raw value
    esp_err_t err = adc_oneshot_read(get_adc1_handle(), ADC_CHANNEL_1, &raw);
    if (err != ESP_OK) {
        ESP_LOGE("MEASUREMENT", "Failed to read raw ADC1 Channel 1 value!");
        return NULL; // Return previous value on failure
    }

    // 2. Convert to millivolts using the channel 1 calibration getter
    if (is_adc_cali_enabled_chan1()) {
        adc_cali_raw_to_voltage(get_adc1_cali_handle_chan1(), raw, &voltage_mv);
    } else {
        voltage_mv = (int)((raw / 4095.0f) * VREF * 1000.0f);
    }

    *microphone = (uint16_t)voltage_mv;
    return microphone;
}

float* read_thermal_matrix_frame(void) {
    // 1. Allocate the temporary raw frame storage on the heap (~640 bytes)
    uint16_t* mlx90641FrameData = malloc(320 * sizeof(uint16_t));
    
    // 2. Allocate the final output temperature array on the heap (~768 bytes)
    float* mlx90641Frame = malloc(192 * sizeof(float));
    
    // Safety Catch: Check if any allocation failed
    if (mlx90641FrameData == NULL || mlx90641Frame == NULL) {
        ESP_LOGE("MEASUREMENT", "Heap allocation failed for thermal frame processing!");
        free(mlx90641FrameData); // Safe to pass NULL to free()
        free(mlx90641Frame);
        return NULL;
    }
    
    float emissivity = 0.95; 
    float TR; 

    // 3. Fetch raw data from the camera array (Default Address: 0x33)
    int status = MLX90641_GetFrameData(0x33, mlx90641FrameData);
    if (status < 0) {
        ESP_LOGE("MEASUREMENT", "Error getting frame data from MLX90641");
        free(mlx90641FrameData);
        free(mlx90641Frame);
        return NULL;
    }

    // 4. Calculate the ambient temperature of the sensor body first
    TR = MLX90641_GetTa(mlx90641FrameData, get_mlx90641_params()) - 8.0; 

    // 5. Calculate the real temperatures for all 192 individual pixels!
    MLX90641_CalculateTo(mlx90641FrameData, get_mlx90641_params(), emissivity, TR, mlx90641Frame);

    // 6. Raw data buffer is no longer needed. Clean it up immediately!
    free(mlx90641FrameData);

    // 7. Return the pointer to the calculated temperatures
    return mlx90641Frame;
}


lis3dh_float_data_t* measureLIS3DH()
{
    lis3dh_float_data_t* results = malloc(sizeof(lis3dh_float_data_t));
    if (lis3dh_new_data (get_lis3dh_dev_handle()) &&
        lis3dh_get_float_data (get_lis3dh_dev_handle(), results)) {
            return results;
    }
    free(results);
    return NULL;
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

void get_ov3660_image(uint16_t* currentData)
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

bme680_values_float_t* measureBME680()
{
    bme680_values_float_t* results = malloc(sizeof(bme680_values_float_t));
    uint32_t duration = bme680_get_measurement_duration(get_bme_dev_handle());
    if (bme680_force_measurement(get_bme_dev_handle())) {
        vTaskDelay(duration);
        
        if (bme680_get_results_float(get_bme_dev_handle(), results)) {
            return results;
        }
    }
    free(results);
    return NULL;
}