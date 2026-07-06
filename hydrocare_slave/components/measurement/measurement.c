#include "measurement.h"



static const char *TAG = "MEASUREMENT";

//ircam

bool measureAmbLight(uint16_t* ambLight)
{
    static uint8_t result[256];
    uint32_t out_len;
    adc_continuous_handle_t handle = get_adc_cont_handle();

    // Read the latest chunk of data from the DMA buffer.
    esp_err_t err = adc_continuous_read(handle, result, sizeof(result), &out_len, 0);
    if (err == ESP_ERR_TIMEOUT) {
        return false; // No new data, can be retried.
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Continuous ADC Read failed for ambient light: %s", esp_err_to_name(err));
        return false;
    }

    // Find the last sample for ADC_CHANNEL_0 in the buffer
    int raw = -1;
    for (int i = 0; i < out_len; i += sizeof(adc_digi_output_data_t)) {
        adc_digi_output_data_t *p = (void*)&result[i];
        if (p->type2.channel == ADC_CHANNEL_0) {
            raw = p->type2.data;
        }
    }

    if (raw == -1) return false; // Channel 0 data not found in this chunk

    int voltage_mv = 0;
    if (is_adc_cali_enabled_chan0()) { 
        adc_cali_raw_to_voltage(get_adc1_cali_handle_chan0(), raw, &voltage_mv);
    } else {
        voltage_mv = (int)((raw / 4095.0f) * VREF * 1000.0f);
    }
    *ambLight = (uint16_t)((voltage_mv * 1000.0f) / R_LOAD);
    return true;
}

bool measureMicrophone(uint16_t* mic_result)
{
    // This buffer should be large enough for one conversion frame
    static uint8_t result[256];
    uint32_t out_len;
    adc_continuous_handle_t handle = get_adc_cont_handle();
    
    // Read from DMA buffer. This is non-blocking and returns immediately.
    // It gives us the most recently filled chunk of data.
    esp_err_t err = adc_continuous_read(handle, result, sizeof(result), &out_len, 0);
    if (err == ESP_ERR_TIMEOUT) {
        // This is not a fatal error, just means no new data was ready.
        // The caller can retry or use the last known value.
        return false; 
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Continuous ADC Read failed: %s", esp_err_to_name(err));
        return false;
    }
    // We only need the very last sample from the buffer.
    adc_digi_output_data_t *p = (void*)&result[out_len - sizeof(adc_digi_output_data_t)];
    int raw = -1;
    // Iterate backwards to find the last sample for our channel
    for (int i = out_len - sizeof(adc_digi_output_data_t); i >= 0; i -= sizeof(adc_digi_output_data_t)) {
        p = (void*)&result[i];
        if (p->type2.channel == ADC_CHANNEL_1) {
            raw = p->type2.data;
            break;
        }
    }
    if (raw == -1) return false; // Channel 1 data not found
    int voltage_mv = 0;
    if (is_adc_cali_enabled_chan1()) {
        adc_cali_raw_to_voltage(get_adc1_cali_handle_chan1(), raw, &voltage_mv);
    } else {
        voltage_mv = (int)((raw / 4095.0f) * VREF * 1000.0f);
    }
    *mic_result = (uint16_t)voltage_mv;
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
  uint32_t inverted_pct = 100 - brightness_pct;  
  uint32_t duty_val;         
  // 2. Map 0-100 to 0-1023 (10-bit duty)
  if(inverted_pct == 100) {
    duty_val = 1024; // Fully off
  } else if (inverted_pct == 0) {
    duty_val = 0;    // Fully on  
  }else {
    duty_val = (inverted_pct * 1024) / 100;
  }
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