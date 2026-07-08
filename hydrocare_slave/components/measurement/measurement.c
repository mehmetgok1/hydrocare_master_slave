#include "measurement.h"
#include <stdint.h>

static const char *TAG = "MEASUREMENT";

bool measureAmbLight(uint16_t* ambientlight)
{
    return true; // This function is now a stub, logic is in continuous_adc_task
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


uint8_t measureLIS3DH_FIFO(lis3dh_float_data_fifo_t accel_results_fifo)
{
    return lis3dh_get_float_data_fifo(get_lis3dh_dev_handle(), accel_results_fifo);
}

void measureLIS3DH(lis3dh_float_data_t* accel_results) // Kept for compatibility, but not used by FIFO sampler
{
    // This function now reads a single value, which is less efficient than the FIFO method.
    lis3dh_get_float_data(get_lis3dh_dev_handle(), accel_results);
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

void measureBME680(bme680_values_float_t* bme680_results,uint32_t* duration)
{
    if (bme680_force_measurement(get_bme_dev_handle())) {
        vTaskDelay(*duration);
        bme680_get_results_float(get_bme_dev_handle(), bme680_results);
    }
}