#include "measurement.h"

static const char *TAG = "MEASUREMENT";

/*// Helper function to read multiple bytes from a register via SPI
static void readMultiple(spi_device_handle_t spi, uint8_t startReg, uint8_t *buffer, uint8_t len)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.cmd = 0xC0 | startReg; // Read + Auto increment
    t.length = len * 8;
    t.rx_buffer = buffer;
    spi_device_polling_transmit(spi, &t);
}*/

/*void readAcceleration()
{
  uint8_t rawData[6];
  readMultiple(get_spi_imu_handle(), OUT_X_L, rawData, 6);

  int16_t x = (int16_t)(rawData[1] << 8 | rawData[0]);
  int16_t y = (int16_t)(rawData[3] << 8 | rawData[2]);
  int16_t z = (int16_t)(rawData[5] << 8 | rawData[4]);

  // High-resolution mode: 12-bit left-aligned
  x >>= 4;
  y >>= 4;
  z >>= 4;

  // ±2g sensitivity = 1 mg/LSB
  ax = x * 0.001f;
  ay = y * 0.001f;
  az = z * 0.001f;
}*/

/*void measureAmbLight()
{
  int raw = 0;
  // Read from ADC_CHANNEL_0 (GPIO 1) using the new one-shot driver
  // We get the handle from the config component now
  ESP_ERROR_CHECK(adc_oneshot_read(get_adc1_handle(), ADC_CHANNEL_0, &raw));

  // We also get the calibration handles and status via getter functions
  int voltage_mv = 0;
  if (adc_cali_enabled_chan0) {
      ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle_chan0, raw, &voltage_mv));
  } else {
      // Fallback to rough calculation if calibration failed
      voltage_mv = (raw / 4095.0) * VREF * 1000;
  }

  float current = (voltage_mv / 1000.0) / R_LOAD; // in Amps
  float current_uA = current * 1e6;               // convert to microamps
  ambLight = (uint16_t)current_uA; // calibration factor (adjust!)
}*/

/*void measureMicrophone()
{
  int raw = 0;
  // Read from ADC_CHANNEL_1 (GPIO 2) using the new one-shot driver
  ESP_ERROR_CHECK(adc_oneshot_read(get_adc1_handle(), ADC_CHANNEL_1, &raw));

  int voltage_mv = 0;
  if (is_adc_cali_enabled_chan1()) {
      ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle_chan1, raw, &voltage_mv));
  }

  microphone = (uint16_t)voltage_mv;  // Store in millivolts
}*/

/*void measureIRTemp()
{
    // This function now depends on your C-based MLX90641 driver.
    // The logic remains the same, but the function calls will change.
    // mlx90641_read_temp_c(&myIRcam);
    // mlx90641_clear_new_data_bit(&myIRcam);
}*/



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