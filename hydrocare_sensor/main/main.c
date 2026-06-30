#include <inttypes.h>    // For vTaskDelay if uncommented
#include "measurement.h"

static const char *TAG = "MAIN";


void app_main(void) {
  initPeripherals();
  lis3dh_float_data_t* lis3dh_data =malloc(sizeof(lis3dh_float_data_t));
  lis3dh_data = measureLIS3DH();
  if (lis3dh_data) {
    ESP_LOGI(TAG, "LIS3DH Data - X: %.2f, Y: %.2f, Z: %.2f", lis3dh_data->ax, lis3dh_data->ay, lis3dh_data->az);
    free(lis3dh_data);
  } else {
    ESP_LOGE(TAG, "Failed to read data from LIS3DH sensor");
  } 
  float *thermal_frame =malloc(192 * sizeof(float));
  thermal_frame = read_thermal_matrix_frame();
  if (thermal_frame) {
    ESP_LOGI(TAG, "Thermal Frame Data - First Pixel: %.2f", thermal_frame[0]);
    ESP_LOGI(TAG, "Thermal Frame Data - Last Pixel: %.2f", thermal_frame[191]);
    ESP_LOGI(TAG, "Thermal Frame Data - Middle Pixel: %.2f", thermal_frame[95]);
    free(thermal_frame);
  } else {
    ESP_LOGE(TAG, "Failed to read thermal frame data");
  }

  uint16_t* ambLight = malloc(sizeof(uint16_t));
  ambLight = measureAmbLight();
  if (ambLight) {
    ESP_LOGI(TAG, "Ambient Light Measurement: %" PRIu16 " uA", *ambLight);
    free(ambLight);
  } else {
    ESP_LOGE(TAG, "Failed to measure ambient light");
  }
  uint16_t* microphone = malloc(sizeof(uint16_t));
  microphone = measureMicrophone();
  if (microphone) {
    ESP_LOGI(TAG, "Microphone Measurement: %" PRIu16 " mV", *microphone);
    free(microphone);
  } else {
    ESP_LOGE(TAG, "Failed to measure microphone");
  }

  //// Start background tasks
  //startHighSpeedSamplerTask();
  //startIRSensorTask();
  //startBMESensorTask();
  //startMeasurementTask();
  
  ESP_LOGI(TAG, "✓ Setup Complete - Ready for SPI with cached sensors");
  //while (1) {
  //  receiveCommand();
  //  // Add a small delay to prevent the task from.
  //  vTaskDelay(pdMS_TO_TICKS(10));
  //}
}
