#include <inttypes.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h" // For vTaskDelay, pdMS_TO_TICKS if uncommented
#include "freertos/task.h"     // For vTaskDelay if uncommented
#include "config.h"
#include "measurement.h"

static const char *TAG = "MAIN";


void app_main(void) {
  initPeripherals();
  lis3dh_float_data_t* lis3dh_data = measureLIS3DH();
  if (lis3dh_data) {
    ESP_LOGI(TAG, "LIS3DH Data - X: %.2f, Y: %.2f, Z: %.2f", lis3dh_data->x, lis3dh_data->y, lis3dh_data->z);
    free(lis3dh_data);
  } else {
    ESP_LOGE(TAG, "Failed to read data from LIS3DH sensor");
  } 
  //powerLEDInit();
  //initSPIComm();
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
