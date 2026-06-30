#include <inttypes.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h" // For vTaskDelay, pdMS_TO_TICKS if uncommented
#include "freertos/task.h"     // For vTaskDelay if uncommented
#include "config.h"
#include "measurement.h"

static const char *TAG = "MAIN";


void app_main(void) {
  initPeripherals();
  uint16_t* imageData = malloc(CROP_SIZE * CROP_SIZE * sizeof(uint16_t)); // Buffer for the cropped image
  get_ov3660_image(imageData);
  ESP_LOGI(TAG, "Image captured and cropped to %dx%d", CROP_SIZE, CROP_SIZE);
  ESP_LOGI(TAG, "First pixel value: 0x%04X", imageData[0]);
  ESP_LOGI(TAG, "Last pixel value: 0x%04X", imageData[CROP_SIZE * CROP_SIZE - 1]);
  ESP_LOGI(TAG, "Middle pixel value: 0x%04X", imageData[(CROP_SIZE * CROP_SIZE) / 2]);
  free(imageData); // Free the allocated memory for the image buffer
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
