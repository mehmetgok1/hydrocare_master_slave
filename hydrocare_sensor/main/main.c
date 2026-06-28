#include <inttypes.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h" // For vTaskDelay, pdMS_TO_TICKS if uncommented
#include "freertos/task.h"     // For vTaskDelay if uncommented
#include "config.h"

static const char *TAG = "MAIN";


void app_main(void) {
  initPins();
  initPeripherals();
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
