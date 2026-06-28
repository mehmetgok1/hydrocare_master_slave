#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
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
