#include <inttypes.h>    // For vTaskDelay if uncommented
#include "communication.h"
static const char *TAG = "MAIN";


void app_main(void) {
  initPeripherals();

  //start slave spi
  initSPIComm();

  // Start background tasks
  startHighSpeedSamplerTask();
  startIRSensorTask();
  startBMESensorTask();
  startMeasurementTask();
  
  ESP_LOGI(TAG, "✓ Setup Complete - Ready for SPI with cached sensors");
  while (1) {
    receiveCommand();
    // Add a small delay to prevent the task from.
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
