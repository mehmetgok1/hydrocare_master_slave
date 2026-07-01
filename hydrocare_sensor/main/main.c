#include <inttypes.h>    // For vTaskDelay if uncommented
#include "communication.h"
static const char *TAG = "MAIN";


void app_main(void) {
  initPeripherals();

  //start slave spi
  initSPIComm();

  // Start background tasks
  setup_timer();
  startHighSpeedSamplerTask();
  startMeasurementTask();
  vTaskDelay(pdMS_TO_TICKS(50));  // Allow tasks to initialize
  ESP_LOGI(TAG, "✓ All background tasks started");
  
  while (1) {
    receiveCommand();
  }
}
