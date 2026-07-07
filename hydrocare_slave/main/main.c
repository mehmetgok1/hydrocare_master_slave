#include <inttypes.h>    // For vTaskDelay if uncommented
#include "communication.h"
#include "freertos/FreeRTOS.h"
static const char *TAG = "MAIN";


void app_main(void) {
  initPeripherals();

  //start slave spi
  initSPIComm();

  // Start background tasks
  setup_timer();
  startSpiCommandHandlerTask();
  startHighSpeedSamplerTask();
  startMeasurementTask();
  initIRSamplerTask();
  initBMESamplerTask();
  vTaskDelay(pdMS_TO_TICKS(50));  // Allow tasks to initialize
  
  // The main loop is now free. The SPI commands are handled by interrupts.
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
/*


ESP_LOGI(TAG, "✓ All background tasks started");
  float *thermal_frame =malloc(192 * sizeof(float));
  float tamb;
  bool status = false;
  while (1){
    status = read_thermal_matrix_frame(thermal_frame,&tamb);
    if (status) {
      ESP_LOGI(TAG, "Thermal Frame Data - First Pixel: %.2f", thermal_frame[0]);
      ESP_LOGI(TAG, "Thermal Frame Data - Last Pixel: %.2f", thermal_frame[191]);
      ESP_LOGI(TAG, "Thermal Frame Data - Middle Pixel: %.2f", thermal_frame[95]);
      //free(thermal_frame);
    } else {
      ESP_LOGE(TAG, "Failed to read thermal frame data");
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
*/