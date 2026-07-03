#ifndef CONFIG_H
#define CONFIG_H

#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include <string.h>
#include "led_strip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//external function declarations
void initPeripherals();
led_strip_handle_t get_led_strip_handle(void);
SemaphoreHandle_t get_adc_mutex(void);
adc_cali_handle_t get_adc1_cali_handle_chan0(void);
adc_cali_handle_t get_adc1_cali_handle_chan1(void);
adc_cali_handle_t get_adc1_cali_handle_chan2(void);
bool is_adc_cali_enabled_chan0(void);
bool is_adc_cali_enabled_chan1(void);
bool is_adc_cali_enabled_chan2(void);
//definitions
//extern String fw_version;

//gpio pin definitions
#define NUMPIXELS   1
#define Neopixel    38

#define AmbLight        1
#define Batt_LVL        2
#define PIR             3
#define Button          4
#define mmWave_Out      5
#define SPI_SCK         6
#define SPI_MOSI        7
#define SPI_MISO        8
#define SPI_CS          9
#define USB_Voltage     10
#define MOSI            11
#define SCK             12
#define MISO            13
#define FLASH_CS        15
#define Sensor_EN       16
#define mmWave_TX       17  // RX in ESP32
#define mmWave_RX       18  // TX in ESP32
#define Perip_EN        19
#define SD_CS           20
#define CE_En           47
#define Batt_EN         48

#endif