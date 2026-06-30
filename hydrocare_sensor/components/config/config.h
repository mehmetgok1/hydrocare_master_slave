#ifndef CONFIG_H
#define CONFIG_H

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"
#include <stdbool.h>
#include "driver/spi_master.h"
#include "bme680.h"
#include "bme680_types.h"
#include "esp_camera.h"
#include "driver/i2c_master.h"
#include "esp_heap_caps.h"
#include "driver/ledc.h"
//initialization functions
void initPeripherals();

/*GETTER FUNCTIONS BELOW*/

// Getter functions for ADC handles and calibration status
adc_oneshot_unit_handle_t get_adc1_handle(void);
adc_cali_handle_t get_adc1_cali_handle_chan0(void);
adc_cali_handle_t get_adc1_cali_handle_chan1(void);
bool is_adc_cali_enabled_chan0(void);
bool is_adc_cali_enabled_chan1(void);
//getter functions for SPI BME handles
bme680_sensor_t* get_bme_dev_handle(void);
//ov3660 camera getter functions
//no need for getter in this lib implemenetation
//Power led getter functions

/*USEFUL DEFINITIONS*/
// I2C address
#define WHO_AM_I 0x0F
#define CTRL_REG1 0x20
#define CTRL_REG4 0x23

//Power led PWM configuration
// PWM configuration for the power LED
#define LEDC_TIMER              LEDC_TIMER_1
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          ledCntrl // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_1
#define LEDC_DUTY_RES           LEDC_TIMER_10_BIT // Set duty resolution to 10 bits
#define LEDC_DUTY               (512) // Set duty to 50%. (2 ** 10) * 50% = 512
#define LEDC_CLK_SRC            LEDC_AUTO_CLK
#define LEDC_FREQUENCY          (4000) // Frequency in Hertz. Set frequency at 4 kHz

//GPIO pin definitions
#define AmbLight    1
#define IA_Out      2
#define SCL         3
#define SDA         4
#define MISO_Perip  5
#define ledCntrlIR  19
#define SCK_Perip   21
#define MOSI_Perip  33
#define SPI_SCK     35
#define SPI_MOSI    36
#define SPI_MISO    37
#define SPI_CS      38
#define ledCntrl    40
#define Acc_CS      41
#define AQ_CS       42
#define Perip_PWR   45

#define CSI_D0      15
#define CSI_D1      16
#define CSI_D2      17
#define CSI_D3      18
#define CSI_D4      8
#define CSI_D5      9
#define CSI_D6      10
#define CSI_D7      11
#define CSI_PCLK    13
#define CSI_MCLK    14
#define CAM_PWR     39
#define CSI_VSYNC   6
#define CSI_HSYNC   7
#define TWI_SDA     48
#define TWI_SCK     47

#endif