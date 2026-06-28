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
#include "esp_camera.h"


void initPins();
void initPeripherals();
void initIMU();
void initBME688();
void initIRTemp();
void initCamera();

// Getter functions for ADC handles and calibration status
adc_oneshot_unit_handle_t get_adc1_handle(void);
adc_cali_handle_t get_adc1_cali_handle_chan0(void);
adc_cali_handle_t get_adc1_cali_handle_chan1(void);
bool is_adc_cali_enabled_chan0(void);
bool is_adc_cali_enabled_chan1(void);

// Getter functions for peripheral handles
spi_device_handle_t get_spi_imu_handle(void);
spi_device_handle_t get_spi_bme_handle(void);
struct bme68x_dev *get_bme_dev_handle(void);


// Using static const for GPIO definitions provides type safety over #define.
static const gpio_num_t AmbLight    = GPIO_NUM_1;
static const gpio_num_t IA_Out      = GPIO_NUM_2;
static const gpio_num_t SCL         = GPIO_NUM_3;
static const gpio_num_t SDA         = GPIO_NUM_4;
static const gpio_num_t MISO_Perip  = GPIO_NUM_5;
static const gpio_num_t ledCntrlIR  = GPIO_NUM_19;
static const gpio_num_t SCK_Perip   = GPIO_NUM_21;
static const gpio_num_t MOSI_Perip  = GPIO_NUM_33;
static const gpio_num_t SPI_SCK     = GPIO_NUM_35;
static const gpio_num_t SPI_MOSI    = GPIO_NUM_36;
static const gpio_num_t SPI_MISOanalogReadResolution    = GPIO_NUM_37;
static const gpio_num_t SPI_CS      = GPIO_NUM_38;
static const gpio_num_t ledCntrl    = GPIO_NUM_40;
static const gpio_num_t Acc_CS      = GPIO_NUM_41;
static const gpio_num_t AQ_CS       = GPIO_NUM_42;
static const gpio_num_t Perip_PWR   = GPIO_NUM_45;

// Camera Interface Pins
static const gpio_num_t CSI_D0      = GPIO_NUM_15;
static const gpio_num_t CSI_D1      = GPIO_NUM_16;
static const gpio_num_t CSI_D2      = GPIO_NUM_17;
static const gpio_num_t CSI_D3      = GPIO_NUM_18;
static const gpio_num_t CSI_D4      = GPIO_NUM_8;
static const gpio_num_t CSI_D5      = GPIO_NUM_9;
static const gpio_num_t CSI_D6      = GPIO_NUM_10;
static const gpio_num_t CSI_D7      = GPIO_NUM_11;
static const gpio_num_t CSI_PCLK    = GPIO_NUM_13;
static const gpio_num_t CSI_MCLK    = GPIO_NUM_14;
static const gpio_num_t CAM_PWR     = GPIO_NUM_39;
static const gpio_num_t CSI_VSYNC   = GPIO_NUM_6;
static const gpio_num_t CSI_HSYNC   = GPIO_NUM_7;

// I2C (Two Wire Interface) Pins
static const gpio_num_t TWI_SDA     = GPIO_NUM_48;
static const gpio_num_t TWI_SCK     = GPIO_NUM_47;

#endif