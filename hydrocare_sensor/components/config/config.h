#ifndef CONFIG_H
#define CONFIG_H

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"
#include <stdbool.h>
#include "driver/spi_master.h"


void initPins();
void initPeripherals();
void init_spi_peripheral();
//void initIMU();
void initBME680();
uint8_t readbme680_register(uint8_t reg_addr);

// Getter functions for ADC handles and calibration status
adc_oneshot_unit_handle_t get_adc1_handle(void);
adc_cali_handle_t get_adc1_cali_handle_chan0(void);
adc_cali_handle_t get_adc1_cali_handle_chan1(void);
bool is_adc_cali_enabled_chan0(void);
bool is_adc_cali_enabled_chan1(void);

//spi_device_handle_t get_spi_imu_handle(void);

#define WHO_AM_I 0x0F
#define CTRL_REG1 0x20
#define CTRL_REG4 0x23


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