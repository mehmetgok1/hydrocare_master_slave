
#ifndef __BME680_H__
#define __BME680_H__

// Uncomment one of the following defines to enable debug output
// #define BME680_DEBUG_LEVEL_1    // only error messages
// #define BME680_DEBUG_LEVEL_2    // debug and error messages

#include <string.h>
#include <stdlib.h>
#include "driver/spi_master.h"
#include "bme680_types.h"
// BME680 addresses
#define BME680_I2C_ADDRESS_1           0x76  // SDO pin is low
#define BME680_I2C_ADDRESS_2           0x77  // SDO pin is high

// BME680 chip id
#define BME680_CHIP_ID                 0x61    // BME680_REG_ID<7:0>

// Definition of error codes
#define BME680_OK                      0
#define BME680_NOK                     -1

#define BME680_INT_ERROR_MASK          0x000f
#define BME680_DRV_ERROR_MASK          0xfff0

// Error codes for I2C and SPI interfaces ORed with BME680 driver error codes
#define BME680_I2C_READ_FAILED         1
#define BME680_I2C_WRITE_FAILED        2
#define BME680_I2C_BUSY                3
#define BME680_SPI_WRITE_FAILED        4
#define BME680_SPI_READ_FAILED         5
#define BME680_SPI_BUFFER_OVERFLOW     6
#define BME680_SPI_SET_PAGE_FAILED     7

// BME680 driver error codes ORed with error codes for I2C and SPI interfaces
#define BME680_RESET_CMD_FAILED        ( 1 << 8)
#define BME680_WRONG_CHIP_ID           ( 2 << 8)
#define BME680_READ_CALIB_DATA_FAILED  ( 3 << 8)
#define BME680_MEAS_ALREADY_RUNNING    ( 4 << 8)
#define BME680_MEAS_NOT_RUNNING        ( 5 << 8)
#define BME680_MEAS_STILL_RUNNING      ( 6 << 8)
#define BME680_FORCE_MODE_FAILED       ( 7 << 8)
#define BME680_NO_NEW_DATA             ( 8 << 8)
#define BME680_WRONG_HEAT_PROFILE      ( 9 << 8)
#define BME680_MEAS_GAS_NOT_VALID      (10 << 8)
#define BME680_HEATER_NOT_STABLE       (11 << 8)

// Driver range definitions
#define BME680_HEATER_TEMP_MIN         200  // min. 200 degree Celsius
#define BME680_HEATER_TEMP_MAX         400  // max. 200 degree Celsius
#define BME680_HEATER_PROFILES         10   // max. 10 heater profiles 0 ... 9
#define BME680_HEATER_NOT_USED         -1   // heater not used profile

#ifdef __cplusplus
extern "C"
{
#endif

bme680_sensor_t* bme680_init_sensor (uint8_t bus, uint8_t addr, uint8_t cs,spi_device_handle_t* spi_bme_handle);

bool bme680_force_measurement (bme680_sensor_t* dev);

uint32_t bme680_get_measurement_duration (const bme680_sensor_t *dev);
bool bme680_is_measuring (bme680_sensor_t* dev);
bool bme680_get_results_fixed (bme680_sensor_t* dev,
                               bme680_values_fixed_t* results);
bool bme680_get_results_float (bme680_sensor_t* dev,
                               bme680_values_float_t* results);
bool bme680_measure_fixed (bme680_sensor_t* dev,
                           bme680_values_fixed_t* results);
bool bme680_measure_float (bme680_sensor_t* dev,
                           bme680_values_float_t* results);
bool bme680_set_oversampling_rates (bme680_sensor_t* dev,
                                    bme680_oversampling_rate_t osr_t,
                                    bme680_oversampling_rate_t osr_p,
                                    bme680_oversampling_rate_t osr_h);
bool bme680_set_filter_size(bme680_sensor_t* dev, bme680_filter_size_t size);
bool bme680_set_heater_profile (bme680_sensor_t* dev,
                                uint8_t  profile,
                                uint16_t temperature,
                                uint16_t duration);
bool bme680_use_heater_profile (bme680_sensor_t* dev, int8_t profile);
bool bme680_set_ambient_temperature (bme680_sensor_t* dev,
                                     int16_t temperature);

#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* __BME680_H__ */
