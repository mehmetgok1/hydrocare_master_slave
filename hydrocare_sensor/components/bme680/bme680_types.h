#ifndef __BME680_TYPES_H__
#define __BME680_TYPES_H__

#include "stdint.h"
#include "stdbool.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct {                                              // invalid value
    int16_t  temperature;    // temperature in degree C * 100 (INT16_MIN)
    uint32_t pressure;       // barometric pressure in Pascal (0)
    uint32_t humidity;       // relative humidity in % * 1000 (0)
    uint32_t gas_resistance; // gas resistance in Ohm         (0)
} bme680_values_fixed_t;
typedef struct {                                              // invalid value
    float   temperature;    // temperature in degree C        (-327.68)
    float   pressure;       // barometric pressure in hPascal (0.0)
    float   humidity;       // relative humidity in %         (0.0)
    float   gas_resistance; // gas resistance in Ohm          (0.0)
} bme680_values_float_t;
typedef enum {
    osr_none = 0,     // measurement is skipped, output values are invalid
    osr_1x   = 1,     // default oversampling rates
    osr_2x   = 2,
    osr_4x   = 3,
    osr_8x   = 4,
    osr_16x  = 5
} bme680_oversampling_rate_t;
typedef enum {
    iir_size_0   = 0,   // filter is not used
    iir_size_1   = 1,
    iir_size_3   = 2,
    iir_size_7   = 3,
    iir_size_15  = 4,
    iir_size_31  = 5,
    iir_size_63  = 6,
    iir_size_127 = 7
} bme680_filter_size_t;
typedef struct {

    uint8_t  osr_temperature;        // T oversampling rate (default osr_1x)
    uint8_t  osr_pressure;           // P oversampling rate (default osr_1x)
    uint8_t  osr_humidity;           // H oversampling rate (default osr_1x)
    uint8_t  filter_size;            // IIR filter size (default iir_size_3)

    int8_t   heater_profile;         // Heater profile used (default 0)
    uint16_t heater_temperature[10]; // Heater temperature for G (default 320)
    uint16_t heater_duration[10];    // Heater duration for G (default 150)

    int8_t   ambient_temperature;    // Ambient temperature for G (default 25)

} bme680_settings_t;
typedef struct {

    uint16_t par_t1;         // calibration data for temperature compensation
    int16_t  par_t2;
    int8_t   par_t3;

    uint16_t par_p1;         // calibration data for pressure compensation
    int16_t  par_p2;
    int8_t   par_p3;
    int16_t  par_p4;
    int16_t  par_p5;
    int8_t   par_p7;
    int8_t   par_p6;
    int16_t  par_p8;
    int16_t  par_p9;
    uint8_t  par_p10;

    uint16_t par_h1;         // calibration data for humidity compensation
    uint16_t par_h2;
    int8_t   par_h3;
    int8_t   par_h4;
    int8_t   par_h5;
    uint8_t  par_h6;
    int8_t   par_h7;

    int8_t   par_gh1;        // calibration data for gas compensation
    int16_t  par_gh2;
    int8_t   par_gh3;

    int32_t  t_fine;         // temperatur correction factor for P and G
    uint8_t  res_heat_range;
    int8_t   res_heat_val;
    int8_t   range_sw_err;

} bme680_calib_data_t;
typedef struct {

    int       error_code;      // contains the error code of last operation

    uint8_t   bus;             // I2C = x, SPI = 1
    uint8_t   addr;            // I2C = slave address, SPI = 0
    uint8_t   cs;              // ESP8266, ESP32: GPIO used as SPI CS
                               // __linux__: device index

    bool      meas_started;    // indicates whether measurement started
    uint8_t   meas_status;     // last sensor status (for internal use only)

    bme680_settings_t    settings;   // sensor settings
    bme680_calib_data_t  calib_data; // calibration data of the sensor

} bme680_sensor_t;
#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* __BME680_TYPES_H__ */
