#ifndef __LIS3DH_H__
#define __LIS3DH_H__


#include "lis3dh_types.h"

// LIS3DH addresses (also used for LIS2DH, LIS2DH12 and LIS2DE12)
#define LIS3DH_I2C_ADDRESS_1           0x18  // SDO pin is low
#define LIS3DH_I2C_ADDRESS_2           0x19  // SDO pin is high

// LIS3DE addresse (also used for LIS2DE)
#define LIS3DE_I2C_ADDRESS_1           0x28  // SDO pin is low
#define LIS3DE_I2C_ADDRESS_2           0x29  // SDO pin is high

// LIS3DH chip id
#define LIS3DH_CHIP_ID                 0x33  // LIS3DH_REG_WHO_AM_I<7:0>

// Definition of error codes
#define LIS3DH_OK                      0
#define LIS3DH_NOK                     -1

#define LIS3DH_INT_ERROR_MASK          0x000f
#define LIS3DH_DRV_ERROR_MASK          0xfff0

// Error codes for I2C and SPI interfaces ORed with LIS3DH driver error codes
#define LIS3DH_I2C_READ_FAILED         1
#define LIS3DH_I2C_WRITE_FAILED        2
#define LIS3DH_I2C_BUSY                3
#define LIS3DH_SPI_WRITE_FAILED        4
#define LIS3DH_SPI_READ_FAILED         5
#define LIS3DH_SPI_BUFFER_OVERFLOW     6

// LIS3DH driver error codes ORed with error codes for I2C and SPI interfaces
#define LIS3DH_WRONG_CHIP_ID              ( 1 << 8)
#define LIS3DH_WRONG_BANDWIDTH            ( 2 << 8)
#define LIS3DH_GET_RAW_DATA_FAILED        ( 3 << 8)
#define LIS3DH_GET_RAW_DATA_FIFO_FAILED   ( 4 << 8)
#define LIS3DH_WRONG_INT_TYPE             ( 5 << 8)
#define LIS3DH_CONFIG_INT_SIGNALS_FAILED  ( 6 << 8)
#define LIS3DH_CONFIG_INT_FAILED          ( 7 << 8)
#define LIS3DH_INT_SOURCE_FAILED          ( 8 << 8)
#define LIS3DH_CONFIG_HPF_FAILED          ( 9 << 8)
#define LIS3DH_ENABLE_HPF_FAILED          (10 << 8)
#define LIS3DH_CONFIG_CLICK_FAILED        (11 << 8)
#define LIS3DH_CLICK_SOURCE_FAILED        (12 << 8)
#define LIS3DH_GET_ADC_DATA_FAILED        (13 << 8)
#define LIS3DH_SENSOR_IN_BYPASS_MODE      (14 << 8)
#define LIS3DH_SENSOR_IN_FIFO_MODE        (15 << 8)
#define LIS3DH_ODR_TOO_HIGH               (16 << 8)


#define LIS3DH_ANY_DATA_READY    0x0f    // LIS3DH_REG_STATUS<3:0>
// register addresses
#define LIS3DH_REG_STATUS_AUX    0x07
#define LIS3DH_REG_OUT_ADC1_L    0x08
#define LIS3DH_REG_OUT_ADC1_H    0x09
#define LIS3DH_REG_OUT_ADC2_L    0x0a
#define LIS3DH_REG_OUT_ADC2_H    0x0b
#define LIS3DH_REG_OUT_ADC3_L    0x0c
#define LIS3DH_REG_OUT_ADC3_H    0x0d
#define LIS3DH_REG_INT_COUNTER   0x0e
#define LIS3DH_REG_WHO_AM_I      0x0f
#define LIS3DH_REG_TEMP_CFG      0x1f
#define LIS3DH_REG_CTRL1         0x20
#define LIS3DH_REG_CTRL2         0x21
#define LIS3DH_REG_CTRL3         0x22
#define LIS3DH_REG_CTRL4         0x23
#define LIS3DH_REG_CTRL5         0x24
#define LIS3DH_REG_CTRL6         0x25
#define LIS3DH_REG_REFERENCE     0x26
#define LIS3DH_REG_STATUS        0x27
#define LIS3DH_REG_OUT_X_L       0x28
#define LIS3DH_REG_OUT_X_H       0x29
#define LIS3DH_REG_OUT_Y_L       0x2a
#define LIS3DH_REG_OUT_Y_H       0x2b
#define LIS3DH_REG_OUT_Z_L       0x2c
#define LIS3DH_REG_OUT_Z_H       0x2d
#define LIS3DH_REG_FIFO_CTRL     0x2e
#define LIS3DH_REG_FIFO_SRC      0x2f
#define LIS3DH_REG_INT1_CFG      0x30
#define LIS3DH_REG_INT1_SRC      0x31
#define LIS3DH_REG_INT1_THS      0x32
#define LIS3DH_REG_INT1_DUR      0x33
#define LIS3DH_REG_INT2_CFG      0x34
#define LIS3DH_REG_INT2_SRC      0x35
#define LIS3DH_REG_INT2_THS      0x36
#define LIS3DH_REG_INT2_DUR      0x37
#define LIS3DH_REG_CLICK_CFG     0x38
#define LIS3DH_REG_CLICK_SRC     0x39
#define LIS3DH_REG_CLICK_THS     0x3a
#define LIS3DH_REG_TIME_LIMIT    0x3b
#define LIS3DH_REG_TIME_LATENCY  0x3c
#define LIS3DH_REG_TIME_WINDOW   0x3d

#ifdef __cplusplus
extern "C"
{
#endif

lis3dh_sensor_t* lis3dh_init_sensor (uint8_t bus, uint8_t addr, uint8_t cs);
bool lis3dh_set_mode (lis3dh_sensor_t* dev, 
                      lis3dh_odr_mode_t odr, lis3dh_resolution_t res,
                      bool x, bool y, bool z);
                       
bool lis3dh_set_scale (lis3dh_sensor_t* dev, lis3dh_scale_t scale);
bool lis3dh_set_fifo_mode (lis3dh_sensor_t* dev, lis3dh_fifo_mode_t mode, 
                           uint8_t thresh, lis3dh_int_signal_t trigger);
bool lis3dh_new_data (lis3dh_sensor_t* dev);

bool lis3dh_get_float_data (lis3dh_sensor_t* dev,
                            lis3dh_float_data_t* data);

uint8_t lis3dh_get_float_data_fifo (lis3dh_sensor_t* dev,
                                    lis3dh_float_data_fifo_t data);

bool lis3dh_get_raw_data (lis3dh_sensor_t* dev, lis3dh_raw_data_t* raw);

uint8_t lis3dh_get_raw_data_fifo (lis3dh_sensor_t* dev,
                                  lis3dh_raw_data_fifo_t raw);
                                   
bool lis3dh_enable_int (lis3dh_sensor_t* dev, 
                        lis3dh_int_type_t type, 
                        lis3dh_int_signal_t signal, bool value);
                                   
bool lis3dh_get_int_data_source (lis3dh_sensor_t* dev, 
                                 lis3dh_int_data_source_t* source);

bool lis3dh_set_int_event_config (lis3dh_sensor_t* dev,
                                  lis3dh_int_event_config_t* config,
                                  lis3dh_int_event_gen_t gen);

bool lis3dh_get_int_event_config (lis3dh_sensor_t* dev,
                                  lis3dh_int_event_config_t* config,
                                  lis3dh_int_event_gen_t gen);
bool lis3dh_get_int_event_source (lis3dh_sensor_t* dev,
                                  lis3dh_int_event_source_t* source,
                                  lis3dh_int_event_gen_t gen);
bool lis3dh_set_int_click_config (lis3dh_sensor_t* dev,
                                  lis3dh_int_click_config_t* config);
bool lis3dh_get_int_click_config (lis3dh_sensor_t* dev,
                                  lis3dh_int_click_config_t* config);
bool lis3dh_get_int_click_source (lis3dh_sensor_t* dev, 
                                  lis3dh_int_click_source_t* source);                          
bool lis3dh_config_int_signals (lis3dh_sensor_t* dev,
                                lis3dh_int_signal_level_t level);
bool lis3dh_config_hpf (lis3dh_sensor_t* dev, 
                        lis3dh_hpf_mode_t mode,  uint8_t cutoff,
                        bool data, bool click, bool int1, bool int2);
bool lis3dh_set_hpf_ref (lis3dh_sensor_t* dev, int8_t ref);
int8_t lis3dh_get_hpf_ref (lis3dh_sensor_t* dev);
int8_t lis3dh_enable_adc (lis3dh_sensor_t* dev, bool enable, bool temp);
bool lis3dh_get_adc (lis3dh_sensor_t* dev,
                     uint16_t* adc1, uint16_t* adc2, uint16_t* adc3);
bool lis3dh_reg_write (lis3dh_sensor_t* dev, 
                       uint8_t reg, uint8_t *data, uint16_t len);
bool lis3dh_reg_read (lis3dh_sensor_t* dev, 
                      uint8_t reg, uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* __LIS3DH_H__ */
