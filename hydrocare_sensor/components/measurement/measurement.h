#ifndef MEASUREMENT_H
#define MEASUREMENT_H

#include <stdint.h>
#include "config.h"

//bme680 calls
bme680_values_float_t* measureBME680();

//ov3660 camera calls
void get_ov3660_image(uint16_t* currentData);

//power led calls
void set_led_brightness(uint8_t brightness_pct);

//IR led calls
void set_ir_led(bool status);

//lis3dh calls
lis3dh_float_data_t* measureLIS3DH();

// IRTEMP camera calls
bool read_thermal_matrix_frame(float* mlx90641Frame, float* Tamb);

// microphone measurement function
uint16_t* measureMicrophone(void);

// ambient light measurement function
uint16_t* measureAmbLight(void);



/*DEFINITIONS*/
//ov3660 definitions
#define CROP_SIZE 64
//adc mic and ambient light definitions
#define VREF            3.3f    // Max reference voltage 
#define R_LOAD          10000.0f // Load resistor value in Ohms (Adjust to match your schematic)
#endif // MEASUREMENT_H