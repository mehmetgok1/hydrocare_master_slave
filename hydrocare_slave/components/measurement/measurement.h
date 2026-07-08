#ifndef MEASUREMENT_H
#define MEASUREMENT_H

#include <stdint.h>
#include "config.h"

//bme680 calls
void measureBME680(bme680_values_float_t* bme680_results);

//ov2640 camera calls
void get_ov2640_image(uint16_t* currentData);

//power led calls
void set_led_brightness(uint8_t brightness_pct);

//IR led calls
void set_ir_led(bool status);

//lis3dh calls
void measureLIS3DH(lis3dh_float_data_t* accel_results);

// IRTEMP camera calls
bool read_thermal_matrix_frame(float* mlx90641Frame, float* Tamb);

// adc measurement function
bool return_adc_result(uint16_t* ambientlight,uint16_t* microphone);



/*DEFINITIONS*/
//ov3660 definitions
#define CROP_SIZE 64
//adc mic and ambient light definitions
#define VREF            3.3f    // Max reference voltage 
#define R_LOAD          10000.0f // Load resistor value in Ohms (Adjust to match your schematic)
#endif // MEASUREMENT_H