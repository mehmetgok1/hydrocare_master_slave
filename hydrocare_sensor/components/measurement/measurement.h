#ifndef MEASUREMENT_H
#define MEASUREMENT_H

#include <stdint.h>
#include "config.h"


// Measurement functions
//void readAcceleration();
//void measureAmbLight();
//void measureMicrophone();
//void measureIRTemp();

//bme680 calls
void read_bme680_chip_id();
bme680_values_float_t* measureBME680();

//ov3660 camera calls
void get_ov3660_image(uint16_t* currentData);

//power led calls
void set_led_brightness(uint8_t brightness_pct);
//IR led calls
void set_ir_led(bool status);



/*DEFINITIONS*/
//ov3660 definitions
#define CROP_SIZE 64

#endif // MEASUREMENT_H