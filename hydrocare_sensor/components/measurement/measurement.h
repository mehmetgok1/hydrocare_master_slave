#ifndef MEASUREMENT_H
#define MEASUREMENT_H

#include <stdint.h>
#include "driver/spi_master.h"
#include "driver/i2c.h"

// Global variables for sensor readings, accessible by other components
extern uint16_t ambLight;
extern uint16_t microphone;
extern float ax, ay, az;
extern float bme_temp;
extern float bme_hum;
extern float bme_pres;
extern float bme_gas;

// Forward declaration for the MLX90641 C-style handle
typedef struct MLX90641_handle_t MLX90641_handle_t;
extern MLX90641_handle_t myIRcam;

// Initialization functions
void initIMU();
void initIRTemp();
void initBME688();
void initCamera();

// Measurement functions
void readAcceleration();
void measureAmbLight();
void measureMicrophone();
void measureIRTemp();
void measureBME688();

#endif // MEASUREMENT_H