#ifndef MEASUREMENT_H
#define MEASUREMENT_H

#include "config.h"
#include <limits.h>

// Battery measurement constants
#define Batt_Meas_Count     10
#define Batt_Const_X        1
#define Batt_Const_Y        -0.1
#define Batt_VoltDiv_Mult   2
#define DigitalSupply       3.3
#define measurementTime     10000       //in ms

// ambient light sensor resistor value in ohms
#define R_LOAD  10000.0
#define VREF    3.3
//mmwave sensor defines
static const uint8_t DATA_FRAME_HEADER[4] = {0xF4, 0xF3, 0xF2, 0xF1};
static const uint8_t DATA_FRAME_TAIL[4]   = {0xF8, 0xF7, 0xF6, 0xF5};
#define BUF_SIZE 256


bool measureBatteryLevel(uint16_t* batteryLevelOut, float* batteryPercentageOut);
bool measureAmbLight(uint16_t* ambLight);
bool measurePIR(uint16_t* PIRValue);
void checkUSB(bool *chargingStatus);
bool checkButton();
void measuremmWave(uint16_t* movingDist, uint8_t* movingEnergy, uint16_t* staticDist, uint8_t* staticEnergy, uint16_t* detectionDist);

#endif