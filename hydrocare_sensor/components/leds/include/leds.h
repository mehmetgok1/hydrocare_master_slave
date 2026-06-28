#ifndef LEDS_H
#define LEDS_H

#include <stdint.h>
#include <stdbool.h>

void powerLEDInit();

void powerLED(uint16_t brightness);

void IRLED(bool status);

#endif // LEDS_H