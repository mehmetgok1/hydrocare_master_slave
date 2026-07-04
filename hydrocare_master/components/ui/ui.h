#ifndef LED_H
#define LED_H
#include "config.h"
#include "freertos/projdefs.h"

void uiChargingScenario(uint8_t *red, uint8_t *green, uint8_t *blue);
void uiOTAStarted(uint8_t *red, uint8_t *green, uint8_t *blue);

#endif