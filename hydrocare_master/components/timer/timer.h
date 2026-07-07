#ifndef TIMER_HANDLER_H
#define TIMER_HANDLER_H

#include "config.h"
#include "freertos/FreeRTOS.h"
#include "driver/gptimer.h"
#include "esp_attr.h"


#define TIMER_RESOLUTION_HZ 1000000 
#define TIMER_ALARM_US      200000 

void initTimer(volatile bool *target_flag);
void setTimer();
void disableTimer();

#endif // TIMER_HANDLER_H