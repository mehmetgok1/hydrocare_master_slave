#include "leds.h"
#include "config.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

// PWM configuration for the power LED
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_10_BIT // 10-bit resolution (0-1023)
#define LEDC_FREQUENCY          (5000) // 5 kHz

void powerLED(uint16_t brightness){
  // Constrain brightness to 0-100%
  if (brightness > 100) {
    brightness = 100;
  }

  // Map 0-100% brightness to 10-bit PWM duty cycle.
  // The logic is inverted for a PMOS driver (0% brightness = max duty, 100% brightness = 0 duty).
  uint32_t max_duty = (1 << LEDC_DUTY_RES) - 1; // 1023 for 10-bit
  uint32_t duty_cycle = max_duty - (brightness * max_duty / 100);

  // Set duty and update the channel
  ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty_cycle);
  ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

void IRLED(bool status){
    // The control is inverted: status=true (1) means ON, which requires a LOW signal.
    if(status) {
        gpio_set_level(ledCntrlIR, 0); // Turn IR LED ON
    } else {
        gpio_set_level(ledCntrlIR, 1); // Turn IR LED OFF
    }
}

void powerLEDInit(){
  // 1. Configure the LEDC timer
  ledc_timer_config_t ledc_timer = {
      .speed_mode       = LEDC_MODE,
      .timer_num        = LEDC_TIMER,
      .duty_resolution  = LEDC_DUTY_RES,
      .freq_hz          = LEDC_FREQUENCY,
      .clk_cfg          = LEDC_AUTO_CLK
  };
  ledc_timer_config(&ledc_timer);

  // 2. Configure the LEDC channel
  ledc_channel_config_t ledc_channel = {
      .speed_mode = LEDC_MODE,
      .channel    = LEDC_CHANNEL,
      .timer_sel  = LEDC_TIMER,
      .intr_type  = LEDC_INTR_DISABLE,
      .gpio_num   = ledCntrl,
      .duty       = (1 << LEDC_DUTY_RES) -1, // Start with LED off (max duty for inverted logic)
      .hpoint     = 0
  };
  ledc_channel_config(&ledc_channel);
}