#include "ui.h"

void setLED(uint8_t *red, uint8_t *green, uint8_t *blue){
    
    led_strip_set_pixel(get_led_strip_handle(), 0, *red, *green, *blue);
    led_strip_refresh(get_led_strip_handle());

}
void uiChargingScenario(uint8_t *red, uint8_t *green, uint8_t *blue){

    for(int j=0; j<5; j++){
        for(int i=0; i<=100; i++){
            *red = i;
            setLED(red, green, blue);
            vTaskDelay(pdMS_TO_TICKS(10));
            if(gpio_get_level((gpio_num_t)USB_Voltage) == 0){
                *red = 0;
                setLED(red, green, blue);
                break;
            }
        }
        for(int i=100; i>=0; i--){
            *red = i;
            setLED(red, green, blue);
            vTaskDelay(pdMS_TO_TICKS(10));
            if(gpio_get_level((gpio_num_t)USB_Voltage) == 0){
                *red = 0;
                setLED(red, green, blue);
                break;
            }
        }
    }
}
void uiOTAStarted(uint8_t *red, uint8_t *green, uint8_t *blue){
    *red = 5; *green = 5; *blue = 5; 
    setLED(red, green, blue);
}