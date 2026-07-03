#include "config.h"


static const char *TAG = "CONFIG";
#define EXAMPLE_MAX_CHAR_SIZE    64

//handles 
static led_strip_handle_t main_led_strip = NULL;

//forward declearation of functions
void initPins();
led_strip_handle_t init_led_strip();
void initPeripherals(){
  // This function should be called first from app_main to set up hardware.
  initPins();
  init_led_strip();
  // NOTE: SPI bus initialization is now handled by the specific drivers
  // that use it (e.g., `initSPIComm()` and `initSD()`). We do not initialize it here.

  // Configure ADC1 Channel 0 (GPIO 1) for the Ambient Light sensor.
  //ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_12));
  //ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11));

  ESP_LOGI(TAG, "Core peripherals initialized.");
}

void initPins(){
  // ===== Configure Output Pins using ESP-IDF native API =====
  gpio_config_t io_conf_output = {
      .pin_bit_mask = (1ULL << CE_En) | (1ULL << Batt_EN) | (1ULL << FLASH_CS) |
                      (1ULL << Sensor_EN) | (1ULL << Perip_EN) | (1ULL << SD_CS) |
                      (1ULL << SPI_CS),
      .mode = GPIO_MODE_OUTPUT,
      .intr_type = GPIO_INTR_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .pull_up_en = GPIO_PULLUP_DISABLE,
  };
  gpio_config(&io_conf_output);

  // ===== Configure Input Pins using ESP-IDF native API =====
  gpio_config_t io_conf_input = {
      .pin_bit_mask = (1ULL << PIR) | (1ULL << mmWave_Out) | (1ULL << USB_Voltage),
      .mode = GPIO_MODE_INPUT,
      .intr_type = GPIO_INTR_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .pull_up_en = GPIO_PULLUP_DISABLE, // These inputs are likely driven externally
  };
  gpio_config(&io_conf_input);

  // Configure Button with an internal pull-up
  gpio_config_t io_conf_button = {
      .pin_bit_mask = (1ULL << Button),
      .mode = GPIO_MODE_INPUT,
      .intr_type = GPIO_INTR_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .pull_up_en = GPIO_PULLUP_ENABLE,
  };
  gpio_config(&io_conf_button);
  // ===== Set Initial Levels for Output Pins =====
  gpio_set_level((gpio_num_t)CE_En, 0);
  gpio_set_level((gpio_num_t)Batt_EN, 0);
  gpio_set_level((gpio_num_t)FLASH_CS, 1);
  gpio_set_level((gpio_num_t)Sensor_EN, 1);
  gpio_set_level((gpio_num_t)Perip_EN, 1);
  gpio_set_level((gpio_num_t)SD_CS, 1);
  gpio_set_level((gpio_num_t)SPI_CS, 1);
  ESP_LOGI(TAG, "GPIO pins initialized using native gpio_config()");
}
led_strip_handle_t init_led_strip(void)
{    // LED strip general initialization, according to your led board design
    led_strip_config_t strip_config = {
        .strip_gpio_num = Neopixel, // The GPIO that connected to the LED strip's data line
        .max_leds = NUMPIXELS,      // The number of LEDs in the strip,
        .led_model = LED_MODEL_WS2812,        // LED strip model
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB, // The color order of the strip: GRB
        .flags = {
            .invert_out = false, // don't invert the output signal
        }
    };

    // LED strip backend configuration: RMT
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,        // different clock source can lead to different power consumption
        .resolution_hz = 10 * 1000 * 1000, // RMT counter clock frequency
        .flags = {
            .with_dma = false,     // Using DMA can improve performance when driving more LEDs
        }
    };
    // LED Strip object handle
    led_strip_handle_t led_strip;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    led_strip_clear(main_led_strip);
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI(TAG, "Created LED strip object with RMT backend");
    return led_strip;
}

// getter function for handler
led_strip_handle_t get_led_strip_handle(void)
{
    return main_led_strip;
}