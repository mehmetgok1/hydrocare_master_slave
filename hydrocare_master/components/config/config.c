#include "config.h"


static const char *TAG = "CONFIG";
#define EXAMPLE_MAX_CHAR_SIZE    64


// Static handles for the ADC driver and calibration to encapsulate them within this file.
static adc_oneshot_unit_handle_t adc1_handle;
static SemaphoreHandle_t adc_mutex = NULL;
static adc_cali_handle_t adc1_cali_handle_chan0 = NULL;
static adc_cali_handle_t adc1_cali_handle_chan1 = NULL;
static adc_cali_handle_t adc1_cali_handle_chan2 = NULL;
static bool adc_cali_enabled_chan0 = false;
static bool adc_cali_enabled_chan1 = false;
static bool adc_cali_enabled_chan2 = false;

//led handle 
led_strip_handle_t main_led_strip = NULL;

//forward declearation of functions
void initPins();
void init_mmWave();
void init_adc_peripheral();
void init_led_strip();

void initPeripherals(){
  // This function should be called first from app_main to set up hardware.
  initPins();
  init_led_strip();
  init_adc_peripheral();
  init_mmWave();
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
void init_led_strip(void)
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
    // Set the first pixel to a dim green to indicate successful initialization
    led_strip_set_pixel(main_led_strip, 0, 0, 5, 0);
    led_strip_refresh(main_led_strip);
    //set finish
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI(TAG, "Created LED strip object with RMT backend");
}

static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
    *out_handle = handle;
    return calibrated;
}
void init_adc_peripheral()
{
    adc_mutex = xSemaphoreCreateMutex();
    if (!adc_mutex) {
        ESP_LOGE(TAG, "Failed to create ADC mutex");
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // ===== Initialize ADC1 for One-Shot Mode =====
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    // ===== Configure ADC Channels =====
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };
    // AmbLight is on GPIO1 -> ADC1_CH0
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_0, &config)); // AmbLight on GPIO1
    // battlevel is on GPIO2 -> ADC1_CH1
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_1, &config));
    // PIR  is on GPIO3 -> ADC1_CH2
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_2, &config));
    // ===== Initialize ADC Calibration =====
    adc_cali_enabled_chan0 = adc_calibration_init(ADC_UNIT_1, ADC_CHANNEL_0, ADC_ATTEN_DB_12, &adc1_cali_handle_chan0);
    adc_cali_enabled_chan1 = adc_calibration_init(ADC_UNIT_1, ADC_CHANNEL_1, ADC_ATTEN_DB_12, &adc1_cali_handle_chan1);
    adc_cali_enabled_chan2 = adc_calibration_init(ADC_UNIT_1, ADC_CHANNEL_2, ADC_ATTEN_DB_12, &adc1_cali_handle_chan2);
}

void init_mmWave() {
    // Set Sensor_EN pin to LOW to enable the sensor
    gpio_set_level((gpio_num_t)Sensor_EN, 0);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Configure UART parameters
    uart_config_t uart_config = {
        .baud_rate = 256000, // Assuming a standard baud rate, replace with RADAR_BAUD if defined
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    // Install UART driver, and get the queue.
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, 256 * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, mmWave_TX, mmWave_RX, -1, -1));
    vTaskDelay(pdMS_TO_TICKS(100));
}



// getter function for handler
led_strip_handle_t get_led_strip_handle(void)
{
    return main_led_strip;
}

adc_oneshot_unit_handle_t get_adc1_handle(void)
{
    return adc1_handle;
}

SemaphoreHandle_t get_adc_mutex(void)
{
    return adc_mutex;
}

adc_cali_handle_t get_adc1_cali_handle_chan0(void)
{
    return adc1_cali_handle_chan0;
}

adc_cali_handle_t get_adc1_cali_handle_chan1(void)
{
    return adc1_cali_handle_chan1;
}

adc_cali_handle_t get_adc1_cali_handle_chan2(void)
{
    return adc1_cali_handle_chan2;
}

bool is_adc_cali_enabled_chan0(void)
{
    return adc_cali_enabled_chan0;
}

bool is_adc_cali_enabled_chan1(void)
{
    return adc_cali_enabled_chan1;
}

bool is_adc_cali_enabled_chan2(void)
{
    return adc_cali_enabled_chan2;
}