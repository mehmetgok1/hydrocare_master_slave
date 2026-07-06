#include "config.h"
#include "MLX90641_API.h"
#include "esp_attr.h"
#include "lis3dh.h"
#include "lis3dh_types.h"

#define fw_version  "0.0.14"
// Static handles for the ADC driver and calibration to encapsulate them within this file.
static adc_oneshot_unit_handle_t adc1_handle;
static SemaphoreHandle_t adc_mutex = NULL;
static adc_cali_handle_t adc1_cali_handle_chan0 = NULL;
static adc_cali_handle_t adc1_cali_handle_chan1 = NULL;
static bool adc_cali_enabled_chan0 = false;
static bool adc_cali_enabled_chan1 = false;
// spi for slave to peripheral communication
spi_device_handle_t spi_bme_handle;
spi_device_handle_t spi_lis3dh_handle;
// I2C bus handle for MLX90614
i2c_master_bus_handle_t i2c0_bus_hdl = NULL;
//bme680 sensor handle
bme680_sensor_t *bme680_sensor = NULL; 
//lis3dh sensor handle
lis3dh_sensor_t *lis3dh_sensor = NULL; 
//IRTEMP camera handlers
//no need for getter in this lib implemenetation
// forward declerations
void initPins();
void init_adc_peripheral();
void init_spi_peripheral();
void initBME680();
void initCamera();
void powerLEDInit();
void initLIS3DH();
void initI2CBus();
void initIRTemp();

static paramsMLX90641 mlx90641_params;
static const char *TAG = "config";

void initPeripherals()
{
    initPins();
    init_adc_peripheral();
    initI2CBus(); // Initialize I2C bus before peripherals that use it
    init_spi_peripheral(); 
    initBME680();
    initCamera();
    powerLEDInit();
    initLIS3DH();
    initIRTemp();
    // Get available PSRAM size
    size_t available_PSRAM_size = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    ESP_LOGI(TAG, "PSRAM Size available (bytes): %d", available_PSRAM_size);
}
void initPins()
{
    // ===== Configure Output Pins =====
    gpio_config_t io_conf_output = {
        .pin_bit_mask = (1ULL << ledCntrlIR) | (1ULL << CAM_PWR) | (1ULL << ledCntrl) |
                        (1ULL << Acc_CS) | (1ULL << AQ_CS) | (1ULL << Perip_PWR),
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&io_conf_output);

    // ===== Configure Input Pins =====
    gpio_config_t io_conf_input = {
        .pin_bit_mask = (1ULL << SPI_CS),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE, // SPI CS lines should be pulled up
    };
    gpio_config(&io_conf_input);

    // ===== Set Initial Levels for Output Pins =====
    gpio_set_level(ledCntrlIR, 1);
    gpio_set_level(CAM_PWR, 1);
    gpio_set_level(ledCntrl, 1);
    gpio_set_level(Acc_CS, 1);
    gpio_set_level(AQ_CS, 1);
    gpio_set_level(Perip_PWR, 1);

    ESP_LOGI(TAG, "GPIO pins initialized using gpio_config()");
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
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_0, &config));
    // IA_Out (Microphone) is on GPIO2 -> ADC1_CH1
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_1, &config));
    // ===== Initialize ADC Calibration =====
    adc_cali_enabled_chan0 = adc_calibration_init(ADC_UNIT_1, ADC_CHANNEL_0, ADC_ATTEN_DB_12, &adc1_cali_handle_chan0);
    adc_cali_enabled_chan1 = adc_calibration_init(ADC_UNIT_1, ADC_CHANNEL_1, ADC_ATTEN_DB_12, &adc1_cali_handle_chan1);
}
void init_spi_peripheral()
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = MOSI_Perip, // Restored pin mappings
        .miso_io_num = MISO_Perip,
        .sclk_io_num = SCK_Perip,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO)); // Using SPI3_HOST (SPI host 3)
}

void initBME680()
{
    // Add the BME680 device to the SPI bus
    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .clock_speed_hz = 1 * 1000 * 1000, // 1 MHz
        .mode = 3, // BME680 uses SPI mode 3 (CPOL=1, CPHA=1)
        .spics_io_num = AQ_CS,
        .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &devcfg, &spi_bme_handle));
    bme680_sensor=bme680_init_sensor(1,  0, AQ_CS,&spi_bme_handle);
    bme680_set_oversampling_rates(bme680_sensor, osr_4x, osr_2x, osr_2x);
    bme680_set_filter_size(bme680_sensor, iir_size_7);
    bme680_set_heater_profile(bme680_sensor, 0, 320, 150);
    bme680_use_heater_profile (bme680_sensor, 0);
}
void initCamera()
{
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = CSI_D0;
  config.pin_d1 = CSI_D1;
  config.pin_d2 = CSI_D2;
  config.pin_d3 = CSI_D3;
  config.pin_d4 = CSI_D4;
  config.pin_d5 = CSI_D5;
  config.pin_d6 = CSI_D6;
  config.pin_d7 = CSI_D7;
  config.pin_xclk = CSI_MCLK;
  config.pin_pclk = CSI_PCLK;
  config.pin_vsync = CSI_VSYNC;
  config.pin_href = CSI_HSYNC;
  config.pin_sccb_sda = TWI_SDA;
  config.pin_sccb_scl = TWI_SCK;
  config.pin_pwdn = -1;
  config.pin_reset = -1;
  config.xclk_freq_hz = 16000000;
  config.pixel_format = PIXFORMAT_RGB565;
  config.frame_size = FRAMESIZE_96X96; // 320×240, we crop center 64×64
  config.jpeg_quality = 12;
  config.fb_count = 1;
  config.fb_location = CAMERA_FB_IN_DRAM;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    ESP_LOGI(TAG, "Camera init failed: 0x%02X", err);
  }
  else
  {
    ESP_LOGI(TAG, "Camera init successful");
  }
}
void powerLEDInit(){
      // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_CLK_SRC,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 1024, // Set duty to 100%. (2 ** 10) * 100% = 1024
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void initLIS3DH()
{
    // Add the LIS3DH device to the SPI bus
    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .clock_speed_hz = 10 * 1000 * 1000, // 400kHz
        .mode = 3, // LIS3DH uses SPI mode 3 (CPOL=1, CPHA=1)
        .spics_io_num = Acc_CS,
        .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &devcfg, &spi_lis3dh_handle));
    lis3dh_sensor=lis3dh_init_sensor(1,  0, Acc_CS,&spi_lis3dh_handle);
    lis3dh_config_hpf (lis3dh_sensor, lis3dh_hpf_normal, 0, false, false, false, false);    lis3dh_get_hpf_ref (lis3dh_sensor);
    // enable ADC inputs and temperature sensor for ADC input 3
    lis3dh_enable_adc (lis3dh_sensor, true, true);
    // LAST STEP: Finally set scale and mode to start measurements
    lis3dh_set_scale(lis3dh_sensor, lis3dh_scale_2_g);
    lis3dh_set_mode (lis3dh_sensor, lis3dh_odr_5000, lis3dh_low_power, true, true, true);

}
void initI2CBus(void) {
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,              // Maps to I2C0
        .scl_io_num = SCL,                   // Replace with your actual SCL GPIO pin number
        .sda_io_num = SDA,                   // Replace with your actual SDA GPIO pin number
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true, 
    };

    // Initialize the master bus
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c0_bus_hdl));
}

void initIRTemp()
{
    MLX90641_I2CInit();
    uint16_t* eeMLX90641 = malloc(832 * sizeof(uint16_t));
    if (eeMLX90641 == NULL) {
        ESP_LOGE("IR_TEMP", "Failed to allocate heap memory for EEPROM buffer!");
        return;
    }
    int error = MLX90641_DumpEE(0x33, eeMLX90641);
    if (error != 0) {
        ESP_LOGE("IR_TEMP", "Failed to dump EEPROM data!");
        free(eeMLX90641); // Clean up heap space on failure
        return;
    }
    error = MLX90641_ExtractParameters(eeMLX90641, &mlx90641_params);
    if (error != 0) {
        ESP_LOGE("IR_TEMP", "Parameter extraction failed.");
        free(eeMLX90641); // Clean up heap space on failure
        return;
    }
    // 1. Signed/Unsigned Integers
    free(eeMLX90641);
    MLX90641_SetRefreshRate(0x33, 0x06);
    int a= MLX90641_GetRefreshRate(0x33);
    ESP_LOGI("IR_TEMP", "MLX90641 Refresh Rate: %d Hz", (1<<(a-1)));
    ESP_LOGI("IR_TEMP", "Initialization successful. eeMLX90641 heap memory has been safely freed.");
}
/* GETTER FUNCTIONS FOR HANDLERS ETC. */
 
// This function passes a pointer to that static structure
paramsMLX90641* get_mlx90641_params(void)
{
    return &mlx90641_params;
}

lis3dh_sensor_t* get_lis3dh_dev_handle(void)
{
    return lis3dh_sensor;
}

bme680_sensor_t* get_bme_dev_handle(void)
{
    return bme680_sensor;
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
// Add these getters:
bool is_adc_cali_enabled_chan0(void)
{
    return adc_cali_enabled_chan0;
}

bool is_adc_cali_enabled_chan1(void)
{
    return adc_cali_enabled_chan1;
}
