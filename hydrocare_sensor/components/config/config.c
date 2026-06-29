#include "config.h"
#include "esp_heap_caps.h"
#define fw_version  "0.0.14"
// Static handles for the ADC driver and calibration to encapsulate them within this file.
static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t adc1_cali_handle_chan0 = NULL;
static adc_cali_handle_t adc1_cali_handle_chan1 = NULL;
static bool adc_cali_enabled_chan0 = false;
static bool adc_cali_enabled_chan1 = false;
uint8_t readbme680_register(uint8_t reg_addr);
// Peripheral handles are now static to this file
spi_device_handle_t spi_bme_handle;
bme680_sensor_t *bme680_sensor = NULL; 
bme680_values_float_t results;
static const char *TAG = "config";

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

void initPeripherals()
{
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

    // ===== Initialize Other Peripherals =====
    init_spi_peripheral(); 
    initBME680();

    if (bme680_sensor == NULL) {
        ESP_LOGE(TAG, "❌ BME680 Initialization failed! Handle allocation failed.");
        return; 
    }

    uint8_t chip_id; 
    bme680_read_reg(bme680_sensor, BME680_REG_ID, &chip_id, 1);
    ESP_LOGI(TAG, "BME680 Chip ID: 0x%02X", chip_id);

    while (1) {
    uint32_t duration = bme680_get_measurement_duration(bme680_sensor);
    
    if (bme680_force_measurement(bme680_sensor)) {
        vTaskDelay(duration);
        
        if (bme680_get_results_float(bme680_sensor, &results)) {
            ESP_LOGI(TAG, "BME680 Sensor: %.2f °C, %.2f %%, %.2f hPa, %.2f Ohm",
                results.temperature, 
                results.humidity, 
                results.pressure, 
                results.gas_resistance);
        }
    }
    
    // Wait a couple of seconds before the next reading
    vTaskDelay(pdMS_TO_TICKS(2000));
    }
    //initIRTemp();
    //initCamera();

    // Get available PSRAM size
    size_t available_PSRAM_size = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    ESP_LOGI(TAG, "PSRAM Size available (bytes): %d", available_PSRAM_size);
}



void init_spi_peripheral()
{
    // Initialize the SPI bus for peripherals
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
    // Change the IIR filter size for temperature and pressure to 7.
    bme680_set_filter_size(bme680_sensor, iir_size_7);
    // Change the heater profile 0 to 200 degree Celcius for 100 ms.
    bme680_set_heater_profile(bme680_sensor, 0, 320, 150);
    bme680_use_heater_profile (bme680_sensor, 0);
}
uint8_t readbme680_register(uint8_t reg_addr)
{
    esp_err_t ret;
    uint8_t tx_data[2];
    uint8_t rx_data[2];

    // For BME680 SPI read, the MSB of the register address must be 1.
    tx_data[0] = reg_addr | 0x80;
    tx_data[1] = 0x00; // Dummy byte to clock out the data

    spi_transaction_t t = {
        .length = 16, // 2 bytes * 8 bits
        .tx_buffer = &tx_data,
        .rx_buffer = &rx_data
    };

    ret = spi_device_polling_transmit(spi_bme_handle, &t);
    ESP_ERROR_CHECK(ret);

    // The actual data is in the second byte received.
    return rx_data[1];
}
adc_oneshot_unit_handle_t get_adc1_handle(void)
{
    return adc1_handle;
}

adc_cali_handle_t get_adc1_cali_handle_chan0(void)
{
    return adc1_cali_handle_chan0;
}

adc_cali_handle_t get_adc1_cali_handle_chan1(void)
{
    return adc1_cali_handle_chan1;
}
