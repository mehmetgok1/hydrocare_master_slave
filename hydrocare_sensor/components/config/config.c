#include "config.h"
#include "esp_heap_caps.h"
#include "driver/i2c_master.h"
#include "esp_rom_sys.h" // For esp_rom_delay_us

#define fw_version  "0.0.14"
// Static handles for the ADC driver and calibration to encapsulate them within this file.
static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t adc1_cali_handle_chan0 = NULL;
static adc_cali_handle_t adc1_cali_handle_chan1 = NULL;
static bool adc_cali_enabled_chan0 = false;
static bool adc_cali_enabled_chan1 = false;

// Peripheral handles are now static to this file
static spi_device_handle_t spi_imu_handle;
static spi_device_handle_t spi_bme_handle;
static struct bme68x_dev bme_dev;

#define WHO_AM_I 0x0F
#define CTRL_REG1 0x20
#define CTRL_REG4 0x23

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

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
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
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

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
  initIMU();
  initBME688();
  initIRTemp();
  initCamera();

  // Get available PSRAM size
  size_t available_PSRAM_size = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
  ESP_LOGI(TAG, "PSRAM Size available (bytes): %d", available_PSRAM_size);
}

// Helper function to write to a register via SPI
static void writeRegister(spi_device_handle_t spi, uint8_t reg, uint8_t value)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.flags = SPI_TRANS_USE_TXDATA;
    t.cmd = reg & 0x3F; // Write command
    t.tx_data[0] = value;
    t.length = 8; // 8 bits of data
    spi_device_polling_transmit(spi, &t);
}
// Helper function to read from a register via SPI
static uint8_t readRegister(spi_device_handle_t spi, uint8_t reg)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.flags = SPI_TRANS_USE_RXDATA;
    t.cmd = 0x80 | reg; // Read command
    t.length = 8;
    spi_device_polling_transmit(spi, &t);
    return t.rx_data[0];
}

void initIRTemp()
{
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .sda_io_num = SDA,
        .scl_io_num = SCL,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
    ESP_LOGI(TAG, "I2C Master Bus for IR Temp Initialized");

    // The rest of this function depends on your C-based MLX90641 driver.
    // You would now pass the bus_handle to your device initialization function,
    // which would then create a device handle with i2c_master_bus_add_device().

    // if (mlx90641_init(&myIRcam, I2C_NUM_0) != ESP_OK) {
    //     ESP_LOGE(TAG, "MLX90641 init failed!");
    // } else {
    //     ESP_LOGI(TAG, "MLX90641 Initialized");
    // }
}

void initIMU()
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = MOSI_Perip,
        .miso_io_num = MISO_Perip,
        .sclk_io_num = SCK_Perip,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    // Initialize SPI3 host
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {
        .command_bits = 8,
        .clock_speed_hz = 1 * 1000 * 1000, // 1 MHz
        .mode = 3, // SPI mode 3 for LIS3DH
        .spics_io_num = Acc_CS,
        .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &devcfg, &spi_imu_handle));

    esp_rom_delay_us(100);

    uint8_t who = readRegister(spi_imu_handle, WHO_AM_I);
    ESP_LOGI(TAG, "LIS3DH WHO_AM_I: 0x%02X (should be 0x33)", who);

    // 100 Hz, all axes enabled
    writeRegister(spi_imu_handle, CTRL_REG1, 0x57);

    // ±2g, High resolution
    writeRegister(spi_imu_handle, CTRL_REG4, 0x08);
    ESP_LOGI(TAG, "IMU (LIS3DH) Initialized");
}

// These are the "bridge" functions the Bosch driver needs.
// They connect the generic driver to our ESP-IDF specific SPI functions.
BME68X_INTF_RET_TYPE bme68x_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.cmd = reg_addr | 0x80; // Read bit
    t.length = len * 8;
    t.rx_buffer = reg_data;
    esp_err_t ret = spi_device_polling_transmit(spi_bme_handle, &t);
    return (ret == ESP_OK) ? BME68X_INTF_RET_SUCCESS : BME68X_E_COM_FAIL;
}

BME68X_INTF_RET_TYPE bme68x_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.cmd = reg_addr & ~0x80; // Write bit
    t.length = len * 8;
    t.tx_buffer = reg_data;
    esp_err_t ret = spi_device_polling_transmit(spi_bme_handle, &t);
    return (ret == ESP_OK) ? BME68X_INTF_RET_SUCCESS : BME68X_E_COM_FAIL;
}

void bme68x_delay_us(uint32_t period, void *intf_ptr) {
    esp_rom_delay_us(period);
}

void initBME688()
{
    // The SPI bus is already initialized in initIMU(). We just add the BME device.
    spi_device_interface_config_t devcfg = {
        .command_bits = 0, // BME680 doesn't use command phase
        .address_bits = 8,
        .clock_speed_hz = 1 * 1000 * 1000, // 1 MHz
        .mode = 0, // BME680 uses SPI mode 0
        .spics_io_num = AQ_CS,
        .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &devcfg, &spi_bme_handle));

    // Initialize the BME68x device structure
    bme_dev.intf = BME68X_SPI_INTF;
    bme_dev.read = bme68x_spi_read;
    bme_dev.write = bme68x_spi_write;
    bme_dev.delay_us = bme68x_delay_us;
    bme_dev.intf_ptr = NULL; // We are using a global handle `spi_bme_handle`

    int8_t rslt = bme68x_init(&bme_dev);
    if (rslt != BME68X_OK) {
        ESP_LOGE(TAG, "BME688 init failed with code %d", rslt);
    } else {
        ESP_LOGI(TAG, "BME688 Initialized successfully");
    }
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
  config.frame_size = FRAMESIZE_QVGA; // 320×240, we crop center 64×64
  config.jpeg_quality = 12;
  config.fb_count = 1;
  config.fb_location = CAMERA_FB_IN_DRAM;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    ESP_LOGE(TAG, "Camera init failed: 0x%x", err);
  }
  else
    ESP_LOGI(TAG, "Camera Initialized");
}

// ===== Getter Functions for ADC Configuration =====

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

bool is_adc_cali_enabled_chan0(void)
{
    return adc_cali_enabled_chan0;
}

bool is_adc_cali_enabled_chan1(void)
{
    return adc_cali_enabled_chan1;
}

// ===== Getter Functions for Peripheral Handles =====

spi_device_handle_t get_spi_imu_handle(void)
{
    return spi_imu_handle;
}

spi_device_handle_t get_spi_bme_handle(void)
{
    return spi_bme_handle;
}

struct bme68x_dev *get_bme_dev_handle(void)
{
    return &bme_dev;
}
