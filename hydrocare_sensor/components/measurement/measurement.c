#include "measurement.h"
#include "config.h"
#include "esp_camera.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "MLX90641.h"

// Include the header for the official Bosch BME68x C driver
#include "bme68x.h"


static const char *TAG = "MEASUREMENT";

#define R_LOAD 10000.0
#define VREF 3.3

// ===== LIS3DH Registers =====
#define OUT_X_L 0x28

uint16_t ambLight = 0;
uint16_t microphone = 0;  // Microphone ADC reading
float ax = 0, ay = 0, az = 0;

// Global variables defined in the header
float bme_temp = 0;
float bme_hum = 0;
float bme_pres = 0;
float bme_gas = 0;

// MLX90641 refresh rates (Control register 0x800D bits 10:7):
// -----------------------------------------------------------
// Bit    Freq      Sec/frame          POR Delay (ms)  Sample Every (ms)
// 0x00 = 0.5 Hz    2 sec              4080 ms         2400 ms
// 0x01 = 1 Hz      1 sec/frame        2080 ms         1200 ms
// 0x02 = 2 Hz      0.5 sec/frame      1080 ms         600 ms (default)
// 0x03 = 4 Hz      0.25 sec/frame     580 ms          300 ms
// 0x04 = 8 Hz      0.125 sec/frame    330 ms          150 ms
// 0x05 = 16 Hz     0.0625 sec/frame   205 ms           75 ms
// 0x06 = 32 Hz     0.03125 sec/frame  143 ms           38 ms
// 0x07 = 64 Hz     0.015625 sec/frame 112 ms           19 ms

MLX90641_handle_t myIRcam; // C-style handle for the thermal camera

// Helper function to read multiple bytes from a register via SPI
static void readMultiple(spi_device_handle_t spi, uint8_t startReg, uint8_t *buffer, uint8_t len)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.cmd = 0xC0 | startReg; // Read + Auto increment
    t.length = len * 8;
    t.rx_buffer = buffer;
    spi_device_polling_transmit(spi, &t);
}

void readAcceleration()
{
  uint8_t rawData[6];
  readMultiple(get_spi_imu_handle(), OUT_X_L, rawData, 6);

  int16_t x = (int16_t)(rawData[1] << 8 | rawData[0]);
  int16_t y = (int16_t)(rawData[3] << 8 | rawData[2]);
  int16_t z = (int16_t)(rawData[5] << 8 | rawData[4]);

  // High-resolution mode: 12-bit left-aligned
  x >>= 4;
  y >>= 4;
  z >>= 4;

  // ±2g sensitivity = 1 mg/LSB
  ax = x * 0.001f;
  ay = y * 0.001f;
  az = z * 0.001f;
}

void measureAmbLight()
{
  int raw = 0;
  // Read from ADC_CHANNEL_0 (GPIO 1) using the new one-shot driver
  // We get the handle from the config component now
  ESP_ERROR_CHECK(adc_oneshot_read(get_adc1_handle(), ADC_CHANNEL_0, &raw));

  // We also get the calibration handles and status via getter functions
  int voltage_mv = 0;
  if (adc_cali_enabled_chan0) {
      ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle_chan0, raw, &voltage_mv));
  } else {
      // Fallback to rough calculation if calibration failed
      voltage_mv = (raw / 4095.0) * VREF * 1000;
  }

  float current = (voltage_mv / 1000.0) / R_LOAD; // in Amps
  float current_uA = current * 1e6;               // convert to microamps
  ambLight = (uint16_t)current_uA; // calibration factor (adjust!)
}

void measureMicrophone()
{
  int raw = 0;
  // Read from ADC_CHANNEL_1 (GPIO 2) using the new one-shot driver
  ESP_ERROR_CHECK(adc_oneshot_read(get_adc1_handle(), ADC_CHANNEL_1, &raw));

  int voltage_mv = 0;
  if (is_adc_cali_enabled_chan1()) {
      ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle_chan1, raw, &voltage_mv));
  }

  microphone = (uint16_t)voltage_mv;  // Store in millivolts
}

void measureIRTemp()
{
    // This function now depends on your C-based MLX90641 driver.
    // The logic remains the same, but the function calls will change.
    // mlx90641_read_temp_c(&myIRcam);
    // mlx90641_clear_new_data_bit(&myIRcam);
}

void measureBME688()
{
    uint8_t n_fields;
    struct bme68x_data data;
    struct bme68x_conf conf;

    // Get current sensor configuration
    bme68x_get_conf(&conf, get_bme_dev_handle());
    // Set desired oversampling
    conf.os_hum = BME68X_OS_16X;
    conf.os_pres = BME68X_OS_1X;
    conf.os_temp = BME68X_OS_2X;
    bme68x_set_conf(&conf, get_bme_dev_handle());

    // Get the data
    bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, get_bme_dev_handle());
}