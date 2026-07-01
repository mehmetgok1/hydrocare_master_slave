/**
 * @copyright (C) 2017 Melexis N.V.
 * Adapted for ESP-IDF v6.0.1 Master Device Handle Model
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "rom/ets_sys.h"
#include "esp_log.h"
#include "MLX90641_I2C_Driver.h"

// Reference your existing global I2C master bus handle
extern i2c_master_bus_handle_t i2c0_bus_hdl;

// Dedicated device handle for the MLX90641 matrix
static i2c_master_dev_handle_t mlx90641_dev_hdl = NULL;

void MLX90641_I2CInit()
{   
    // If already initialized, do nothing
    if (mlx90641_dev_hdl != NULL) {
        return;
    }

    // Attach the MLX90641 as a device onto the master bus
    // Default factory slave address for MLX90641 is 0x33
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x33, 
        .scl_speed_hz = 100000, // 100kHz Standard Mode is used here
    };

    esp_err_t err = i2c_master_bus_add_device(i2c0_bus_hdl, &dev_cfg, &mlx90641_dev_hdl);
    if (err != ESP_OK) {
        ESP_LOGE("MLX90641_I2C", "Failed to add MLX90641 device to I2C bus (%s)", esp_err_to_name(err));
    }
}

int MLX90641_I2CGeneralReset(void)
{    
    if (mlx90641_dev_hdl == NULL) {
        MLX90641_I2CInit();
    }

    // Melexis General Reset: Send 0x06 to general call address
    uint8_t reset_cmd = 0x06;
    esp_err_t err = i2c_master_transmit(mlx90641_dev_hdl, &reset_cmd, 1, pdMS_TO_TICKS(100));
    
    if (err != ESP_OK) {
        return -1;
    }         
    
    vTaskDelay(pdMS_TO_TICKS(50));   
    return 0;
}

int MLX90641_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data)
{
    if (mlx90641_dev_hdl == NULL) {
        MLX90641_I2CInit();
    }

    uint8_t reg_buf[2] = {
        (uint8_t)(startAddress >> 8), 
        (uint8_t)(startAddress & 0x00FF)
    };
    
    size_t bytes_to_read = nMemAddressRead * 2;
    static uint8_t rx_buf[3328]; 
    if (bytes_to_read > sizeof(rx_buf)) {
        return -1;
    }

    // Pass the registered device handle here instead of the raw bus handle
    esp_err_t err = i2c_master_transmit_receive(mlx90641_dev_hdl, 
                                                reg_buf, sizeof(reg_buf), 
                                                rx_buf, bytes_to_read, 
                                                pdMS_TO_TICKS(500));
    if (err != ESP_OK) {
        return -1;
    }

    for (int cnt = 0; cnt < nMemAddressRead; cnt++) {
        int idx = cnt << 1;
        data[cnt] = ((uint16_t)rx_buf[idx] << 8) | rx_buf[idx + 1];
    }
    
    return 0;   
} 

void MLX90641_I2CFreqSet(int freq)
{
    // Handled natively via scl_speed_hz configuration in MLX90641_I2CInit
}

int MLX90641_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data)
{
    if (mlx90641_dev_hdl == NULL) {
        MLX90641_I2CInit();
    }

    uint8_t tx_buf[4] = {
        (uint8_t)(writeAddress >> 8),
        (uint8_t)(writeAddress & 0x00FF),
        (uint8_t)(data >> 8),
        (uint8_t)(data & 0x00FF)
    };
    uint16_t dataCheck = 0;

    // Pass the registered device handle here instead of the raw bus handle
    esp_err_t err = i2c_master_transmit(mlx90641_dev_hdl, tx_buf, sizeof(tx_buf), pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        return -1;
    }         
    
    if (MLX90641_I2CRead(slaveAddr, writeAddress, 1, &dataCheck) != 0) {
        return -1;
    }
    
    if (dataCheck != data) {
        return -2;
    }    
    
    return 0;
}