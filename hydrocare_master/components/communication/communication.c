#include "communication.h"

static const char *TAG = "SPI_MASTER";

// Global SPI Device Handle
static spi_device_handle_t spi_handle;
static uint8_t *spiTxBuffer = NULL;
static uint8_t *spiRxBuffer = NULL;

extern bool debug_infos;
// ==================== INITIALIZATION ====================
void initSPIComm(void) {
    ESP_LOGI(TAG, "Initializing SPI on SPI3_HOST...");

    // 1. Configure the SPI Bus
    spi_bus_config_t buscfg = {
        .miso_io_num = SPI_MISO,
        .mosi_io_num = SPI_MOSI,
        .sclk_io_num = SPI_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = SPI_BUFFER_SIZE + 8 // Room for buffer + command bytes
    };

    // 2. Configure the SPI Device attached to the bus
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SPI_CLOCK_HZ, // e.g., 10000000 (10 MHz)
        .mode = 0,                      // SPI mode 0
        .spics_io_num = SPI_CS,         // Let ESP-IDF handle CS automatically
        .queue_size = 3,                // We only need a small queue for polling
        // Add a small pre-transaction delay to mimic your delayMicroseconds(50)
        .cs_ena_pretrans = 1,           
        .cs_ena_posttrans = 1,
    };

    // Initialize the SPI bus on SPI3_HOST (DMA channel auto-selected in IDF v5/v6)
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    
    // Attach the Slave device to the SPI bus
    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &devcfg, &spi_handle));

    ESP_LOGI(TAG, "SPI Init OK - Protocol-based 10 MHz");
}

void allocateSPIBuffer(void) {
    if (spiRxBuffer == NULL) {
        spiRxBuffer = (uint8_t*) heap_caps_calloc(1, SPI_BUFFER_SIZE + 2, MALLOC_CAP_DMA);
        if (!spiRxBuffer) ESP_LOGE(TAG, "RX DMA allocation failed!");
        ESP_LOGI(TAG, "RX DMA buffer allocated");
    }
    
    // Allocate a persistent TX buffer full of zeroes
    if (spiTxBuffer == NULL) {
        spiTxBuffer = (uint8_t*) heap_caps_calloc(1, SPI_BUFFER_SIZE + 2, MALLOC_CAP_DMA);
        if (!spiTxBuffer) ESP_LOGE(TAG, "TX DMA allocation failed!");
        ESP_LOGI(TAG, "TX DMA buffer allocated");
    }
}

void deallocateSPIBuffer(void) {
    if (spiRxBuffer != NULL) {
        heap_caps_free(spiRxBuffer);
        spiRxBuffer = NULL;
    }
    if (spiTxBuffer != NULL) {
        heap_caps_free(spiTxBuffer);
        spiTxBuffer = NULL;
    }
    ESP_LOGI(TAG, "SPI DMA buffers freed");
}

// ==================== PROTOCOL IMPLEMENTATION ====================

uint8_t spiRead(uint8_t address) {
    if (address > 0x7F) {
        ESP_LOGE(TAG, "Read address 0x%02X exceeds 7-bit space", address);
        return 0xFF;
    }
    
    uint8_t cmdByte = PROTO_CMD_READ | (address & PROTO_ADDR_MASK);
    uint8_t tx_data[2] = {cmdByte, 0x00}; // Command + Dummy byte
    uint8_t rx_data[2] = {0};

    spi_transaction_t t = {
        .length = 16, // 2 bytes * 8 bits
        .tx_buffer = tx_data,
        .rx_buffer = rx_data
    };

    // polling_transmit blocks until done, keeps transaction tight
    spi_device_polling_transmit(spi_handle, &t);
    
    esp_rom_delay_us(50); // Mimic your Arduino delay between transactions
    
    return rx_data[1]; // Return the slave's DATA/STATUS byte
}

void spiWrite(uint8_t address, uint8_t data) {
    if (address > 0x7F) {
        ESP_LOGE(TAG, "Write address 0x%02X exceeds 7-bit space", address);
        return;
    }
    
    uint8_t cmdByte = PROTO_CMD_WRITE | (address & PROTO_ADDR_MASK);
    uint8_t tx_data[3] = {cmdByte, 0x00, data}; // Command + Dummy + Data
    uint8_t rx_data[3] = {0};

    spi_transaction_t t = {
        .length = 24, // 3 bytes * 8 bits
        .tx_buffer = tx_data,
        .rx_buffer = rx_data
    };

    spi_device_polling_transmit(spi_handle, &t);
    
    esp_rom_delay_us(50);
}

void spiReadBulk(uint8_t address, uint8_t *buffer, uint16_t numBytes) {
    if (address > 0x7F || !buffer || numBytes > SPI_BUFFER_SIZE) {
        ESP_LOGE(TAG, "Bulk read invalid arguments");
        return;
    }

    // Set only the command bytes. The rest of spiTxBuffer is already 0x00 from calloc.
    // This perfectly satisfies your protocol where Master sends zeroes to clock in data.
    spiTxBuffer[0] = PROTO_CMD_READ | (address & PROTO_ADDR_MASK);
    spiTxBuffer[1] = 0x00; 

    // We use the persistent spiRxBuffer to catch the raw DMA transaction
    spi_transaction_t t = {
        .length = (numBytes + 2) * 8, 
        .tx_buffer = spiTxBuffer,
        .rx_buffer = spiRxBuffer 
    };

    // Execute the bulk DMA transfer natively
    // The driver automatically splits this >4092 byte payload into a linked 
    // list of DMA descriptors under the hood. You don't have to manage it.
    spi_device_transmit(spi_handle, &t); 
    
    uint8_t statusByte = spiRxBuffer[1];
    
    // Copy the pure payload (skipping the cmd/status bytes) into the user's struct
    memcpy(buffer, &spiRxBuffer[2], numBytes);

    esp_rom_delay_us(50);

    if(debug_infos) {
        ESP_LOGI(TAG, "Bulk read complete: Status=0x%02X, Bytes=%u", statusByte, numBytes);
    }
}

// ==================== CONVENIENCE FUNCTIONS ====================

// Helper to replace Arduino's millis()
static uint32_t get_millis(void) {
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

SensorDataPacket* readSlaveData(void) {
    if (spiRxBuffer == NULL) {
        ESP_LOGE(TAG, "ERROR: SPI RX Buffer not allocated!");
        return NULL;
    }

    // ========== STEP 1: Set Trigger ==========
    spiWrite(ADDR_CTRL, CTRL_TRIGGER_MEASUREMENT);
    if(debug_infos) ESP_LOGI(TAG, "write trigger measurement command");
    vTaskDelay(pdMS_TO_TICKS(10)); // Replaces Arduino delay()

    // ========== STEP 2: Poll for MEASURED status ==========
    uint32_t startTime = get_millis();
    uint8_t status = 0;
    bool measured = false;
    while (get_millis() - startTime < 2000) {
        status = spiRead(ADDR_STATUS);
        if (status != 0xFF && (status & STATUS_MEASURED)) {
            measured = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    
    if (!measured) {
        ESP_LOGE(TAG, "Timeout waiting for STATUS_MEASURED");
        return NULL;
    }

    // ========== STEP 3: Set Lock ==========
    spiWrite(ADDR_CTRL, CTRL_LOCK_BUFFERS);
    if(debug_infos) ESP_LOGI(TAG, "write lock data trigger");
    vTaskDelay(pdMS_TO_TICKS(5));

    // ========== STEP 4: Poll for LOCKED status ==========
    startTime = get_millis();
    bool locked = false;
    while (get_millis() - startTime < 2000) {
        status = spiRead(ADDR_STATUS);
        if (status != 0xFF && (status & STATUS_LOCKED)) {
            locked = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    
    if (!locked) {
        ESP_LOGE(TAG, "Timeout waiting for STATUS_LOCKED");
        // If we timed out here, we already sent a LOCK command.
        // We must send an UNLOCK to reset the slave for the next cycle.
        //spiWrite(ADDR_CTRL, CTRL_UNLOCK_BUFFERS);
        return NULL;
    }

    // ========== STEP 5: Bulk Read Sensor Data ==========
    spiReadBulk(ADDR_SENSOR_DATA, spiRxBuffer, SPI_BUFFER_SIZE);
    if(debug_infos) ESP_LOGI(TAG, "Ready to process sensor data packet");

    // ========== STEP 6: Release Lock ==========
    vTaskDelay(pdMS_TO_TICKS(5));
    spiWrite(ADDR_CTRL, CTRL_UNLOCK_BUFFERS);
    if(debug_infos) ESP_LOGI(TAG, "write unlock buffers command");
    vTaskDelay(pdMS_TO_TICKS(5));

    SensorDataPacket *packet = (SensorDataPacket*)(spiRxBuffer);
    
    if(debug_infos) {
        ESP_LOGI(TAG, "[Packet] Sequence: %u | Temp: %.1f C | Humidity: %.1f %% | Light: %u",
                 packet->sequence, packet->temperature, packet->humidity, packet->ambientLight);
    }
    
    return packet;
}

void sendIRLED(bool state) {
    ESP_LOGI(TAG, "Setting IR LED: %s", state ? "ON" : "OFF");
    spiWrite(ADDR_IR_LED, state ? 0x01 : 0x00);
}

void sendBrightness(uint8_t brightness) {
    if (brightness > 100) brightness = 100;
    ESP_LOGI(TAG, "Setting LED brightness: %u%%", brightness);
    spiWrite(ADDR_BRIGHTNESS, brightness);
}

// Downsample function remains pure logic, no IDF-specific changes needed
uint16_t* downsampleRGBFrame(uint16_t* rgbFrame64x64, uint16_t* outFrame16x16) {
    if (!rgbFrame64x64 || !outFrame16x16) return NULL;
    
    int outIdx = 0;
    
    for (int outY = 0; outY < 16; outY++) {
        for (int outX = 0; outX < 16; outX++) {
            int inY_start = outY * 4;
            int inX_start = outX * 4;
            
            uint32_t sumR = 0, sumG = 0, sumB = 0;
            uint8_t count = 0;
            
            for (int dy = 0; dy < 4; dy++) {
                for (int dx = 0; dx < 4; dx++) {
                    int inY = inY_start + dy;
                    int inX = inX_start + dx;
                    
                    if (inY < 64 && inX < 64) {
                        uint16_t pixel = rgbFrame64x64[inY * 64 + inX];
                        uint8_t r = (pixel >> 11) & 0x1F;
                        uint8_t g = (pixel >> 5) & 0x3F;
                        uint8_t b = pixel & 0x1F;
                        
                        sumR += r;
                        sumG += g;
                        sumB += b;
                        count++;
                    }
                }
            }
            
            if (count > 0) {
                uint8_t avgR = sumR / count;
                uint8_t avgG = sumG / count;
                uint8_t avgB = sumB / count;
                uint16_t averaged = ((avgR & 0x1F) << 11) | ((avgG & 0x3F) << 5) | (avgB & 0x1F);
                outFrame16x16[outIdx++] = averaged;
            }
        }
    }
    return outFrame16x16;
}