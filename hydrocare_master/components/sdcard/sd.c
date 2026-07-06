
#include "sd.h"


static const char *TAG = "SDCARD";
sdmmc_card_t *global_card_ptr = NULL; // ADD THIS GLOBAL POINTER

esp_err_t write_file(const char *path, const char *data)
{
    ESP_LOGI(TAG, "Opening file %s", path);
    FILE *f = fopen(path, "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return ESP_FAIL;
    }
    
    // Safer and faster for writing raw strings
    fputs(data, f); 
    fclose(f);
    
    ESP_LOGI(TAG, "File written");
    return ESP_OK;
}

esp_err_t read_file(const char *path)
{
    ESP_LOGI(TAG, "Reading file %s", path);
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return ESP_FAIL;
    }
    char line[EXAMPLE_MAX_CHAR_SIZE];
    fgets(line, sizeof(line), f);
    fclose(f);

    // strip newline
    char *pos = strchr(line, '\n');
    if (pos) {
        *pos = '\0';
    }
    ESP_LOGI(TAG, "Read from file: '%s'", line);

    return ESP_OK;
}

void init_sd(){
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI(TAG, "Initializing SD card");
    ESP_LOGI(TAG, "Using SPI peripheral");

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.unaligned_multi_block_rw_max_chunk_size = 16;

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = MOSI,
        .miso_io_num = MISO,
        .sclk_io_num = SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 8192,
    };
    esp_err_t ret = spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return;
    }
    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SD_CS;
    slot_config.host_id = host.slot;
    ESP_LOGI(TAG, "Mounting filesystem");
    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &global_card_ptr);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                     "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        return;
    }
    ESP_LOGI(TAG, "Filesystem mounted");
    sdmmc_card_print_info(stdout, global_card_ptr);
    // First create a file.
    const char *file_hello = MOUNT_POINT"/hello.txt";
    char data[EXAMPLE_MAX_CHAR_SIZE];
    snprintf(data, EXAMPLE_MAX_CHAR_SIZE, "%s %s!\n", "Hello", global_card_ptr->cid.name);
    ret = write_file(file_hello, data);
    if (ret != ESP_OK) {
        return;
    }
}
// ADD THIS NEW FUNCTION
void deinit_sd() {
    if (global_card_ptr != NULL) {
        esp_err_t err = esp_vfs_fat_sdcard_unmount(MOUNT_POINT, global_card_ptr);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Card successfully unmounted. SPI bus released.");
            global_card_ptr = NULL;
        } else {
            ESP_LOGE(TAG, "Failed to unmount card (%s)", esp_err_to_name(err));
        }
    }
}
//uint8_t eepromRead(uint8_t address)
//{
//    return EEPROM.read(address);
//}
//void eepromWrite(uint8_t address, uint8_t value)
//{ 
//    EEPROM.write(address, value);
//    EEPROM.commit();
//}
//void initEEPROM()
//{
//    if (!EEPROM.begin(EEPROM_SIZE)) {
//        Serial.println("Failed to initialise EEPROM");
//        while (true);  // Stop execution
//    }
//    byte value = eepromRead(deviceRoleAddress);
//    Serial.print("Read byte from EEPROM: ");
//    Serial.println(value, HEX);
//}

