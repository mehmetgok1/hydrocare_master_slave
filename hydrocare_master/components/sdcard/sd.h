#ifndef MEMORY_H
#define MEMORY_H
#include "config.h"
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "sd_test_io.h"



//function declarations
void init_sd();
esp_err_t write_file(const char *path, char *data);
esp_err_t read_file(const char *path);


//definitionss
#define MOUNT_POINT "/hydrocare_sd"
#define EXAMPLE_MAX_CHAR_SIZE    64

//#define EEPROM_SIZE       1        // Size in bytes
//#define deviceRoleAddress 0        // Address to read/write
//
//extern String sessionFolder;      // Root session folder (e.g., 20260325_143022)

//void initEEPROM();
//uint8_t eepromRead(uint8_t address);
//void eepromWrite(uint8_t address, uint8_t value);

//void initSessionFolder();          // Create timestamped folder

#endif