#ifndef OTA_H
#define OTA_H
#include "config.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_crt_bundle.h"

//void printFirmwareInfo();
//void manualUpdate();
void printUpdateError();
void performOTAUpdate();
void checkForUpdate();
void connectToWiFi();
extern bool otaUpdateAvailable;


#endif