#ifndef wifi_stream_h
#define wifi_stream_h

#include "ota.h"
#include "sd.h"
#include "esp_timer.h"

void streamFolderToTCP(char* folderName,char* serverIp);
#endif