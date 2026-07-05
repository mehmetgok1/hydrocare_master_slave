#ifndef wifi_stream_h
#define wifi_stream_h

#include "ota.h"
#include "sd.h"
#include "esp_timer.h"

void stream_folder_to_tcp(const char* folder_name,char* server_ip);
#endif