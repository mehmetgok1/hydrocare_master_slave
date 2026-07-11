#ifndef wifi_stream_h
#define wifi_stream_h

#include "ota.h"
#include "sd.h"
#include "esp_timer.h"
#include "gap.h"
#include <sys/stat.h>



// Define your AWS details here
#define API_GATEWAY_URL CONFIG_AWS_API_GATEWAY_URL
#define API_KEY CONFIG_AWS_API_KEY
void stream_folder_to_tcp(const char* folder_name,char* server_ip);
#endif