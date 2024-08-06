#pragma once

#include "Arduino.h"
#include "Wire.h"



// #define USING_OTA
// #define USING_WIFI

#ifdef USING_WIFI
    #define WF_MODE_STA     1
    #define WF_MODE_AP      2
    #define WF_MODE_AUTO    3

    // #define GTD_WIFI_NET_NAME   ((const char*)"TP-Link_2722")
    // #define GTD_WIFI_NET_PASS   ((const char*)"89565544")
    // #define GTD_WIFI_NET_NAME   ((const char*)"KULON")
    // #define GTD_WIFI_NET_PASS   ((const char*)"1Qwertyuiop")
    
    #define GTD_WIFI_NET_NAME   ((const char*)"UFO_ESP32")
    #define GTD_WIFI_NET_PASS   ((const char*)"12345678")
    
    #define GTD_WIFI_OWN_NAME   ((const char*)"RC_ESP32")
    #define GTD_WIFI_OWN_PASS   ((const char*)"12345678")

    // #define GTD_WF_MODE WF_MODE_AUTO
    #define GTD_WF_MODE WF_MODE_STA
    // #define GTD_WF_MODE WF_MODE_AP
 
    #if (GTD_WF_MODE == WF_MODE_AUTO || GTD_WF_MODE == WF_MODE_AP)
        #define LIMIT_WiFi_DEVICE_CONNETION_TIME ((uint64_t)5000) //(ms)  == 10 sec
        #define GTD_WIFI_MAX_CONNECTION (int(1))
        IPAddress GTD_WIFI_local_ip(192, 168, 2, 1);
        IPAddress GTD_WIFI_gateway(192, 168, 2, 1);
    #endif

 
#else
    #undef USING_OTA
    #undef USING_ASWSERVER
#endif
