#pragma once

#include "UFO_TaskClassBase.h"

// #ifdef USING_WIFI
#include "UFO_Config.h"
#include <WiFi.h>

//to config section 
#define UFO_WIFI_NET_NAME ((const char *)"RC_ESP32")
#define UFO_WIFI_NET_PASS ((const char *)"12345678")
#define UFO_WIFI_OWN_NAME ((const char *)"UFO_ESP32")
#define UFO_WIFI_OWN_PASS ((const char *)"12345678")
#define LIMIT_WiFi_DEVICE_CONNETION_TIME ((uint64_t)5000) //(ms)
#define UFO_WIFI_MAX_CONNECTION (int(1))
#define UFO_WIFI_RECONECTION_COUNT ((int)1)

enum UFO_WiFi_Mode
{
    UFO_WiFi_MODE_AUTO,
    UFO_WiFi_MODE_STA,
    UFO_WiFi_MODE_AP,
};

class UFO_WiFiHandler:public UFO_TaskClassBase
{
private:
    IPAddress _local_ip = IPAddress(192, 168, 2, 2);
    UFO_WiFi_Mode _mode = UFO_WiFi_MODE_AUTO;
    bool _wasConnected = false;

private:
    void __StartAP()
    {
        WiFi.softAP(UFO_WIFI_OWN_NAME, UFO_WIFI_OWN_PASS, 1, 0, UFO_WIFI_MAX_CONNECTION);
        WiFi.softAPConfig(_local_ip, _local_ip, IPAddress(255, 255, 255, 0));
        delay(1000);
        Serial.println(WiFi.softAPIP());
        _wasConnected = true;
        return;
    }

    void __StartSTA()
    {
        WiFi.mode(WIFI_STA);
        WiFi.begin(UFO_WIFI_NET_NAME, UFO_WIFI_NET_PASS);
        Serial.print("Connecting to ");
        Serial.println(UFO_WIFI_NET_NAME);

        int8_t recCount = 0;
        while (recCount < UFO_WIFI_RECONECTION_COUNT)
        {
            if (WiFi.waitForConnectResult(LIMIT_WiFi_DEVICE_CONNETION_TIME) != WL_CONNECTED)
            {
                delay(2000);
            }
            else
            {
                _wasConnected = true;
                break;
            }
            ++recCount;
        }
        Serial.println(WiFi.localIP());
        return;
    }

    bool __CheckConnection()
    {
        if (WiFi.status() > 3)
        {
            return false;
        }
        return true;
    }

    void __Handle()
    {
        if (_wasConnected)
        {
            if (!__CheckConnection())
            {
                Serial.println("Reconection!");
                // Connect();
            }
            //todo: events

        }
    }

public:
    UFO_WiFiHandler() {
        Connect();
    }
    UFO_WiFiHandler(UFO_WiFi_Mode mode)
    {
        this->_mode = mode;
        Connect();
    }
    ~UFO_WiFiHandler() {}

    void Setup()final {}

    void Connect()
    {
        switch (_mode)
        {
        case UFO_WiFi_MODE_AUTO:
            __StartSTA();
            Serial.println("Connection failed!");
            Serial.println("Starting in Ap mode!");
            //todo
            delay(2000);
            __StartAP();
            break;
        case UFO_WiFi_MODE_STA:
            __StartSTA();
            break;
        case UFO_WiFi_MODE_AP:
            __StartAP();
            break;
        default:
            break;
        }
    }

    void SetMode(UFO_WiFi_Mode mode)
    {
        if (mode != _mode)
        {
            this->_mode = mode;
            Disconnect();
            Connect();
        }
    }

    void Iteration() final
    {
        __Handle();
    }

    void Disconnect()
    {
        _wasConnected = false;
        WiFi.disconnect(true);
    }
};
// #endif