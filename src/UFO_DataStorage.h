#pragma once
#include "UFO_Config.h"
// #include "RC_Event.h"


struct UFO_Data_IMU
{
    float
        Yaw = 0,
        Pitch = 0,
        Roll = 0;
    bool locked = false;
} UFO_IMUData;

struct UFO_Data_WiFi{
    bool connection = false;
} UFO_WiFiData;


struct UFJ_Data_BME
{
    float _Tem = 0;
    float _Pre = 0;
    float _Lvl = 0;
} UFO_BMEData;