#pragma once

#include "UFO_DataStorage.h"
#include "UFO_TaskMinimal.h"

#define PRESSURE_SEALEVELHPA (1013.25F) /**< Average sea level pressure is 1013.25 hPa */

// WARNING!!!
//  ESP_DRAM_LOGx for interraption (maybe for IRAM_ATTR also )
void /*IRAM_ATTR*/ UFO_Task_BME(void *arg)
{
    UFO_SendToFunckMinimal *data = (UFO_SendToFunckMinimal *)arg;
    delay(10);
    Serial.print("Task created: ");
    Serial.println(data->name);
    Serial.print("\tupdate time:");
    Serial.println(data->updTime);

    // Adafruit_BME280 bme;
    delay(10);
    // if (!bme.begin(118))
    // {
    //     Serial.println("ERROR bme");
    // }
    // bme.setSeaLevelPre(PRESSURE_SEALEVELHPA);

    
    
    // // bme.setTemperatureCompensation(-8.); todo
    delay(10);

    while (true)
    {
        // bme.readData(UFO_BMEData._Tem, UFO_BMEData._Pre, UFO_BMEData._Lvl);
        // Serial.print("P/T/l\t");
        // Serial.print(UFO_BMEData._Pre);
        // Serial.print("\t");
        // Serial.print(UFO_BMEData._Tem);
        // Serial.print("\t");
        // Serial.println(UFO_BMEData._Lvl);
        delay(data->updTime); // (this delay is neccesary for yeild)
    }

    Serial.print("Task terminated: ");
    Serial.println(data->name);

    delete data;
    vTaskDelete(NULL);
}