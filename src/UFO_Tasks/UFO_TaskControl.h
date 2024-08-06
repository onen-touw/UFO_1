#pragma once 


#include "UFO_TaskMinimal.h"
#include "../UFO_Control.h"


// WARNING!!!
//  ESP_DRAM_LOGx for interraption (maybe for IRAM_ATTR also )
void /*IRAM_ATTR*/ UFO_Task_Control(void *arg)
{
    UFO_SendToFunckMinimal *data = (UFO_SendToFunckMinimal *)arg;
    delay(10);
    Serial.print("Task created: ");
    Serial.println(data->name);
    Serial.print("\tupdate time:");
    Serial.println(data->updTime);


    while (true)
    {

        
        delay(data->updTime); // (this delay is neccesary for yeild)
    }

    Serial.print("Task terminated: ");
    Serial.println(data->name);

    delete data;
    vTaskDelete(NULL);
}