#pragma once

#include "UFO_TaskMinimal.h"
#include "../UFO_Sensors/UFO_Sensors_I2C/UFO_INA219.h"


struct SendToInaTask
{
    QueueHandle_t _q;
    UFO_I2C_Driver* _d;
};


#define INA3221_TAG "INA0"
void /*IRAM_ATTR*/ UFO_Task_INA(void *arg)
{
    UFO_SendToFunckMinimal *data = reinterpret_cast<UFO_SendToFunckMinimal *>(arg);
    SendToInaTask* d =reinterpret_cast<SendToInaTask*>(data->arg);
    UFO_I2C_Driver *driver = d->_d;
    QueueHandle_t queue = d->_q;

    delay(10);
    Serial.print("Task created: ");
    Serial.println(data->name);
    Serial.print("\tupdate time:");
    Serial.println(data->updTime);

    UFO_INA219 ina1(d->_d, 0x40);
    UFO_INA219 ina2(d->_d, 0x41);
    ina1.InitSensor();
    ina2.InitSensor();
    String bufStr;

    while (true)
    {
        ina1.Update();
        ina2.Update();
        UFO_INA3221_ChannelData d;
        for (size_t i = 0; i < 3; i++)
        {
            bufStr = ESP_TAG;
            bufStr += INA3221_TAG;
            bufStr += (i+1);
            d = ina.Get(i);
            bufStr += '@';
            bufStr += d._timestamp;
            bufStr += ',';
            bufStr += d._busVolt;
            bufStr += ',';
            bufStr += d._current;
            xQueueSend(queue, &bufStr, 300);
            delay(5);
        }
        delay(data->updTime); // (this delay is neccesary for yeild)
    }

    Serial.print("Task terminated: ");
    Serial.println(data->name);

    delete data;
    vTaskDelete(NULL);

}