#pragma once

#include "UFO_TaskMinimal.h"
#include "../UFO_Socket.h"

void /*IRAM_ATTR*/ UFO_Task_Socket(void *arg)
{

    UFO_SendToFunckMinimal *data = reinterpret_cast<UFO_SendToFunckMinimal*>(arg);
    delay(10);
    Serial.print("Task created: ");
    Serial.println(data->name);
    Serial.print("\tupdate time:");
    Serial.println(data->updTime);
    
    if (!data->arg)
    {
        Serial.println("data->arg* error");
    }

    UFO_SockConfigMinimal* cm = reinterpret_cast<UFO_SockConfigMinimal*>(data->arg);
    if (!cm)
    {
        Serial.println("UFO_SockConfigMinimal* error");
    }
    
    UFO_Socket sock;
    sock.SetConfig(cm);
    delay(10);

    // delete cm;
    
    sock.Setup();

    while (true)
    {
        // Serial.println('.');
        sock.Iteration();
        delay(data->updTime); // (this delay is neccesary for yeild)
    }

    Serial.print("Task terminated: ");
    Serial.println(data->name);
    
    // sock.Close();

    delete data;
    vTaskDelete(NULL);
}