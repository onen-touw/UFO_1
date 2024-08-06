#pragma once

#include "UFO_TaskMinimal.h"
#include "../UFO_WiFi.h"
#include "../UFO_Config.h"

RxDataHandler rxDH;
struct EngData
{
    int32_t throttle = 0;
    int32_t error = 3;
} endData;
void parsePacket(AsyncUDPPacket packet)
{
    // std::string msg = reinterpret_cast<const char *>(packet.data());
    String msg = reinterpret_cast<const char *>(packet.data());
    rxDH(msg);
    // int32_t val = mag.toInt();
    // if (abs(val - endData.throttle) > endData.error)
    // {
    //     endData.throttle = val;

    //     Serial.println(val);
    // }
    // toESC();
}

// WARNING!!!
//  ESP_DRAM_LOGx for interraption (maybe for IRAM_ATTR also )
void /*IRAM_ATTR*/ UFO_Task_WiFiControl(void *arg)
{
    UFO_SendToFunckMinimal *data = (UFO_SendToFunckMinimal *)arg;
    delay(10);
    Serial.print("Task created: ");
    Serial.println(data->name);
    Serial.print("\tupdate time:");
    Serial.println(data->updTime);

    UFO_WiFiHandler wfHandler(UFO_WiFi_Mode::UFO_WiFi_MODE_AP);
    delay(10);
    
    AsyncUDP udp;

    rxDH.Addhandler([&](RX_INDEX i, int32_t v)
                    {
            switch (i)
            {
            case RX_IND_THROTTLE:
                if (abs(v - endData.throttle) > endData.error){
                    endData.throttle = v;
                    Serial.println(v);
                    // u_int16_t PWM_width_us = constrain(endData.throttle, 1000, 2000);
                    // u_int16_t DutyCycle = map(PWM_width_us, 1000, 1800, 3275, 6553); // Dutycycle to generate a pwm signal with that width in microseconds(us)
                    // ledcWrite(1, DutyCycle);
                }
                break;
            default:
                break;
            } });

    if (!udp.listen(8080))
    {
        Serial.println("err");
    }
    udp.onPacket(parsePacket);

    while (true)
    {
        // wfHandler.Iteration();

        delay(data->updTime); // (this delay is neccesary for yeild)
    }

    Serial.print("Task terminated: ");
    Serial.println(data->name);

    delete data;
    vTaskDelete(NULL);
}