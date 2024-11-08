#pragma once

#include "UFO_LoraStructDefinition.h"



class UFO_Lora_e220440T22D
{
private:
    
    UFO_LoraControlBlock _ctrlBlk;



public:
    UFO_Lora_e220440T22D(/* args */) {}
    ~UFO_Lora_e220440T22D() {}
    
    void Setup(){
        UFO_gpioConfig(UFO_LORA_AUX_PIN, gpio_mode_t::GPIO_MODE_INPUT);
        UFO_gpioConfig(UFO_LORA_M0_PIN, gpio_mode_t::GPIO_MODE_OUTPUT);
        UFO_gpioConfig(UFO_LORA_M1_PIN, gpio_mode_t::GPIO_MODE_OUTPUT);
        Serial2.begin(115200);
        Serial2.setTimeout(100);
    }
    
    
    void GoSleep(){
        gpio_set_level(UFO_LORA_M1_PIN, 1);   
        gpio_set_level(UFO_LORA_M1_PIN, 1);   
    }

private:
    esp_err_t __WaitResponce(int64_t ttime)
    {
        int64_t t = UFO_CoreTimeMilli();

        // make darn sure millis() is not about to reach max data type limit and start over
        if (((unsigned long)(t + ttime)) == 0)
        {
            t = 0;
        }

        while (gpio_get_level(UFO_LORA_AUX_PIN) == LOW)
        {
            if ((UFO_CoreTimeMilli() - t) > ttime)
            {
                Serial.println("Timeout error!");
                return ESP_FAIL;
            }
        }
    }

    esp_err_t __NoYeildWait(int64_t ttime)
    {
        int64_t t = UFO_CoreTimeMilli();

        // make darn sure millis() is not about to reach max data type limit and start over
        if (((unsigned long)(t + ttime)) == 0)
        {
            t = 0;
        }

        while ((UFO_CoreTimeMilli() - t) > ttime) {}
    };

    void __Send(){

        if (xSemaphoreTake(_ctrlBlk._lock, 300))
        {
            if (_ctrlBlk._ready)
            {
                uint8_t len =Serial2.write( _ctrlBlk._data._payload,  _ctrlBlk._data._len);
                if (len != _ctrlBlk._data._len)
                {
                    Serial.println("Error occurred during sending");        //add counter
                }
            }
            xSemaphoreGive(_ctrlBlk._lock);
        }
        
        __WaitResponce(200);

        while (Serial2.available())
        {
            Serial2.read();
        }
    }

    void __Reciev(){

        String s = Serial2.readString();

        if (s.length() <= 1)
        {
            Serial.println("Error occurred during recieving");        //add counter
        }

        // _calbackFunc
    }
    
    
};
