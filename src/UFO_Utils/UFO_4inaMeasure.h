#pragma once

#include "UFO_Sensors/UFO_Sensors_I2C/UFO_INA219.h"

class UFO_4inaMeasure
{
private:
    uint8_t _sz = 0;
    uint8_t addr[4] = { 0x40u, 0x41, 0x44, 0x45 };
    UFO_INA219* _sensors = nullptr;

public:
    UFO_4inaMeasure() {
        _sensors =static_cast<UFO_INA219*>(operator new(4*sizeof(UFO_INA219)));
    }
    ~UFO_4inaMeasure() {
        for (size_t i = 0; i < _sz; i++)
        {
            _sensors->~UFO_INA219();
        }
        operator delete (_sensors);
    }

    void InitSensors(UFO_I2C_Driver* driver, uint8_t* adr, uint8_t sz){
        _sz = sz;
        for (size_t i = 0; i < _sz; i++)
        {
            UFO_INA219* ptr = new(_sensors + i) UFO_INA219(driver, adr[i]);
            if (!ptr)
            {
                Serial.println("UFO_4inaMeasure new problem");
                break;
            }
            Serial.print("Allocated: ");
            Serial.println(adr[i]);

            _sensors[i].InitSensor();
        }
    }


    String Update() {
        for (size_t i = 0; i < _sz; i++)
        {
            _sensors[i].Update();
            delay(5);
        }
        UFO_DataINA219 d;
        
        String s;

        for (size_t i = 0; i < _sz; i++)
        {
            d = _sensors[i].Get();    
            s+=i;
            s+="-|\t";
            s+= d._busVolt;
            s+="V|\t";
            s+= d._current;
            s+="mA\t\t\t";
        }
        return s;
    }


};