#pragma once 

#include "UFO_Config.h"

#include "UFO_Driver/UFO_ESC.h"

#define UFO_PIN_ESC1 13
#define UFO_PIN_ESC2 26
#define UFO_PIN_ESC3 14
#define UFO_PIN_ESC4 27


#define UFO_DEFAULT_FRONT_LEFT      0
#define UFO_DEFAULT_BACK_RIGHT      1
#define UFO_DEFAULT_FRONT_RIGHT     2
#define UFO_DEFAULT_BACK_LEFT       3
#define UFO_MOTOR_COUNT             ((uint8_t)4)

#define UFO_MOTOR_VAL_MAX           ((uint16_t)2000)
#define UFO_MOTOR_VAL_MIN           ((uint16_t)1000)

#define UFO_MOTOR_RES_VAL_MAX       ((uint16_t)(std::pow(2, UFO_PWM_RESOLUTION)/10))           //2^16/10
#define UFO_MOTOR_RES_VAL_MIN       ((uint16_t)(UFO_MOTOR_RES_VAL_MAX/2))                           //2^16/2/10

#define UFO_DEFAULT_ARM_DUTY            ((uint16_t)(0.14*(UFO_MOTOR_VAL_MAX - UFO_MOTOR_VAL_MIN) + UFO_MOTOR_VAL_MIN))


// todo castom system 
// [+] todo one-motor class(struct)  -- for store data
// todo arm-errors 

// [+] very hard:
//      todo pwm-class

struct UFO_Motor
{
    UFO_ESC_driver _driver;
    uint16_t _trust = 0;
};

class UFO_Motors
{
private:
    bool _arm = false;
    UFO_Motor _motors[UFO_MOTOR_COUNT]= {};
public:
    UFO_Motors()
    {
    }
    
    void Begin(){
        uint8_t id = 0;
        _motors[UFO_DEFAULT_FRONT_LEFT]._driver.Setup(id++, UFO_PIN_ESC1);
        _motors[UFO_DEFAULT_BACK_RIGHT]._driver.Setup(id++, UFO_PIN_ESC2);
        _motors[UFO_DEFAULT_FRONT_RIGHT]._driver.Setup(id++, UFO_PIN_ESC3);
        _motors[UFO_DEFAULT_BACK_LEFT]._driver.Setup(id++, UFO_PIN_ESC4);
        _ForceSendAll(UFO_MOTOR_VAL_MIN);
    }
    
    uint16_t GetTrust(uint8_t ind){
        return _motors[ind]._trust;
    }

    void SendAll(uint16_t val){
        for (size_t i = 0; i < UFO_MOTOR_COUNT; ++i)
        {
            Send(i, val);
        }
    }

    void MuteAll(){
        SendAll(UFO_MOTOR_VAL_MIN);
    }

    bool IsArmed(){
        return _arm;
    }
    // !!!WARNING!!! val === [1000; 2000]
    void Send(uint8_t motorind, uint16_t val){
        if (_arm)
        {
           _ForceSend(motorind, val);
        }
        else {
            Serial.println("arm error");
        }
    }
    
    void Arm(){
        if (_arm)
        {
            return;
        }
        Serial.println("armed");
        _arm = true;
        SendAll(UFO_DEFAULT_ARM_DUTY);
    }

    void DisArm(){
        if (!_arm)
        {
            return;
        }
        Serial.println("disArmed");
        MuteAll();
        _arm = false;
    }

    void Debug(){
        Serial.print("Motors trust: ");
        for (size_t i = 0; i < UFO_MOTOR_COUNT; ++i)
        {
            Serial.print(i);
            Serial.print(": ");
            Serial.print(_motors[i]._trust);
            Serial.print("; ");
        }
        Serial.println();
    }

    ~UFO_Motors() {}

private:
    void _ForceSend(uint8_t mototInd, int16_t val)
    {
        int16_t v = constrain(val, UFO_MOTOR_VAL_MIN, UFO_MOTOR_VAL_MAX);
        _motors[mototInd]._trust=v;
        v = map(v, UFO_MOTOR_VAL_MIN, UFO_MOTOR_VAL_MAX, UFO_MOTOR_RES_VAL_MIN, UFO_MOTOR_RES_VAL_MAX);

        Serial.print("motor: ");
        Serial.print(mototInd);
        Serial.print("; ");
        Serial.print("val: ");
        Serial.print(val);
        Serial.print(" ");
        Serial.println(v);
        
        _motors[mototInd]._driver.Write(v);
    }

    void _ForceSendAll(int16_t val)
    {
        for (size_t i = 0; i < UFO_MOTOR_COUNT; ++i)
        {
            _ForceSend(i, val);
        }
    }
};