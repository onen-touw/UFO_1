#pragma once 

#include "UFO_Config.h"

#define OUTPUT_ESC1 13
#define OUTPUT_ESC2 26
#define OUTPUT_ESC3 14
#define OUTPUT_ESC4 27


#define UFO_DEFAULT_FRONT_RIGHT     OUTPUT_ESC1
#define UFO_DEFAULT_FRONT_LEFT      OUTPUT_ESC2
#define UFO_DEFAULT_BACK_RIGHT      OUTPUT_ESC3
#define UFO_DEFAULT_BACK_LEFT       OUTPUT_ESC4

#define UFO_MOTOR_CHANEL_FRONT_LEFT     ((uint8_t)0)
#define UFO_MOTOR_CHANEL_FRONT_RIGHT    ((uint8_t)2)
#define UFO_MOTOR_CHANEL_BACK_RIGHT     ((uint8_t) 1)
#define UFO_MOTOR_CHANEL_BACK_LEFT      ((uint8_t)3)

#define UFO_MOTOR_PWM_FREQ          ((uint16_t)50)
#define UFO_MOTOR_PWM_RESOLUTION    ((uint8_t)16)

#define UFO_MOTOR_COUNT             ((uint8_t)4)

#define UFO_MOTOR_VAL_MAX           ((uint16_t)2000)
#define UFO_MOTOR_VAL_MIN           ((uint16_t)1000)

#define UFO_MOTOR_RES_VAL_MAX           ((uint16_t)(std::pow(2,UFO_MOTOR_PWM_RESOLUTION)/10))           //2^16/10
#define UFO_MOTOR_RES_VAL_MIN           ((uint16_t)(UFO_MOTOR_RES_VAL_MAX/2))                           //2^16/2/10

#define UFO_DEFAULT_ARM_DUTY            ((uint16_t)(0.14*(UFO_MOTOR_VAL_MAX - UFO_MOTOR_VAL_MIN) + UFO_MOTOR_VAL_MIN))


// todo castom system 
// todo one-motor class(struct)  -- for store data
// todo arm-errors 

/// very hard:
//      todo pwm-class

// struct UFO_Motor 
// {
//     int16_t trust = 0;
//     int8_t channel = 0;
// };


class UFO_Motors
{
private:
    bool castom = false;
    bool _arm = false;
    int16_t _motors[UFO_MOTOR_COUNT]={0,0,0,0};
public:
    UFO_Motors()
    {
        // if (!castom)
        // {
        //     pinMode(UFO_DEFAULT_FRONT_RIGHT, OUTPUT);
        //     pinMode(UFO_DEFAULT_FRONT_LEFT, OUTPUT);
        //     pinMode(UFO_DEFAULT_BACK_RIGHT, OUTPUT);
        //     pinMode(UFO_DEFAULT_BACK_LEFT, OUTPUT);

        //     ledcSetup(UFO_MOTOR_CHANEL_FRONT_LEFT, UFO_MOTOR_PWM_FREQ, UFO_MOTOR_PWM_RESOLUTION);
        //     ledcSetup(UFO_MOTOR_CHANEL_BACK_RIGHT, UFO_MOTOR_PWM_FREQ, UFO_MOTOR_PWM_RESOLUTION);
        //     ledcSetup(UFO_MOTOR_CHANEL_FRONT_RIGHT, UFO_MOTOR_PWM_FREQ, UFO_MOTOR_PWM_RESOLUTION);
        //     ledcSetup(UFO_MOTOR_CHANEL_BACK_LEFT, UFO_MOTOR_PWM_FREQ, UFO_MOTOR_PWM_RESOLUTION);

        //     ledcAttachPin(UFO_DEFAULT_FRONT_LEFT, UFO_MOTOR_CHANEL_FRONT_LEFT);
        //     ledcAttachPin(UFO_DEFAULT_BACK_RIGHT, UFO_MOTOR_CHANEL_BACK_RIGHT); 
        //     ledcAttachPin(UFO_DEFAULT_FRONT_RIGHT, UFO_MOTOR_CHANEL_FRONT_RIGHT); 
        //     ledcAttachPin(UFO_DEFAULT_BACK_LEFT, UFO_MOTOR_CHANEL_BACK_LEFT); 

        // }
        // MuteAll();
    }
    
    void Begin(){
        // pinMode(UFO_DEFAULT_FRONT_RIGHT, OUTPUT);
        // pinMode(UFO_DEFAULT_FRONT_LEFT, OUTPUT);
        // pinMode(UFO_DEFAULT_BACK_RIGHT, OUTPUT);
        pinMode(UFO_DEFAULT_BACK_LEFT, OUTPUT);

        // ledcSetup(UFO_MOTOR_CHANEL_FRONT_LEFT, UFO_MOTOR_PWM_FREQ, UFO_MOTOR_PWM_RESOLUTION);
        // ledcSetup(UFO_MOTOR_CHANEL_BACK_RIGHT, UFO_MOTOR_PWM_FREQ, UFO_MOTOR_PWM_RESOLUTION);
        // ledcSetup(UFO_MOTOR_CHANEL_FRONT_RIGHT, UFO_MOTOR_PWM_FREQ, UFO_MOTOR_PWM_RESOLUTION);
        ledcSetup(UFO_MOTOR_CHANEL_BACK_LEFT, UFO_MOTOR_PWM_FREQ, UFO_MOTOR_PWM_RESOLUTION);

        // ledcAttachPin(UFO_DEFAULT_FRONT_LEFT, UFO_MOTOR_CHANEL_FRONT_LEFT);
        // ledcAttachPin(UFO_DEFAULT_BACK_RIGHT, UFO_MOTOR_CHANEL_BACK_RIGHT);
        // ledcAttachPin(UFO_DEFAULT_FRONT_RIGHT, UFO_MOTOR_CHANEL_FRONT_RIGHT);
        ledcAttachPin(UFO_DEFAULT_BACK_LEFT, UFO_MOTOR_CHANEL_BACK_LEFT);

        _ForceSendAll(UFO_MOTOR_VAL_MIN);
        
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
    void Send(uint8_t chan, uint16_t val){
        if (_arm)
        {
           _ForceSend(chan, val);
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
        MuteAll();
        _arm = false;
        Serial.println("disArmed");
    }

    void Debug(){
        Serial.print("Motors trust: ");
        for (size_t i = 0; i < UFO_MOTOR_COUNT; ++i)
        {
            Serial.print(_motors[i]);
            Serial.print("; ");
        }
        Serial.println();
    }

    ~UFO_Motors() {}

private:
    void _ForceSend(int8_t chan, int16_t val)
    {
        int16_t v = constrain(val, UFO_MOTOR_VAL_MIN, UFO_MOTOR_VAL_MAX);
        v = map(v, UFO_MOTOR_VAL_MIN, UFO_MOTOR_VAL_MAX, UFO_MOTOR_RES_VAL_MIN, UFO_MOTOR_RES_VAL_MAX);

        Serial.print("chan: ");
        Serial.print(chan);
        Serial.print("; ");
        Serial.print("val: ");
        Serial.print(val);
        Serial.print(" ");
        Serial.println(v);
        
        ledcWrite(chan, v);
        _motors[chan] = val;
    }

    void _ForceSendAll(int16_t val)
    {
        for (size_t i = 0; i < UFO_MOTOR_COUNT; ++i)
        {
            _ForceSend(i, val);
        }
    }
};