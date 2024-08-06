#pragma once
#include "UFO_TaskClassBase.h"
#include "UFO_PID.h"
#include "UFO_DataStorage.h"
#include "UFO_Motors.h"
// #include "driver/rmt.h"



class UFO_Control : public UFO_TaskClassBase
{
private:
    UFO_PID _rollPid = UFO_PID(0.2,  0.05, 0.2, 0.001);
    UFO_PID _pitchPid = UFO_PID(0.2,  0.05, 0.2, 0.001);
    int16_t _targetRollAngle = 0;
    int16_t _targetPitchAngle = 0;

    int16_t _trustTarget = 0;       //from RC

    UFO_Motors _motors;

public:
    UFO_Control(/* args */) {}
    ~UFO_Control() {}

    void Setup() final {
        _motors.Begin();
    }

    void Iteration() final {
        if (!_motors.IsArmed())
        {
            Serial.println("Control::Disarmed");
            return;
        }
        
        int16_t roll = _rollPid.Calculate(_targetRollAngle, UFO_IMUData.Roll);
        int16_t pitch = _pitchPid.Calculate(_targetPitchAngle, UFO_IMUData.Pitch);
        _motors.Send(UFO_MOTOR_CHANEL_FRONT_LEFT, _trustTarget + roll+ pitch);  
        _motors.Send(UFO_MOTOR_CHANEL_FRONT_RIGHT, _trustTarget - roll+ pitch);  
        _motors.Send(UFO_MOTOR_CHANEL_BACK_LEFT, _trustTarget + roll - pitch);
        _motors.Send(UFO_MOTOR_CHANEL_BACK_RIGHT, _trustTarget - roll - pitch);
        _motors.Debug();
        Serial.print("roll: ");
        Serial.print(roll);
        Serial.print(";\tpitch: ");

        Serial.println(pitch);
    }

    void SetTargetTrust(int16_t tr){
        _trustTarget = tr;
    }
    void SetArm(int8_t s){
        if (s)
        {
            _motors.Arm();
            return;
        }
        _motors.DisArm();
    }


    
};