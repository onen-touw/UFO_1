#pragma once
#include "UFO_TaskClassBase.h"
#include "UFO_math/UFO_PID.h"
#include "UFO_StructDefinition.h"
#include "UFO_Motors.h"
// #include "driver/rmt.h"



class UFO_Control : public UFO_TaskClassBase
{
private:
    UFO_PID _rollPid = UFO_PID(0.02,  0.01, 0.2, 0.05);
    UFO_PID _pitchPid = UFO_PID(0.02,  0.01, 0.2, 0.05);

    float _targetRollAngle = 0;  // from rc  [-30 30]    or less
    float _targetPitchAngle = 0;    // from rc  [-30 30]    or less
    int16_t _trustTarget = 0;       //from RC   [1000 2000]

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
        
        double roll = _rollPid.Calculate(_targetRollAngle, UFO_IMUData.Roll);          //maybe pids need in scale factor
        double pitch = _pitchPid.Calculate(_targetPitchAngle, UFO_IMUData.Pitch);

        // _motors.Send(UFO_MOTOR_CHANEL_FRONT_LEFT, _trustTarget + roll+ pitch);  
        // _motors.Send(UFO_MOTOR_CHANEL_FRONT_RIGHT, _trustTarget - roll+ pitch);  
        // _motors.Send(UFO_MOTOR_CHANEL_BACK_LEFT, _trustTarget + roll - pitch);
        // _motors.Send(UFO_MOTOR_CHANEL_BACK_RIGHT, _trustTarget - roll - pitch);

        _motors.Debug();
        Serial.print(">PID pitch:");
        Serial.println(pitch);

        Serial.print(">PID roll:");
        Serial.println(roll);
        Serial.print(">Angle pitch:");
        Serial.println(UFO_IMUData.Pitch* 180 / M_PI);
        Serial.print(">Angle roll:");
        Serial.println(UFO_IMUData.Roll* 180 / M_PI);
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