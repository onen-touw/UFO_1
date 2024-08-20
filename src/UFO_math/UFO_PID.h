#pragma once


class UFO_PID
{

private:
    float _dt;
    float _Kp;
    float _Kd;
    float _Ki;
    float _pre_error;
    float _integral;

public:
    // Kp -  proportional gain
    // Ki -  Integral gain
    // Kd -  derivative gain
    // dt -  loop interval time
    UFO_PID(float Kp, float Ki, float Kd, float dt) : _Kp(Kp),_Ki(Ki), _Kd(Kd), _dt(dt),_pre_error(0), _integral(0)
    {}
    ~UFO_PID() {}

    // Returns the manipulated variable given a setpoint and current process value
    float Calculate(float setpoint, float processVal)
    {

        // Calculate error
        float error = setpoint - processVal;

        // Proportional term
        double Pout = _Kp * error;

        // Integral term
        _integral += error * _dt;
        float Iout = _Ki * _integral;

        // Derivative term
        float derivative = (error - _pre_error) / _dt;
        float Dout = _Kd * derivative;

        // Calculate total output
        float output = Pout + Iout + Dout;

        // // Restrict to max/min
        // if (output > _max)
        //     output = _max;
        // else if (output < _min)
        //     output = _min;

        // Save error to previous error
        _pre_error = error;

        return output;
    }
};
