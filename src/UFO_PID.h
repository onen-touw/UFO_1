#pragma once


class UFO_PID
{

private:
    double _dt;
    double _Kp;
    double _Kd;
    double _Ki;
    double _pre_error;
    double _integral;

public:
    // Kp -  proportional gain
    // Ki -  Integral gain
    // Kd -  derivative gain
    // dt -  loop interval time
    UFO_PID(double dt, double Kp, double Kd, double Ki) : _dt(dt),
                                                                                  _Kp(Kp),
                                                                                  _Kd(Kd),
                                                                                  _Ki(Ki),
                                                                                  _pre_error(0),
                                                                                  _integral(0)
    {
    }
    ~UFO_PID() {}

    // Returns the manipulated variable given a setpoint and current process value
    double Calculate(double setpoint, double pv)
    {

        // Calculate error
        double error = setpoint - pv;

        // Proportional term
        double Pout = _Kp * error;

        // Integral term
        _integral += error * _dt;
        double Iout = _Ki * _integral;

        // Derivative term
        double derivative = (error - _pre_error) / _dt;
        double Dout = _Kd * derivative;

        // Calculate total output
        double output = Pout + Iout + Dout;

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
