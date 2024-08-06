#pragma once

class UFO_TaskClassBase
{
private:
    /* data */
public:
    UFO_TaskClassBase(/* args */) {}
    ~UFO_TaskClassBase() {}
    virtual void Setup() = 0;           // change return class to bool;
    virtual void Iteration() = 0;       // change return class to exception;
};