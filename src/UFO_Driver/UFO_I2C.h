#pragma once
#include "UFO_Config.h"

// I2C matrix
// #define UFO_I2C_WARE_TYPES 2
// static UFO_I2C_PIN_MATRIX[scl_hardware] = [ [sda_hardware, scl_hardware]  [sda_software scl_software]  ]



class UFO_I2C_Driver
{
private:
    uint8_t _addr = 0;

public:
    UFO_I2C_Driver(/* args */) {}
    ~UFO_I2C_Driver() {}


    
};