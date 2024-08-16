#pragma once 

#include "UFO_I2C_driver.h"

#define UFO_I2C_BUFFER_SIZE ((int32_t)32)

class UFO_I2C_Interface 
{
protected:
    UFO_I2C_Driver* _driver;
    uint8_t _addr = 0;
public:
    UFO_I2C_Interface() {}
    // UFO_I2C_Interface(uint8_t addr) : _addr(addr) {}
    ~UFO_I2C_Interface() {}
    
    esp_err_t Init(uint8_t addr, UFO_I2C_Driver* driver){
        _driver = driver;
        _addr= addr;
        esp_err_t err = ESP_OK;
        if (!_driver->Initialized())
        {
            //set critical
            err = ESP_FAIL;
        }
        
        return err;
    }

    void SetAddress(uint8_t addr){
        _addr= addr;
    }
    int16_t Read16_Signed_LittleEndian(uint8_t reg)
    {
        return static_cast<int16_t>(Read16_LittleEndian(reg));
    }

    uint16_t Read16_LittleEndian(uint8_t reg)
    {
        uint16_t temp = Read16(reg);
        return (temp >> 8) | (temp << 8);
    }

    uint32_t Read24(uint8_t reg)
    {
        uint8_t buf[3];
        buf[0] = reg;
        esp_err_t ret = _driver->WriteRead(_addr, buf, 1, buf, 3);

        return uint32_t(buf[0]) << 16 | uint32_t(buf[1]) << 8 | uint32_t(buf[2]);
    }

    uint16_t Read16( byte reg)
    {
        uint8_t buf[2];
        buf[0] = reg;
        esp_err_t ret = _driver->WriteRead(_addr, buf, 1, buf, 2);
        return uint16_t(buf[0]) << 8 | uint16_t(buf[1]);
    }

    uint8_t Read8(uint8_t reg)
    {
        uint8_t buf[] = {reg};
        esp_err_t ret = _driver->WriteRead(_addr, buf, 1, buf, 1);
        return buf[0];
    }

    esp_err_t Read(uint8_t reg, uint8_t* rBuf , uint8_t size){
        uint8_t buf[1] = {reg};
        esp_err_t ret = _driver->WriteRead(_addr, buf, 1, rBuf, size);
        return ret;
    }

    esp_err_t Write8(uint8_t reg, uint8_t val)
    {
        uint8_t buf[] = {reg, val};
        return _driver->Write(_addr, buf, 2);
    }
};