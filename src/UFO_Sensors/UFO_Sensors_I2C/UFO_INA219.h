#pragma once

#include "../../UFO_Config.h"
#include "../../UFO_Driver/UFO_I2C_Interface.h"


#define UFO_INA219_ADDRESS 0x40



class UFO_INA219 : private UFO_I2C_Interface
{
private:
    /* data */
    enum : uint8_t
    {
        INA_REG_CONFIG = 0x00,
        INA_REG_SHUNTVOLT = 0x01,
        INA_REG_BUSVOLT = 0x02,
        INA_REG_POWER = 0x03,
        INA_REG_CURRENT = 0x04,
        INA_REG_CALIB = 0x05,
    };

    enum INA219_VRange : uint8_t
    {
        INA_BUSBOLT_RANGE_16V = 0b0,
        INA_BUSBOLT_RANGE_32V = 0b1,
    };
    enum INA219_Gain : uint8_t
    {
        INA_GAIN_1_40MV = 0b00,  // Gain 1, 40mV Range
        INA_GAIN_2_80MV = 0b01,  // Gain 2, 80mV Range
        INA_GAIN_4_160MV = 0b10, // Gain 4, 160mV Range
        INA_GAIN_8_320MV = 0b11, // Gain 8, 320mV Range
    };

    enum INA219_Resolution : uint8_t
    {
        INA_SHUNT_ADC_RESOLUT_9BIT = 0b0000,             // 9-bit bus res = 0..511
        INA_SHUNT_ADC_RESOLUT_10BIT = 0b0001,            // 10-bit bus res = 0..1023
        INA_SHUNT_ADC_RESOLUT_11BIT = 0b0010,            // 11-bit bus res = 0..2047
        INA_SHUNT_ADC_RESOLUT_12BIT = 0b0011,            // 12-bit bus res = 0..4097
        INA_SHUNT_ADC_RESOLUT_12BIT_2S_1060US = 0b1001,  // 2 x 12-bit bus samples averaged together
        INA_SHUNT_ADC_RESOLUT_12BIT_4S_2130US = 0b1010,  // 4 x 12-bit bus samples averaged together
        INA_SHUNT_ADC_RESOLUT_12BIT_8S_4260US = 0b1011,  // 8 x 12-bit bus samples averaged together
        INA_SHUNT_ADC_RESOLUT_12BIT_16S_8510US = 0b1100, // 16 x 12-bit bus samples averaged together
        INA_SHUNT_ADC_RESOLUT_12BIT_32S_17MS = 0b1101,   // 32 x 12-bit bus samples averaged together
        INA_SHUNT_ADC_RESOLUT_12BIT_64S_34MS = 0b1110,   // 64 x 12-bit bus samples averaged together
        INA_SHUNT_ADC_RESOLUT_12BIT_128S_69MS = 0b1111,  // 128 x 12-bit bus samples averaged together
    };

    enum INA219_Mode : uint8_t
    {
        INA_MODE_POWERDOWN = 0x000,            /**< power down */
        INA_MODE_SVOLT_TRIGGERED = 0b001,      /**< shunt voltage triggered */
        INA_MODE_BVOLT_TRIGGERED = 0b010,      /**< bus voltage triggered */
        INA_MODE_SANDBVOLT_TRIGGERED = 0b011,  /**< shunt and bus voltage triggered */
        INA_MODE_ADCOFF = 0b100,               /**< ADC off */
        INA_MODE_SVOLT_CONTINUOUS = 0b101,     /**< shunt voltage continuous */
        INA_MODE_BVOLT_CONTINUOUS = 0b110,     /**< bus voltage continuous */
        INA_MODE_SANDBVOLT_CONTINUOUS = 0b111, /**< shunt and bus voltage continuous */
    };

    INA219_Mode _mode;
    INA219_Resolution _busADC;
    INA219_Resolution _shuntADC;
    INA219_Gain _pga_shuntVolt;
    INA219_VRange _busVoltRange;


public:
    UFO_INA219(UFO_I2C_Driver* driver) {

        esp_err_t err = this->Init(UFO_INA219_ADDRESS, driver);
        if (err != ESP_OK)
        {
            // SetCritical;      //think about it
            Serial.println("driver initializing priblem");
        }
    }

    void InitSensor()
    {
        // check if sensor, i.e. the chip ID is correct
        // int32_t _sensorID = this->Read8(BME280_REGISTER_CHIPID);
        // Serial.println(_sensorID);
        // if (_sensorID != 0x60){
        //     return false;
        // }

        //default
        SetConfig(
            INA_MODE_SANDBVOLT_CONTINUOUS,
            INA_SHUNT_ADC_RESOLUT_12BIT,
            INA_SHUNT_ADC_RESOLUT_12BIT,
            INA_GAIN_8_320MV,
            INA_BUSBOLT_RANGE_32V
            );
        Calibrate32v1A();

    }
    
    uint16_t SetConfig(INA219_Mode mode, INA219_Resolution busADC, INA219_Resolution shuntADC, INA219_Gain pga_shuntVolt, INA219_VRange busVoltRange) {
        _mode = mode;
        _busADC = busADC;
        _shuntADC = shuntADC;
        _pga_shuntVolt = pga_shuntVolt;
        _busVoltRange = busVoltRange;

        uint16_t res = 0b0<<15;
        res |= (_busVoltRange << 13);
        res |= (_pga_shuntVolt << 11);
        res |= (_busADC << 7);
        res |= (_shuntADC << 3);
        res |= _mode;
        this->Write(INA_REG_CONFIG, res, 2, BIT_ORDER_MSB);
    }

    void Calibrate32v1A(){
        uint32_t ina219_calValue = 10240;

        ina219_currentDivider_mA = 25;    // Current LSB = 40uA per bit (1000/40 = 25)
        ina219_powerMultiplier_mW = 0.8f; // Power LSB = 800uW per bit

        this->Write(INA_REG_CALIB, ina219_calValue, 2, BIT_ORDER_MSB);
        
    }

    // void Calibrate(float vShuntMax, float rShunt, float expectedCurrentMax){
    //     float vBusMax = 0;
    //     if (_mode == INA_BUSBOLT_RANGE_32V)
    //     {
    //         vBusMax = 32;
    //     }
    //     else
    //     {
    //         vBusMax = 16;
    //     }
    //     float MaxPossible_I = vShuntMax/rShunt;
    //     float Minimum_LSB = expectedCurrentMax/32767;
    //     float Maximum_LSB = expectedCurrentMax/4096;
    //     float CurrentLSB = (Maximum_LSB-Minimum_LSB)*0.1 + Minimum_LSB;

    //     float Cal = trunc(0.04096 / (CurrentLSB * rShunt));
    //     float Power_LSB = 20 * CurrentLSB;
    //     float Max_Current = CurrentLSB *32767;

    //     float MaxCurrentBeforeOverflow = 0.f;
    //     if (Max_Current >= MaxPossible_I)
    //     {
    //         MaxCurrentBeforeOverflow = MaxPossible_I;
    //     }
    //     else {
    //         MaxCurrentBeforeOverflow = Max_Current;
    //     }
        
    //     float MaxShuntVoltage = MaxCurrentBeforeOverflow * rShunt;
    //     float MaxShuntBeforeOverflow= 0.f;

    //     if (MaxShuntVoltage >= vShuntMax)
    //     {
    //         MaxShuntBeforeOverflow = vShuntMax;
    //     }
    //     else 
    //     {
    //         MaxShuntBeforeOverflow = MaxShuntVoltage;
    //     }
        
    //     float MaxPower = MaxCurrentBeforeOverflow*vBusMax;


    // }

    ~UFO_INA219() {}


private:
    void __Reset(){
    }
    
};