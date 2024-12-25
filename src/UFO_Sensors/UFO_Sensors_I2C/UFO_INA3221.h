#pragma once

#include "../../UFO_Config.h"
#include "../../UFO_Driver/UFO_I2C_Interface.h"
#include <bitset>
#define UFO_INA_3221_ADDR_BASIC 0x40

// Channels
enum class UFO_INA3221_CHANNEL : uint8_t{
    INA3221_CH1 = 0,
    INA3221_CH2,
    INA3221_CH3,
    INA3221_CH_NUM
};
 enum class UFO_INA3221_AvgSamples: uint8_t{
    INA3221_REG_CONF_AVG_1 = 0,
    INA3221_REG_CONF_AVG_4,
    INA3221_REG_CONF_AVG_16,
    INA3221_REG_CONF_AVG_64,
    INA3221_REG_CONF_AVG_128,
    INA3221_REG_CONF_AVG_256,
    INA3221_REG_CONF_AVG_512,
    INA3221_REG_CONF_AVG_1024
};

struct UFO_INA3221_ChannelData {
    uint16_t _raw = 0;
    float _current = 0.f;
    float _busVolt = 0.f;
    float _shuntVolt = 0.f;
    float _power = 0.f;
    uint32_t _timestamp = 0;
};

struct UFO_INA3221_ChannelDataPrivate {
    // Shunt resistance in mOhm
    float _shuntRes = 1.5f;
    // Series filter resistance in Ohm
    float _filterRes = 0.f;
    bool _enable = true;
};

struct UFO_INA3221_Channel
{
    UFO_INA3221_ChannelData _data;
    UFO_INA3221_ChannelDataPrivate _private;
};

class UFO_INA3221 : private UFO_I2C_Interface 
{
private:
    // Shunt resistance in mOhm
    // uint32_t _shuntRes[static_cast<size_t>(UFO_INA3221_CHANNEL::INA3221_CH_NUM)];
    // Series filter resistance in Ohm
    // uint32_t _filterRes[static_cast<size_t>(UFO_INA3221_CHANNEL::INA3221_CH_NUM)];
    // Value of Mask/Enable register.
    // masken_reg_t _masken_reg;

    UFO_INA3221_Channel _data[static_cast<uint32_t>(UFO_INA3221_CHANNEL::INA3221_CH_NUM)]={};

private:

 enum : uint8_t{
    INA3221_REG_CONF = 0,
    INA3221_REG_CH1_SHUNTV,
    INA3221_REG_CH1_BUSV,
    INA3221_REG_CH2_SHUNTV,
    INA3221_REG_CH2_BUSV,
    INA3221_REG_CH3_SHUNTV,
    INA3221_REG_CH3_BUSV,
    INA3221_REG_CH1_CRIT_ALERT_LIM,
    INA3221_REG_CH1_WARNING_ALERT_LIM,
    INA3221_REG_CH2_CRIT_ALERT_LIM,
    INA3221_REG_CH2_WARNING_ALERT_LIM,
    INA3221_REG_CH3_CRIT_ALERT_LIM,
    INA3221_REG_CH3_WARNING_ALERT_LIM,
    INA3221_REG_SHUNTV_SUM,
    INA3221_REG_SHUNTV_SUM_LIM,
    INA3221_REG_MASK_ENABLE,
    INA3221_REG_PWR_VALID_HI_LIM,
    INA3221_REG_PWR_VALID_LO_LIM,
    INA3221_REG_MANUF_ID = 0xFE,
    INA3221_REG_DIE_ID   = 0xFF
};

// Conversion times
 enum : uint8_t{
    INA3221_REG_CONF_CT_140US = 0,
    INA3221_REG_CONF_CT_204US,
    INA3221_REG_CONF_CT_332US,
    INA3221_REG_CONF_CT_588US,
    INA3221_REG_CONF_CT_1100US,
    INA3221_REG_CONF_CT_2116US,
    INA3221_REG_CONF_CT_4156US,
    INA3221_REG_CONF_CT_8244US
};

// Averaging modes
 enum : uint8_t{
    INA3221_REG_CONF_AVG_1 = 0,
    INA3221_REG_CONF_AVG_4,
    INA3221_REG_CONF_AVG_16,
    INA3221_REG_CONF_AVG_64,
    INA3221_REG_CONF_AVG_128,
    INA3221_REG_CONF_AVG_256,
    INA3221_REG_CONF_AVG_512,
    INA3221_REG_CONF_AVG_1024
};

public:
    explicit UFO_INA3221(UFO_I2C_Driver *driver)
    {
        esp_err_t err = this->Init(UFO_INA_3221_ADDR_BASIC, driver);
        if (err != ESP_OK)
        {
            // SetCritical;      //think about it
            Serial.println("driver initializing problem");
        }
    }
    ~UFO_INA3221() {}

    void InitSensor(){
        enum {ch1,ch2, ch3};
        _data[ch1]._private._filterRes = 0;
        _data[ch2]._private._filterRes = 0;
        _data[ch3]._private._filterRes = 0;

        _data[ch1]._private._shuntRes = 1.5 ;//1500000
        _data[ch2]._private._shuntRes = 1.5;
        _data[ch3]._private._shuntRes = 1.5;

        __ResetChip();

        uint16_t id = this->Read16(INA3221_REG_DIE_ID);
        Serial.print("INA3221 die id: ");
        Serial.println(id);
        id = this->Read16(INA3221_REG_MANUF_ID);
        Serial.print("INA3221 man id: ");
        Serial.println(id);
    }

    void Update()
    {
        //for clear time meas
        for (uint8_t i = 0; i < static_cast<uint32_t>(UFO_INA3221_CHANNEL::INA3221_CH_NUM); ++i)
        {
            if (_data[i]._private._enable)
            {
                _data[i]._data._shuntVolt = __GetShuntVoltage(i);
                _data[i]._data._busVolt = __GetVoltage(i);
                _data[i]._data._timestamp = millis();
            }
        }
        for (size_t i = 0; i < static_cast<uint32_t>(UFO_INA3221_CHANNEL::INA3221_CH_NUM); ++i)
        {
            _data[i]._data._busVolt /= 1000.f;
            _data[i]._data._current = _data[i]._data._shuntVolt / _data[i]._private._shuntRes /*  / 1000.f *1000.f  */;
        }
        
    }
    void SetConfig(bool chan1 = false,bool chan2 = false,bool chan3 = false)
    {
        uint16_t conf = 0b0;
    }
    // with current compensation
    void Update(int)
    {
        for (uint8_t i = 0; i < static_cast<uint32_t>(UFO_INA3221_CHANNEL::INA3221_CH_NUM); ++i)
        {
            if (_data[i]._private._enable)
            {
                int32_t shunt_uV = __GetShuntVoltage(i);
                int32_t _busVolt = __GetVoltage(i);

                _data[i]._data._shuntVolt = shunt_uV;
                _data[i]._data._busVolt = _busVolt;

                float bias_in = 10.0f;                                   // Input bias current at IN– in uA
                float r_in = 0.670f;                                     // Input resistance at IN– in MOhm
                uint32_t adc_step = 40;                                  // smallest shunt ADC step in uV
                float shunt_res = _data[i]._private._shuntRes / 1000.0f; // convert to Ohm
                float filter_res = _data[i]._private._filterRes;
                int32_t offset = 0.0f;
                float reminder;

                offset = (shunt_res + filter_res) * (_busVolt / r_in + bias_in) - bias_in * filter_res;

                // Round the offset to the closest shunt ADC value
                reminder = offset % adc_step;
                if (reminder < adc_step / 2)
                {
                    offset -= reminder;
                }
                else
                {
                    offset += adc_step - reminder;
                }
                _data[i]._data._current = (shunt_uV - offset) / shunt_res;
            }
        }
    }
    void operator()()
    {
        Update();
    }
    const UFO_INA3221_ChannelData& Get(uint8_t chan ) const
    {
        return _data[chan]._data;
    }



private:
    int32_t __GetShuntVoltage(uint8_t chReg)
    {
        int16_t val_raw = 0;
        int32_t result = 0;
        uint8_t reg = 0;
        // check it
        switch (chReg)
        {
        case 0:
            reg = INA3221_REG_CH1_SHUNTV;
            break;
        case 1:
            reg = INA3221_REG_CH2_SHUNTV;
            break;
        case 2:
            reg = INA3221_REG_CH3_SHUNTV;
            break;
        }
        _data->_data._raw = this->Read16(reg);
        val_raw = static_cast<int16_t>(_data->_data._raw);

        // val_raw = static_cast<int16_t>(this->Read16(reg));
        // _data->_data._raw = val_raw;
        // Serial.print(val_raw);
        // Serial.print(" >> ");
        result = static_cast<int32_t>((val_raw) >> 3) * 40; // 40 is LSB see DS
        // Serial.println(result);
        return result;
    }

    int32_t __GetVoltage(uint8_t chReg)
    {
        int32_t result = 0.f;
        int16_t vRaw = 0;

        switch (chReg) {
        case 0:
            chReg = INA3221_REG_CH1_BUSV;
            break;
        case 1:
            chReg = INA3221_REG_CH2_BUSV;
            break;
        default:
            chReg = INA3221_REG_CH3_BUSV;
            break;
    }
        vRaw = static_cast<int16_t>(this->Read16(chReg));
        result = static_cast<int32_t>((static_cast<int16_t>(vRaw) >> 3)) * 8; // 8 is LSB
        return result;
    }

    void __ResetChip()
    {
        uint16_t v = 1 << 15;
        this->Write16(INA3221_REG_CONF, v); // Write a one to bit 7 reset bit; toggle reset device
    }
};