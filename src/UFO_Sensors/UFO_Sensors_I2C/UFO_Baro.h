#pragma once

#include "../../UFO_Config.h"
#include "../../UFO_Driver/UFO_I2C_Interface.h"

#define UFO_BME280_ADDRESS ((uint8_t)0x76) // Primary I2C Address

// now only bme280

struct UFO_BaroData_t
{
    float Tempreture = 0;
    float Presure = 0;
    float Altitude = 0;
};

class UFO_Baro : private UFO_I2C_Interface
{
private:
    enum : uint8_t
    {
        BME280_REGISTER_DIG_T1 = 0x88,
        BME280_REGISTER_DIG_T2 = 0x8A,
        BME280_REGISTER_DIG_T3 = 0x8C,

        BME280_REGISTER_DIG_P1 = 0x8E,
        BME280_REGISTER_DIG_P2 = 0x90,
        BME280_REGISTER_DIG_P3 = 0x92,
        BME280_REGISTER_DIG_P4 = 0x94,
        BME280_REGISTER_DIG_P5 = 0x96,
        BME280_REGISTER_DIG_P6 = 0x98,
        BME280_REGISTER_DIG_P7 = 0x9A,
        BME280_REGISTER_DIG_P8 = 0x9C,
        BME280_REGISTER_DIG_P9 = 0x9E,

        BME280_REGISTER_DIG_H1 = 0xA1,
        BME280_REGISTER_DIG_H2 = 0xE1,
        BME280_REGISTER_DIG_H3 = 0xE3,
        BME280_REGISTER_DIG_H4 = 0xE4,
        BME280_REGISTER_DIG_H5 = 0xE5,
        BME280_REGISTER_DIG_H6 = 0xE7,

        BME280_REGISTER_CHIPID = 0xD0,
        BME280_REGISTER_VERSION = 0xD1,
        BME280_REGISTER_SOFTRESET = 0xE0,

        BME280_REGISTER_CAL26 = 0xE1, // R calibration stored in 0xE1-0xF0

        BME280_REGISTER_CONTROLHUMID = 0xF2,
        BME280_REGISTER_STATUS = 0XF3,
        BME280_REGISTER_CONTROL = 0xF4,
        BME280_REGISTER_CONFIG = 0xF5,
        BME280_REGISTER_PRESSUREDATA = 0xF7,
        BME280_REGISTER_TEMPDATA = 0xFA,
        BME280_REGISTER_HUMIDDATA = 0xFD
    } _reg;
    struct
    {
        uint16_t dig_T1; ///< temperature compensation value
        int16_t dig_T2;  ///< temperature compensation value
        int16_t dig_T3;  ///< temperature compensation value

        uint16_t dig_P1; ///< pressure compensation value
        int16_t dig_P2;  ///< pressure compensation value
        int16_t dig_P3;  ///< pressure compensation value
        int16_t dig_P4;  ///< pressure compensation value
        int16_t dig_P5;  ///< pressure compensation value
        int16_t dig_P6;  ///< pressure compensation value
        int16_t dig_P7;  ///< pressure compensation value
        int16_t dig_P8;  ///< pressure compensation value
        int16_t dig_P9;  ///< pressure compensation value

        uint8_t dig_H1; ///< humidity compensation value
        int16_t dig_H2; ///< humidity compensation value
        uint8_t dig_H3; ///< humidity compensation value
        int16_t dig_H4; ///< humidity compensation value
        int16_t dig_H5; ///< humidity compensation value
        int8_t dig_H6;  ///< humidity compensation value
    } _calib;

    enum Sampling :uint8_t
    {
        SAMPLING_NONE = 0b000,
        SAMPLING_X1 = 0b001,
        SAMPLING_X2 = 0b010,
        SAMPLING_X4 = 0b011,
        SAMPLING_X8 = 0b100,
        SAMPLING_X16 = 0b101
    };
    enum Mode:uint8_t
    {
        MODE_SLEEP = 0b00,
        MODE_FORCED = 0b01,
        MODE_NORMAL = 0b11
    };
    enum Filter:uint8_t
    {
        FILTER_OFF = 0b000,
        FILTER_X2 = 0b001,
        FILTER_X4 = 0b010,
        FILTER_X8 = 0b011,
        FILTER_X16 = 0b100
    };
	enum Duration:uint8_t
	{
		STANDBY_MS_0_5 = 0b000,
		STANDBY_MS_10 = 0b110,
		STANDBY_MS_20 = 0b111,
		STANDBY_MS_62_5 = 0b001,
		STANDBY_MS_125 = 0b010,
		STANDBY_MS_250 = 0b011,
		STANDBY_MS_500 = 0b100,
		STANDBY_MS_1000 = 0b101
	};

private:

	int32_t t_fine;	   //!< temperature with high resolution
	float _seaLevelPre = 0;
	int32_t t_fine_adjust = 0; //!< add to compensate temp readings
    UFO_BaroData_t _data;

public:
    UFO_Baro(UFO_I2C_Driver* driver) {
        esp_err_t err = this->Init(UFO_BME280_ADDRESS, driver);
        if (err!= ESP_OK)
        {
            //SetCritical;      //think about it
            Serial.println("driver initializing priblem");
        }
        
    }
    ~UFO_Baro() {}

    //todo:: errors
    void InitSensor()
    {
        // check if sensor, i.e. the chip ID is correct
        // int32_t _sensorID = this->Read8(BME280_REGISTER_CHIPID);
        // Serial.println(_sensorID);
        // if (_sensorID != 0x60){
        //     return false;
        // }

        // reset the device using soft-reset
        // this makes sure the IIR is off, etc.
        this->Write8(BME280_REGISTER_SOFTRESET, 0xB6);

        // wait for chip to wake up.
        delay(10);

        // if chip is still reading calibration, delay
        while (__IsReadingCalibration())
        {
            delay(10);
            Serial.print(".");
        }
        Serial.println();
        __ReadCoefficients(); // read trimming parameters, see DS 4.2.2

        SetSampling(); // use defaults

        delay(100);
    }

    void SetSampling(
        Mode mode = MODE_NORMAL,
        Sampling tempSampling = SAMPLING_X16,
        Sampling pressSampling = SAMPLING_X16,
        Filter filter = FILTER_X4,
        Duration duration = STANDBY_MS_0_5)
    {
        // making sure sensor is in sleep mode before setting configuration
        // as it otherwise may be ignored
        this->Write8(BME280_REGISTER_CONTROL, MODE_SLEEP);

        // you must make sure to also set REGISTER_CONTROL after setting the
        // CONTROLHUMID register, otherwise the values won't be applied (see
        // DS 5.4.3)
        uint8_t conf = SAMPLING_X1;
        this->Write8(BME280_REGISTER_CONTROLHUMID, conf);


        conf = (duration << 5) | (filter << 2) | 0;
        this->Write8(BME280_REGISTER_CONFIG, conf);


        conf = (tempSampling << 5) | (pressSampling << 2) | mode;
        this->Write8(BME280_REGISTER_CONTROL, conf);
}



float GetTemperature()
{
	int32_t var1, var2;

	int32_t adc_T = this->Read24(BME280_REGISTER_TEMPDATA);
	if (adc_T == 0x800000) // value in case temp measurement was disabled
		return NAN;
	adc_T >>= 4;

	var1 = (int32_t)((adc_T / 8) - ((int32_t)_calib.dig_T1 * 2));
	var1 = (var1 * ((int32_t)_calib.dig_T2)) / 2048;
	var2 = (int32_t)((adc_T / 16) - ((int32_t)_calib.dig_T1));
	var2 = (((var2 * var2) / 4096) * ((int32_t)_calib.dig_T3)) / 16384;

	t_fine = var1 + var2 + t_fine_adjust;
	int32_t T = (t_fine * 5 + 128) / 256;
    _data.Tempreture = static_cast<float>(T) / 100; 
	return _data.Tempreture;
}
float GetPressure()
{
	int64_t var1, var2, var3, var4;

	GetTemperature(); // must be done first to get t_fine

	int32_t adc_P = this->Read24(BME280_REGISTER_PRESSUREDATA);
	if (adc_P == 0x800000) // value in case pressure measurement was disabled
		return NAN;
	adc_P >>= 4;

	var1 = static_cast<int64_t>(t_fine) - 128000;
	var2 = var1 * var1 * static_cast<int64_t>(_calib.dig_P6);
	var2 = var2 + ((var1 * static_cast<int64_t>(_calib.dig_P5)) * 131072);
	var2 = var2 + (static_cast<int64_t>(_calib.dig_P4) * 34359738368);
	var1 = ((var1 * var1 * static_cast<int64_t>(_calib.dig_P3)) / 256) +
		   ((var1 * static_cast<int64_t>(_calib.dig_P2) * 4096));
	var3 = ((int64_t)1) * 140737488355328;
	var1 = (var3 + var1) * static_cast<int64_t>(_calib.dig_P1) / 8589934592;

	if (var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}

	var4 = 1048576 - adc_P;
	var4 = (((var4 * 2147483648) - var2) * 3125) / var1;
	var1 = (static_cast<int64_t>(_calib.dig_P9) * (var4 / 8192) * (var4 / 8192)) /
		   33554432;
	var2 = (static_cast<int64_t>(_calib.dig_P8) * var4) / 524288;
	var4 = ((var4 + var1 + var2) / 256) + (static_cast<int64_t>(_calib.dig_P7) * 16);

	_data.Presure = var4 / 256.0;
	return _data.Presure;
}
float GetAltitude(float seaLevel)
{
	// Equation taken from BMP180 datasheet (page 16):
	//  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf
	float atmospheric = GetPressure() / 100.0f; 
    _data.Altitude = 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903));
	return _data.Altitude;
}
float GetSeaLevelForAltitude(float altitude, float atmospheric)
{
	// Equation taken from BMP180 datasheet (page 17):
	//  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf
	return atmospheric / pow(1.0 - (altitude / 44330.0), 5.255);
}
void SetTemperatureCompensation(float adjustment)
{
	// convert the value in C into and adjustment to t_fine
	t_fine_adjust = ((static_cast<int32_t>(adjustment * 100) << 8)) / 5;
};

void operator ()() {
    GetAltitude(1013.25F); // 1013.25F
}

void Update(){
    GetAltitude(1013.25F); // 1013.25F
}

const UFO_BaroData_t& Get() const {
    return _data;
}

private:
void __ReadCoefficients()
{   
	_calib.dig_T1 = this->Read16_LittleEndian(BME280_REGISTER_DIG_T1);   
	_calib.dig_T2 = this->Read16_Signed_LittleEndian(BME280_REGISTER_DIG_T2);   
	_calib.dig_T3 = this->Read16_Signed_LittleEndian(BME280_REGISTER_DIG_T3);   

	_calib.dig_P1 = this->Read16_LittleEndian(BME280_REGISTER_DIG_P1);
	_calib.dig_P2 = this->Read16_Signed_LittleEndian(BME280_REGISTER_DIG_P2);
	_calib.dig_P3 = this->Read16_Signed_LittleEndian(BME280_REGISTER_DIG_P3);
	_calib.dig_P4 = this->Read16_Signed_LittleEndian(BME280_REGISTER_DIG_P4);
	_calib.dig_P5 = this->Read16_Signed_LittleEndian(BME280_REGISTER_DIG_P5);
	_calib.dig_P6 = this->Read16_Signed_LittleEndian(BME280_REGISTER_DIG_P6);
	_calib.dig_P7 = this->Read16_Signed_LittleEndian(BME280_REGISTER_DIG_P7);
	_calib.dig_P8 = this->Read16_Signed_LittleEndian(BME280_REGISTER_DIG_P8);
	_calib.dig_P9 = this->Read16_Signed_LittleEndian(BME280_REGISTER_DIG_P9);

	_calib.dig_H1 = this->Read8(BME280_REGISTER_DIG_H1);
	_calib.dig_H2 = this->Read16_Signed_LittleEndian(BME280_REGISTER_DIG_H2);
	_calib.dig_H3 = this->Read8(BME280_REGISTER_DIG_H3);
	_calib.dig_H4 = (static_cast<int8_t>(this->Read8(BME280_REGISTER_DIG_H4)) << 4) |
						   (this->Read8(BME280_REGISTER_DIG_H4 + 1) & 0xF);
	_calib.dig_H5 = (static_cast<int8_t>(this->Read8(BME280_REGISTER_DIG_H5 + 1)) << 4) |
						   (this->Read8(BME280_REGISTER_DIG_H5) >> 4);
	_calib.dig_H6 = static_cast<int8_t>(this->Read8(BME280_REGISTER_DIG_H6));
}

bool __IsReadingCalibration()
{
	uint8_t const rStatus = this->Read8(BME280_REGISTER_STATUS);
	return (rStatus & (1 << 0)) != 0;
}

};
