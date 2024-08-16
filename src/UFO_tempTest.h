#pragma once
#include "Arduino.h"
#include "UFO_Driver/UFO_I2C_Interface.h"
#define QMC5883L_X_LSB		0x00
#define QMC5883L_X_MSB		0x01
#define QMC5883L_Y_LSB		0x02
#define QMC5883L_Y_MSB		0x03
#define QMC5883L_Z_LSB		0x04
#define QMC5883L_Z_MSB		0x05
#define QMC5883L_STATUS     0x06
#define QMC5883L_T_LSB		0x07
#define QMC5883L_T_MSB      0x08
#define QMC5883L_CTRL       0x09
#define QMC5883L_RESET1		0x0A
#define QMC5883L_RESET2		0x0B
#define QMC5883L_WHOAMI		0x0D

#define QMC5883L_WHOAMI_VALUE	0xFF
struct calData {
	bool valid;
	float accelBias[3];
	float gyroBias[3];
	float magBias[3];
	float magScale[3];
};
struct MagData {
	float magX;
	float magY;
	float magZ;
};

class QMC5883L {
public:
	QMC5883L() {};

	// Inherited via IMUBase
	int init( UFO_I2C_Driver* driver){
        i2c.Init(0xd, driver);
	//check sensor
	if (!(readByte(IMUAddress, QMC5883L_WHOAMI) == QMC5883L_WHOAMI_VALUE && readByte(IMUAddress, 0x0C) == 0x01)) {
		return -1;
	}
	//load cal
		calibration = { 0 };
		calibration.magScale[0] = 1.f;
		calibration.magScale[1] = 1.f;
		calibration.magScale[2] = 1.f;
	// Reset sensor.
	writeByte(IMUAddress, QMC5883L_RESET1, 0x80);
	delay(100);
	writeByte(IMUAddress, QMC5883L_RESET2, 0x01);
	delay(100);
	//start up sensor, 200hz ODR, 128 OSR, 8G, Continuous mode.
	writeByte(IMUAddress, QMC5883L_CTRL, 0x1D);
	return 0;
    }
	const MagData& Get() const {
		return mag;
	}
	void update() {
        if (!(readByte(IMUAddress, QMC5883L_STATUS) & 0x01)) {
		mag.magX = 0.f;
		mag.magY = 0.f;
		mag.magZ = 0.f;
		return;
	}
	uint8_t rawData[6] = { 0 };
	int16_t magCount[3] = { 0, 0, 0 };
	readBytes(IMUAddress, QMC5883L_X_LSB, 6, &rawData[0]);

	if (!(readByte(IMUAddress, QMC5883L_STATUS) & 0x02)) {                                           // Check if magnetic sensor overflow set, if not then report data
	magCount[0] = ((int16_t)rawData[1] << 8) | rawData[0];   // Turn the MSB and LSB into a signed 16-bit value
	magCount[1] = ((int16_t)rawData[3] << 8) | rawData[2];   // Data stored as little Endian
	magCount[2] = ((int16_t)rawData[5] << 8) | rawData[4];
	}

	// Calculate the mag value
	float mx, my, mz;
	mx = ((float)(magCount[0] * mRes - calibration.magBias[0]) * calibration.magScale[0]);
	my = ((float)(magCount[1] * mRes - calibration.magBias[1]) * calibration.magScale[1]);  // get actual magnetometer value, this depends on scale being set
	mz = ((float)(magCount[2] * mRes - calibration.magBias[2]) * calibration.magScale[2]);  //mul by 100 to convert from G to µT
	
	readBytes(IMUAddress, QMC5883L_T_LSB, 2, &rawData[0]);
	temperature = (float)((((int16_t)rawData[1] << 8) | rawData[0]) * tRes) + 20.f;
	switch (geometryIndex) {
	case 0:
		mag.magX = mx;
		mag.magY = my;
		mag.magZ = mz;
		break;
	case 1:
		mag.magX = -my;
		mag.magY = mx;
		mag.magZ = mz;
		break;
	case 2:
		mag.magX = mx;
		mag.magY = my;
		mag.magZ = mz;
		break;
	case 3:
		mag.magX = my;
		mag.magY = -mx;
		mag.magZ = mz;
		break;
	case 4:
		mag.magX = -mz;
		mag.magY = -my;
		mag.magZ = -mx;
		break;
	case 5:
		mag.magX = -mz;
		mag.magY = mx;
		mag.magZ = -my;
		break;
	case 6:
		mag.magX = -mz;
		mag.magY = my;
		mag.magZ = mx;
		break;
	case 7:
		mag.magX = -mz;
		mag.magY = -mx;
		mag.magZ = my;
		break;
	}
    }
	void calibrateMag(calData* cal) {}
	int setIMUGeometry(int index) { geometryIndex = index; return 0; };

private:
	float mRes = 10. * 819.2f / 32768.f;				//mRes value for full range (+-819.2 uT scaled * 10) readings (16 bit)
	float tRes = 100.f / 32768.f;			//tRes value for full range readings (16 bit)
	float temperature = 0.f;
	int geometryIndex = 0;
	UFO_I2C_Interface i2c;
	MagData mag = { 0 };

	calData calibration;
	uint8_t IMUAddress;

	void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
	{
		i2c.Write8(subAddress, data);
	}

	uint8_t readByte(uint8_t address, uint8_t subAddress)
	{
		return i2c.Read8(subAddress);                          // Return data read from slave register
	}

	void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* dest)
	{
        i2c.Read(subAddress, dest, count);
	}
};
