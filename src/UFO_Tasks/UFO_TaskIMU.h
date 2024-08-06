#pragma once

#include "UFO_DataStorage.h"
#include "UFO_TaskMinimal.h"
#include "FastIMU.h"
#include "../UFO_math/Madgwick.h"


#define IMU_ADDRESS 0x68    //Change to the address of the IMU
#define PERFORM_CALIBRATION //Comment to disable startup calibration
MPU6050 IMU;               //Change to the name of any supported IMU!
calData calib = { 0 };  //Calibration data
AccelData IMUAccel;    //Sensor data
GyroData IMUGyro;
MagData IMUMag;
Madgwick filter;



// WARNING!!!
//  ESP_DRAM_LOGx for interraption (maybe for IRAM_ATTR also )
void /*IRAM_ATTR*/ UFO_Task_IMU(void *arg)
{
    UFO_SendToFunckMinimal *data = (UFO_SendToFunckMinimal *)arg;
    delay(10);
    Serial.print("Task created: ");
    Serial.println(data->name);
    Serial.print("\tupdate time:");
    Serial.println(data->updTime);
    delay(10);

    delay(100);
    int err = IMU.init(calib, IMU_ADDRESS);
    if (err != 0)
    {
        Serial.print("Error initializing IMU: ");
        Serial.println(err);
        while (true)
        {
            ;
        }
    }
    // err = IMU.setAccelRange(4);
    err = IMU.setGyroRange(2000);
    if (err != 0)
    {
        Serial.print("Error Setting range: ");
        Serial.println(err);
        while (true)
        {
            ;
        }
    }

#ifdef PERFORM_CALIBRATION
    Serial.println("FastIMU Calibrated Quaternion example");
    if (IMU.hasMagnetometer())
    {
        delay(1000);
        Serial.println("Move IMU in figure 8 pattern until done.");
        delay(3000);
        IMU.calibrateMag(&calib);
        Serial.println("Magnetic calibration done!");
    }
    else
    {
        delay(1000);
    }
    Serial.println("Keep IMU level.");
    delay(5000);
    IMU.calibrateAccelGyro(&calib);
    Serial.println("Calibration done!");
    Serial.println("Accel biases X/Y/Z: ");
    Serial.print(calib.accelBias[0]);
    Serial.print(", ");
    Serial.print(calib.accelBias[1]);
    Serial.print(", ");
    Serial.println(calib.accelBias[2]);
    Serial.println("Gyro biases X/Y/Z: ");
    Serial.print(calib.gyroBias[0]);
    Serial.print(", ");
    Serial.print(calib.gyroBias[1]);
    Serial.print(", ");
    Serial.println(calib.gyroBias[2]);
    
    delay(5000);
    IMU.init(calib, IMU_ADDRESS);

    filter.begin(0.2f);
#endif
    delay(100);

    while (true)
    {
        // if (!UFO_IMUData.locked)
        // {
            
        //     Serial.print(">yaw:");
        //     // Serial.println(yaw);
        //     Serial.print(">pitch:");
        //     // Serial.println(pitch);
        //     Serial.print(">roll:");
        //     // Serial.println(roll);
        // }
        IMU.update();
        IMU.getAccel(&IMUAccel);
        IMU.getGyro(&IMUGyro);
        filter.updateIMU(IMUGyro.gyroX, IMUGyro.gyroY, IMUGyro.gyroZ, IMUAccel.accelX, IMUAccel.accelY, IMUAccel.accelZ);
        float
            q0 = filter.getQuatW(),
            q1 = filter.getQuatX(),
            q2 = filter.getQuatY(),
            q3 = filter.getQuatZ();
        float roll = atan2f(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2);
        float pitch = asinf(-2.0f * (q1 * q3 - q0 * q2));
        float yaw = atan2f(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3);

        Serial.print(">yaw:");
        Serial.println(yaw* 180 / M_PI);
        Serial.print(">pitch:");
        Serial.println(pitch* 180 / M_PI);
        Serial.print(">roll:");
        Serial.println(roll* 180 / M_PI);

        delay(data->updTime); // (this delay is neccesary for yeild)
    }

    Serial.print("Task terminated: ");
    Serial.println(data->name);

    delete data;
    vTaskDelete(NULL);
}