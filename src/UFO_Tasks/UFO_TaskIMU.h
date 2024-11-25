#pragma once

#include "UFO_DataStorage.h"
#include "UFO_TaskMinimal.h"
// #include "FastIMU.h"
#include "../UFO_math/Madgwick.h"
#include "../UFO_Sensors/UFO_Sensors_I2C/UFO_IMU.h"
#include "../UFO_math/UFO_FiltersInclude.h"

void /*IRAM_ATTR*/ UFO_Task_IMU(void *arg)
{
    UFO_SendToFunckMinimal *data = (UFO_SendToFunckMinimal *)arg;
    UFO_I2C_Driver* driver = reinterpret_cast<UFO_I2C_Driver*>(data->arg);
    delay(10);
    Serial.print("Task created: ");
    Serial.println(data->name);
    Serial.print("\tupdate time:");
    Serial.println(data->updTime);
    delay(10);
    
    UFO_IMU imu(driver);            //set reinterpret_cast<UFO_I2C_Driver*>(data->arg) instead driver
    Madgwick filter;
    UFO_IMU_Data imuData;
    // UFO_AdaptiveFilter adFilter;
    UFO_KalmanFilter adFilter;

    adFilter.Set(0.5, 0.5, 0.8);
    filter.begin(0.2f);
    // adFilter.Set(0.1f, 0.5f, 0.3f);
    imu.InitSensor();
    UFO_IMU_CalibrationData dataCal;
    Serial.println("Keep IMU level.");
    delay(5000);
    imu.Calibrate();
    dataCal = imu.GetOffsets();
    Serial.println("Calibration done!");
    Serial.println("Accel biases X/Y/Z: ");
    Serial.print(dataCal._accelOffset._x);
    Serial.print(", ");
    Serial.print(dataCal._accelOffset._y);
    Serial.print(", ");
    Serial.println(dataCal._accelOffset._z);
    Serial.println("Gyro biases X/Y/Z: ");
    Serial.print(dataCal._gyroOffset._x);
    Serial.print(", ");
    Serial.print(dataCal._accelOffset._y);
    Serial.print(", ");
    Serial.println(dataCal._accelOffset._z);
    delay(100);

    while (true)
    {
        imu.Update();
        imuData = imu.Get();
        filter.updateIMU(
            imuData._gyro._x,
        imuData._gyro._y,
        imuData._gyro._z,
        imuData._accel._x,
        imuData._accel._y,
        imuData._accel._z);
        
        float
            q0 = filter.getQuatW(),
            q1 = filter.getQuatX(),
            q2 = filter.getQuatY(),
            q3 = filter.getQuatZ();
        float roll = atan2(0.5f - q1 * q1 - q2 * q2, q0 * q1 + q2 * q3);
        float pitch = asinf(-2.0f * (q1 * q3 - q0 * q2));
        float yaw = atan2f(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3);
        Serial.print(">rollFiltered:");
        Serial.println(adFilter(roll)* 180 / M_PI); // <-------------- filter
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
