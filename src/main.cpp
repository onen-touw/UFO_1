#include <Arduino.h>

#include "RxTxDataHandler.h"

#include "UFO_Tasks/UFO_Task.h"
// #include "UFO_Motors.h"
#include "UFO_Control.h"

#include "UFO_tempTest.h"
#include "UFO_Sensors/UFO_Sensors_I2C/UFO_Compass.h"

// #define TEST_MMM

UFO_Control control;
UFO_Motors motors;
// AsyncUDP udp;
// RxDataHandler rxDH;
// UFO_PDOA test_cls;

UFO_I2C_Driver driver;
#ifdef TEST_MMM
QMC5883L qmc;
MagData mag_data = {};
#else
UFO_Compass compass(&driver);
Vector3<float> comData = {};
#endif
void setup()
{
    delay(100);
    Serial.begin(115200);

    // Wire.begin();
    // Wire.setClock(400000); //400khz clock

    esp_err_t err = driver.Init(UFO_I2C_port::UFO_I2C_HARDWARE);
    Serial.println(err);
    Serial.print("inited ");
    Serial.println(driver.Initialized());


#ifdef TEST_MMM
    qmc.init(&driver);
#else
    compass.InitSensor();
#endif

#pragma region
    // test_cls.CreateItem(123);
    // test_cls.CreateItem(0.213f);
    // test_cls.CreateItem(true);
    // String txt = "string";
    // test_cls.CreateItem(txt);  
    // test_cls.CreatePacket();
    // String packet = test_cls.GetPacket();
    // for (size_t i = 0; i < packet.length(); i++)
    // {
    //     Serial.print(static_cast<uint8_t>(packet[i]));
    //     Serial.print("\t|");
    //     Serial.print(packet[i]);
    //     Serial.println("|");
    // }
    // Serial.println();
    delay(100);
    // control.Setup();
    delay(100);

    // rxDH.Addhandler([&](RX_INDEX i, int32_t v)
    //                 {
    // switch (i)
    // {
    // case RX_IND_THROTTLE:
    // if (abs(v - endData.throttle) > endData.error){
    //     endData.throttle = v;

    //         Serial.println(v);

    // }
    //     break;
    // default:
    //     break;
    // } });


    // if (!udp.listen(8080))
    // {
    //     Serial.println("err");
    // }
    // udp.onPacket(parsePacket);
#pragma endregion

    Serial.println("Start");
      for (uint8_t address = 1; address < 127; ++address)
    {
        if (!driver.ZeroWrite(address))
        {
            Serial.printf("Found on adress: %d(dec); 0x%x(hex)\n" , address, address);
        }
    }
    
    delay(200);
    // UFO_CreateTask(UFO_TASKS_ID::TASK_WIFI_HANDL);
    delay(100);
    // UFO_CreateTask(UFO_TASKS_ID::TASK_IMU, &driver);
    delay(100);
    // UFO_CreateTask(UFO_TASKS_ID::TASK_BME);
    delay(100);

    motors.Begin();
}
void loop()
{   
#ifdef TEST_MMM
    qmc.update();
    mag_data = qmc.Get();
    Serial.print(">X:");
    Serial.println(mag_data.magX);
    Serial.print(">Y:");
    Serial.println(mag_data.magY );
    Serial.print(">Z:");
    Serial.println(mag_data.magZ );
    delay(20);
#else

    compass();
    comData = compass.Get();
    Serial.print(">X:");
    Serial.println(comData._x);
    Serial.print(">Y:");
    Serial.println(comData._y);
    Serial.print(">Z:");
    Serial.println(comData._z);
    delay(20);

#endif

#pragma region


    // if (Serial.available() > 1)
    // {
    //     char key = Serial.read();
    //     int val = Serial.parseInt();
    //     switch (key)
    //     {
    //     case 'V':
    //         motors.SendAll(val);
    //         break;
    //     case 'A':
    //         motors.Arm();
    //         digitalWrite(4, 1);
    //         digitalWrite(2, 1);
    //         break;
    //     case 'D':
    //         motors.DisArm();
    //         digitalWrite(4, 0);
    //         digitalWrite(2, 0);
    //         break;
    //     case '0':
    //         motors.Send(0, val);
    //         break;
    //     case '1':
    //         motors.Send(1, val);
    //         break;
    //     case '2':
    //         motors.Send(2, val);
    //         break;
    //     case '3':
    //         motors.Send(3, val);
    //         break;
    //     default:
    //         break;
    //     }
    // }

#pragma endregion
    
}

// #include <QMC5883LCompass.h>
// #include "MPU6050_6Axis_MotionApps20.h"
// #include "UFO_Tasks/UFO_Task.h"

// QMC5883LCompass compass;

// void setup()
// {
//     Serial.begin(115200);
//     Wire.begin();
//     // compass.init();
//     // Serial.println("CALIBRATING. Keep moving your sensor...");
//     // compass.calibrate();

//     //   Serial.println("DONE. Copy the lines below and paste it into your projects sketch.);");
//     //   Serial.println();
//     //   Serial.print("compass.setCalibrationOffsets(");
//     //   Serial.print(compass.getCalibrationOffset(0));
//     //   Serial.print(", ");
//     //   Serial.print(compass.getCalibrationOffset(1));
//     //   Serial.print(", ");
//     //   Serial.print(compass.getCalibrationOffset(2));
//     //   Serial.println(");");
//     //   Serial.print("compass.setCalibrationScales(");
//     //   Serial.print(compass.getCalibrationScale(0));
//     //   Serial.print(", ");
//     //   Serial.print(compass.getCalibrationScale(1));
//     //   Serial.print(", ");
//     //   Serial.print(compass.getCalibrationScale(2));
//     //   Serial.println(");");
//     // delay(5000);
//     compass.setMagneticDeclination(12, 0);
//     compass.setCalibrationOffsets(-66.00, 36.00, -446.00);
//     compass.setCalibrationScales(1.03, 0.96, 1.01);

// }
// float hh = 0;

// // Tilt compensation
// float tiltCompensate(VectorFloat &v, float roll, float pitch)
// {

//     // Some of these are used twice, so rather than computing them twice in the algorithem we precompute them before hand.
//     float cosRoll = std::cos(roll);
//     float sinRoll = sin(roll);
//     float cosPitch = cos(pitch);
//     float sinPitch = sin(pitch);

//     // Tilt compensation
//     // float Xh = v.x * cosPitch + v.z * sinPitch;
//     // float Yh = v.x * sinRoll * sinPitch + v.y * cosRoll - v.z * sinRoll * cosPitch;
//     float Xh = v.x * cosPitch + v.y * sinRoll * sinPitch - v.z * cosRoll * sinPitch;
//     float Yh = v.y * cosRoll + v.z * sinRoll;

//     float heading = atan2(Yh, Xh);

//     return heading;
// }

// // Correct angle
// float correctAngle(float heading)
// {
//     if (heading < 0)
//     {
//         heading += 2 * PI;
//     }
//     if (heading > 2 * PI)
//     {
//         heading -= 2 * PI;
//     }

//     return heading;
// }

// void loop()
// {


//     // int x, y, z;
//     // // // Read compass values
//     // compass.read();
//     // x = compass.getX();
//     // y = compass.getY();
//     // z = compass.getZ();
//     // VectorFloat v = VectorFloat(x, y, z).getNormalized();
//     // // Serial.print("X: ");
//     // // Serial.print(v.x);
//     // // Serial.print(" Y: ");
//     // // Serial.print(v.y);
//     // // Serial.print(" Z: ");
//     // // Serial.print(v.z);
//     // // Serial.println();
//     // float heading = atan2(y, x);
//     // hh = heading;
//     // heading = hh + 0.20944;
//     // float headingDegrees = correctAngle(heading) * 180 / M_PI;

//     // Serial.print("Heading (no comp/ comp): ");
//     // Serial.print(headingDegrees);
//     // Serial.print("\t");

//     // float comH = tiltCompensate(v, ypr[2], ypr[1]) + 0.20944;
//     // comH = correctAngle(comH) * 180 / M_PI;
//     // Serial.println(comH);

//     // delay(50);
// }
