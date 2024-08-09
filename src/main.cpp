#include <Arduino.h>
#include "WiFi.h"
#include "AsyncUDP.h"
#include "RxTxDataHandler.h"
#include "UFO_Tasks/UFO_Task.h"
#include "Adafruit_BME280.h"
#include "UFO_Driver/UFO_ESC.h"
// #include "UFO_Motors.h"
#include "UFO_Control.h"
#include "UFO_Communication.h"

UFO_Control control;
UFO_Motors motors;
// AsyncUDP udp;
// RxDataHandler rxDH;
UFO_PDOA test_cls;

void setup()
{
    delay(100);
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(400000); //400khz clock

    test_cls.CreateItem(123);
    test_cls.CreateItem(0.213f);
    test_cls.CreateItem(true);
    String txt = "string";
    test_cls.CreateItem(txt);  
    test_cls.CreatePacket();
    String packet = test_cls.GetPacket();
    for (size_t i = 0; i < packet.length(); i++)
    {
        Serial.print(static_cast<uint8_t>(packet[i]));
        Serial.print("\t|");
        Serial.print(packet[i]);
        Serial.println("|");
    }
    
    Serial.println();
    
    delay(100);
    control.Setup();

    delay(100);

    for (int32_t address = 1; address < 127; address++)
    {
        Wire.beginTransmission(address);
        uint8_t error = Wire.endTransmission();
        if (error == 0)
        {
            Serial.print("\tfound:");
            Serial.print(" ");
            Serial.println(address);
        }
    }
    // uint8_t pins[] = {13,14,26,27};

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
    delay(200);

    Serial.println("Start");
    // UFO_CreateTask(UFO_TASKS_ID::TASK_WIFI_HANDL);
    delay(100);
    // UFO_CreateTask(UFO_TASKS_ID::TASK_IMU);
    delay(100);
    // UFO_CreateTask(UFO_TASKS_ID::TASK_BME);
    delay(100);

    motors.Begin();
}
void loop()
{
    if (Serial.available() > 1)
    {
        char key = Serial.read();
        int val = Serial.parseInt();
        switch (key)
        {
        case 'V':
            motors.SendAll(val);
            break;
        case 'A':
            motors.Arm();
            digitalWrite(4, 1);
            digitalWrite(2, 1);
            break;
        case 'D':
            motors.DisArm();
            digitalWrite(4, 0);
            digitalWrite(2, 0);
            break;
        case '0':
            motors.Send(0, val);
            break;
        case '1':
            motors.Send(1, val);
            break;
        case '2':
            motors.Send(2, val);
            break;
        case '3':
            motors.Send(3, val);
            break;
        default:
            break;
        }
    }

   
    // if (Serial.available() > 1)
    // {
    //     char key = Serial.read();
    //     int val = Serial.parseInt();
    //     switch (key)
    //     {
    //     case 'V':
    //         control.SetTargetTrust(val);
    //         break;
    //     case 'A':
    //         control.SetArm(1);
    //         break;
    //     case 'D':
    //         control.SetArm(1);
    //         break;
    //     default:
    //         break;
    //     }
    // }
    // control.Iteration();


    // motor01.send_dshot_value(dval);
    // control.Iteration();
}

// #include <QMC5883LCompass.h>
// #include "MPU6050_6Axis_MotionApps20.h"
// #include "UFO_Tasks/UFO_Task.h"

// QMC5883LCompass compass;
// // MPU6050 mpu;
// // uint8_t fifoBuffer[64]; // FIFO storage buffer
// // Quaternion q;       // [w, x, y, z]         quaternion container
// // VectorFloat gravity; // [x, y, z]            gravity vector
// // float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

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

//     // mpu.initialize();
//     // mpu.CalibrateAccel();
//     // mpu.CalibrateGyro();
//     // mpu.dmpInitialize();
//     // mpu.setDMPEnabled(true);
//     UFO_CreateTask(UFO_TASKS_ID::TASK_IMU);
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

//     // mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
//     // // mpu.getFIFOBytes(fifoBuffer, packetSize);
//     // mpu.dmpGetQuaternion(&q, fifoBuffer);
//     // mpu.dmpGetGravity(&gravity, &q);
//     // mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//     // Serial.print("ypr\t");
//     // Serial.print(ypr[0] * 180 / M_PI);
//     // Serial.print("\t");
//     // Serial.print(ypr[1] * 180 / M_PI);
//     // Serial.print("\t");
//     // Serial.println(ypr[2] * 180 / M_PI);

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
