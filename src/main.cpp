#include <Arduino.h>
#include "UFO_Tasks/UFO_Task.h"

#include "RxTxDataHandler.h"
#include "UFO_Control.h"

#define UFO_TEST_MODE


#ifdef UFO_TEST_MODE



//==============================================
// #define UFO_TEST_UDP
#define UFO_TEST_SocketTask
#define TEST_WIFI
// #define TEST_PID
// #define TEST_COMPASS
// #define TEST_MOTORS
// #define TEST_BME
#define TEST_WHEELS
//==============================================

#ifdef TEST_WHEELS
#include "TEST_Wheel.h"
#endif

#ifdef UFO_TEST_SocketTask
#include "UFO_Socket.h"
#endif

#ifdef TEST_WIFI
#include "UFO_WiFi_.h"
#endif

#ifdef UFO_TEST_UDP
// #include "UFO_UDP.h"
#include "UFO_Socket.h"
#endif

#ifdef TEST_BME
#include "UFO_Sensors/UFO_Sensors_I2C/UFO_Baro.h"
#define TEST_BME_NEW UFO_ENABLE /*UFO_DISABLE*/
#endif

#ifdef TEST_COMPASS
#include "UFO_Sensors/UFO_Sensors_I2C/UFO_Compass.h"
#endif


#ifdef TEST_MOTORS
UFO_Control control;
UFO_Motors motors;
#endif

#ifdef TEST_PID
#include "UFO_math/UFO_PID.h"
#include "UFO_Sensors/UFO_Sensors_I2C/UFO_IMU.h"
#endif

#endif //UFO_TEST_MODE
//==============================================

#ifdef UFO_TEST_SocketTask
    SockTxData_t* txdata = new SockTxData_t();
    RxDataHandler rxdh;

#endif


#ifdef TEST_BME
UFO_Baro baro(&driver);
UFO_BaroData_t baroData;
#endif

#ifdef TEST_COMPASS
UFO_Compass compass(&driver);
Vector3<float> comData = {};
#endif

#ifdef TEST_PID
UFO_IMU imu(&driver);
float
    _kp = 10.f,
    _ki = 0.f,
    _kd = 0.f,
    _ti = 0.1f;
UFO_PID pitchPID(_kp, _ki, _kd, _ti);
UFO_IMU_Data imuData;
Madgwick maFilter;
UFO_KalmanFilter adFilter;
#endif

#ifdef TEST_WIFI
UFO_WiFi_ _wfd;
#endif

#ifdef UFO_TEST_UDP
// UFO_UDP udp;
UFO_Socket sock;
SockTxData_t* txdata = new SockTxData_t();
int32_t _cnt = 0;
RxDataHandler rxdh;
#endif
        int32_t p = 0, r=0;

UFO_I2C_Driver driver;
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


#ifdef TEST_COMPASS
    compass.InitSensor();
#endif

#ifdef TEST_BME
    baro.InitSensor();
    baro.SetTemperatureCompensation(-5.f);
#endif
#ifdef TEST_PID
    adFilter.Set(0.5f, 0.5f, 0.8f);
    imu.Calibrate();
    maFilter.begin(0.2f);

    //calibration
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
#ifndef UFO_TEST_MODE
    delay(200);
    // UFO_CreateTask(UFO_TASKS_ID::TASK_WIFI_HANDL);
    delay(100);
    // UFO_CreateTask(UFO_TASKS_ID::TASK_IMU, &driver);
    delay(100);
    // UFO_CreateTask(UFO_TASKS_ID::TASK_BME);
    delay(100);
#endif
    // motors.Begin();

#ifdef TEST_WIFI
    err = _wfd.Init();
    // err = _wfd.SetApIP("192.168.40.12");
#endif


#ifdef UFO_TEST_UDP
    Serial.println("Test buffer");

    String ss = "test_buffer";

    // txdata->_lock = xSemaphoreCreateMutex();
    // if (!txdata->_lock)
    // {
    //     // set critical  [no mem]
    //     Serial.print("critical error:: [xSemaphoreCreateMutex] no mem");
    //     Serial.print(" ");
    //     Serial.print(__FILE__);
    //     Serial.print("\t");
    //     Serial.println(__LINE__);
    // }
    // Serial.println("lock created");

    memcpy(txdata->_dcb._buff, ss.c_str(), ss.length());
    txdata->_dcb._len = ss.length();

    Serial.printf("string: %s, len: %d\n", txdata->_dcb._buff, txdata->_dcb._len);

    Serial.println("Sock initializing");
    sock.SetType(UFO_SOCK_SERVER);
    sock.SetPort(6464);
    sock.SetTrsmCB(txdata);

    rxdh.Addhandler([&](RX_INDEX i, int32_t v){
        switch (i)
        {
        case RX_IND_PITCH:
            p=v;
            break;
        case RX_IND_ROLL: 
            r=v;
            break;
        default:
            break;
        }


    });

    sock.SetRecvFuck([](SockDCB_t* rrcv){
        String ss = (rrcv->_buff);
        rxdh(ss);
    });


    sock.Setup();
#endif

#ifdef UFO_TEST_SocketTask
    rxdh.Addhandler([&](RX_INDEX i, int32_t v){
        switch (i)
        {
        case RX_IND_PITCH:
            p=v;
            break;
        case RX_IND_ROLL: 
            r=v;
            break;
        default:
            break;
        }
        TestPinLoop(p,r);
    });


    UFO_SockConfigMinimal* sockConf = new UFO_SockConfigMinimal();
    sockConf->_type = UFO_SOCK_SERVER;
    sockConf->_callbackFunc = [&](SockDCB_t* rrcv){
        String ss = (rrcv->_buff);
        rxdh(ss);
    };
    sockConf->_port = 6464;
    sockConf->_TxBuf = txdata;
    UFO_CreateTask( TASK_SOCKET, sockConf);


    TestPinSetup();
#endif
    // UFO_CreateTask( TASK_IMU, &driver);


    // TestPinSetup();
}


void loop()
{   
#ifdef TEST_COMPASS
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

#ifdef TEST_BME
    #if TEST_BME_NEW
    baro.Update();
    #else
    baro();
    #endif
    baroData = baro.Get();

    Serial.print(">T:");
    Serial.println(baroData.Tempreture);
    Serial.print(">P:");
    Serial.println(baroData.Presure);
    Serial.print(">ALT:");
    Serial.println(baroData.Altitude);
    delay(20);
#endif

#ifdef TEST_PID
    if (Serial.available() > 1)
    {
        char key = Serial.read();
        float val = Serial.parseFloat();
        switch (key)
        {
        case 'P':
            Serial.print("SetP ");
            Serial.println(val);
            _kp = val;
            break;
        case 'I':
            Serial.print("SetI ");
            Serial.println(val);
            _ki = val;
            break;
        case 'D':
            Serial.print("SetD ");
            Serial.println(val);
            _kd = val;
            break;
        case 'T':
            Serial.print("SetT ");
            Serial.println(val);
            _ti = val;
            break;
        case 'O':
            Serial.println("Info");
            Serial.print("P ");
            Serial.print(_kp);
            Serial.print("\tI ");
            Serial.print(_ki);
            Serial.print("\tD ");
            Serial.print(_kd);
            Serial.print("\tTi ");
            Serial.println(_ti);
            
        default:
            break;
        }
        pitchPID = UFO_PID(_kp, _ki, _kd, _ti);;
    }
    imu.Update();
    imuData = imu.Get();
    maFilter.updateIMU(
        imuData._gyro._x,
        imuData._gyro._y,
        imuData._gyro._z,
        imuData._accel._x,
        imuData._accel._y,
        imuData._accel._z);
    float
        q0 = maFilter.getQuatW(),
        q1 = maFilter.getQuatX(),
        q2 = maFilter.getQuatY(),
        q3 = maFilter.getQuatZ();
    // float roll = atan2(0.5f - q1 * q1 - q2 * q2, q0 * q1 + q2 * q3);
    float pitch = asinf(-2.0f * (q1 * q3 - q0 * q2));
    // float yaw = atan2f(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3);
    float val = adFilter(pitch) * 180 / M_PI;
    Serial.print(">pitch:");
    Serial.println(val); // <-------------- filter
    Serial.print(">pitchPID:");
    Serial.println(pitchPID.Calculate(0, val)); // <-------------- filter

    delay(20);
    // Serial.print(">yaw:");
    // Serial.println(yaw* 180 / M_PI);
    // Serial.print(">pitch:");
    // Serial.println(pitch* 180 / M_PI);
    // Serial.print(">roll:");
    // Serial.println(roll* 180 / M_PI);
#endif

#ifdef UFO_TEST_UDP
    String s(_cnt);
    memcpy(txdata->_dcb._buff, s.c_str(), s.length());
    txdata->_dcb._len = s.length();
    txdata->_ready = true;
    ++_cnt;
    sock.Iteration();
    TestPinLoop(p, r);
    delay(10);
#endif

#ifdef UFO_TEST_SocketTask
    txdata->Msg("MSG");
    delay(20);
#endif
    

}


#pragma region
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
#pragma endregion