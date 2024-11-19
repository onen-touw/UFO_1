#include <Arduino.h>
#include "UFO_Tasks/UFO_Task.h"

#include "RxTxDataHandler.h"
#include "UFO_Control.h"

#define UFO_TEST_MODE


#ifdef UFO_TEST_MODE



//==============================================
// #define TEST_WIFI
#define UFO_TEST_SOCK_COMM
#define UFO_TEST_LORA_2

// #define UFO_TEST_SocketTask
// #define TEST_WHEELS
//==============================================

#ifdef TEST_WIFI
#include "UFO_WiFi_.h"
#endif

#ifdef UFO_TEST_SOCK_COMM
#include "UFO_WiDTx/UFO_Socket.h"
UFO_Socket sock;
#endif

#ifdef UFO_TEST_LORA_2
#include "UFO_WiDTx/UFO_Lora.h"
UFO_Lora_e220440T22D lora;
UFO_TrsmControlBlock* txdata = new UFO_TrsmControlBlock();

#endif


#ifdef TEST_WHEELS
#include "TEST_Wheel.h"
#endif

#ifdef UFO_TEST_SocketTask
#include "UFO_Socket.h"
#endif



#ifdef TEST_MOTORS
UFO_Control control;
UFO_Motors motors;
#endif


#endif //UFO_TEST_MODE
//==============================================

#ifdef TEST_WIFI
UFO_WiFi_ _wfd;
#endif

#ifdef UFO_TEST_SocketTask
    SockTxData_t* txdata = new SockTxData_t();
    RxDataHandler rxdh;

#endif

int32_t p = 0, r = 0;
int32_t _cnt = 0;



UFO_I2C_Driver driver;
void setup()
{
    std::cout << "stdcttest write without initialization\n";
    printf("pftest write without initialization\n");
    delay(100);
    Serial.begin(115200);
    std::cout << "stdcttest write without initialization\n";
    printf("pftest write without initialization\n");

    Serial2.begin(9600);
    Serial2.setTimeout(100);

    // Wire.begin();
    // Wire.setClock(400000); //400khz clock

    esp_err_t err = driver.Init(UFO_I2C_port::UFO_I2C_HARDWARE);
    Serial.println(err);
    Serial.print("inited ");
    Serial.println(driver.Initialized());


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

#ifdef TEST_WIFI
    err = _wfd.Init();
    // err = _wfd.SetApIP("192.168.40.12");
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
        // String ss = (rrcv->_buff);
        // rxdh(ss);
        Serial.printf(">>%s\n", rrcv->_buff);
    };
    sockConf->_port = 6464;
    sockConf->_TxBuf = txdata;
    UFO_CreateTask( TASK_SOCKET, sockConf);

    TestPinSetup();
#endif
    // TestPinSetup();






#ifdef UFO_TEST_LORA_2
    UFO_LoraConfigMinimal cfg;

    UFO_LoraSettings sett;
    sett._selfAddr._addh = 0;
    sett._selfAddr._addl = 25;
    sett._selfAddr._chan = 8;

    sett._targAddr._addh = UFO_LORA_BROADCAST;
    sett._targAddr._addl = UFO_LORA_BROADCAST;
    sett._targAddr._chan = 10;
    sett.adrt =  LORA_AIR_DATA_RATE_110_384;

    cfg._ctrlBlk = txdata;
    cfg._settings =sett;
    cfg._callback = [](UFO_TrsmDataControlBlock* rcv){
        Serial.printf("rcv: %s\n", rcv->_payload);
    };

    lora.SetConfig(cfg);
    lora.Setup();
#endif
}


void loop()
{   
#ifdef UFO_TEST_LORA_2

    txdata->Msg(UFO_LORA_OFFSET_FOR_ADDR, "hyatinka", 9);

    lora.Iteration();
    delay(50);
#endif


#ifdef UFO_TEST_SocketTask
    String m(_cnt++);
    m+= "_cnt";
    txdata->Msg(m.c_str());
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