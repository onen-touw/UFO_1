// #include <Arduino.h>
#include "UFO_Config.h"

#include "UFO_Driver/UFO_ESC.h"
#ifdef UFO_TEST_MODE

#include "UFO_Tasks/UFO_Task.h"

#include "RxTxDataHandler.h"
// #include "UFO_Control.h"

//==============================================
#define TEST_WIFI
#define UFO_TEST_SOCK_COMM
// #define UFO_TEST_LORA_2
// #define UFO_TEST_UART
#define UFO_TEST_INA3221
// #define UFO_TEST_INA219
// #define TEST_4INA
// #define UFO_TEST_HX711_SCALE

// #define UFO_TEST_SocketTask
#define TEST_WHEELS
//==============================================

#ifdef TEST_WIFI
#include "UFO_WiFi.h"
#endif

#ifdef UFO_TEST_SOCK_COMM
#include "UFO_WiDTx/UFO_Socket.h"
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

#ifdef UFO_TEST_HX711_SCALE
#include "UFO_Sensors/UFO_other/UFO_HX711.h"
#endif


#ifdef TEST_MOTORS
UFO_Control control;
UFO_Motors motors;
#endif




void rrrr( const char* t, const char* f, int i, const char* msg){
    const char* in = "\033[1;"
    "41;33m";
    const char* out= "\033[0m";
    
    
    std::cout << in << "[" << t << "] - Error in file[" << f << "] Line[" << i << "] msg: " << msg << out << "\n"; 
}

#define UFO_ErrorGeneric(tag, msg) rrrr(tag, __FILE__, __LINE__, msg)

//==============================================


#ifdef UFO_TEST_SOCK_COMM
    UFO_TrsmControlBlock* txdata = new UFO_TrsmControlBlock();
    RxDataHandler rxdh;

#endif
#ifdef UFO_TEST_UART
#include "UFO_Driver/UFO_Uart.h"
UFO_Uart port1(2);
char rxbf[255] = {};
#endif

#ifdef UFO_TEST_INA3221
#include "UFO_Sensors/UFO_Sensors_I2C/UFO_INA3221.h"

#define INA3221_TAG "INA0"
#define ESP_TAG "ESP1"



#endif
#ifdef UFO_TEST_INA219
#include "UFO_Sensors/UFO_Sensors_I2C/UFO_INA219.h"
#endif

#ifdef TEST_4INA
#include "UFO_Utils/UFO_4inaMeasure.h"
#endif

int32_t p = 0, r = 0;
int32_t _cnt = 0;

UFO_I2C_Driver driver;




#ifdef UFO_TEST_INA3221
UFO_INA3221 ina(&driver);
#endif

#ifdef UFO_TEST_INA219
UFO_INA219 ina(&driver, 0x45);
#endif

#ifdef UFO_TEST_HX711_SCALE
UFO_HX711 hx711;
#endif

#ifdef TEST_4INA
// UFO_4inaMeasure _mes;

UFO_INA219 s_0x40(&driver,  0x40);
UFO_INA219 s_0x41(&driver, 0x41);
UFO_INA219 s_0x44(&driver, 0x44);
UFO_INA219 s_0x45(&driver, 0x45);
#endif


void setup()
{


// int16_t v = constrain(val, UFO_MOTOR_VAL_MIN, UFO_MOTOR_VAL_MAX);
// v = map(v, UFO_MOTOR_VAL_MIN, UFO_MOTOR_VAL_MAX, UFO_MOTOR_RES_VAL_MIN, UFO_MOTOR_RES_VAL_MAX);

    delay(100);
    // Serial.begin(115200);
    // std::cout << "stdcttest write without initialization\n";
#ifdef UFO_TEST_UART

    delay(100);
    esp_err_t ee = port1.Init();
    if (ee != ESP_OK)
    {
        std::cout << "some problems\n";
    }
    // port1.SendWithBreak("Hello world");
    // port1.SendMsg("Hello world");
    // port1.Write("Hello world");
    // port1.Write("trash after hello world");

    return;
#else
    Serial.begin(115200);
#endif


//     Serial2.begin(0);
//     Serial2.setTimeout(100);
// return;


    // Wire.begin();
    // Wire.setClock(400000); //400khz clock

    esp_err_t err = driver.Init(UFO_I2C_port::UFO_I2C_HARDWARE);
    Serial.println(err);
    Serial.print("inited ");
    Serial.println(driver.Initialized());
    
    // analogWriteFrequency(10000);

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
    
    uint8_t inaAddrArr[127]={};
    uint8_t sz = 0;
    Serial.println("Start");
    for (uint8_t address = 1; address < 127; ++address)
    {
        if (!driver.ZeroWrite(address))
        {
            Serial.printf("Found on adress: %d(dec); 0x%x(hex)\n" , address, address);

            // for a while
            inaAddrArr[sz] = address;
            ++sz;
        }
    }

#ifdef TEST_WIFI
    UFO_WiFi_t wf = UFO_WiFi_GetInstance(); 
    wf.Init();
    // err = _wfd.Init();
    // err = _wfd.SetApIP("192.168.40.12");
#endif


    // rxdh.Addhandler([&](RX_INDEX i, int32_t v){
    //     switch (i)
    //     {
    //     case RX_IND_PITCH:
    //         p=v;
    //         break;
    //     case RX_IND_ROLL: 
    //         r=v;
    //         break;
    //     default:
    //         break;
    //     }
    //     // TestPinLoop(r,p);
    //     TestPinLoop(p,r, 0);
    // });


#ifdef UFO_TEST_SOCK_COMM
    
    UFO_SockConfigMinimal* sockConf = new UFO_SockConfigMinimal();
    sockConf->_type = UFO_SOCK_SERVER;
    sockConf->_callbackFunc = [&](UFO_TrsmDataControlBlock* rrcv){
        // String ss = (rrcv->_payload);
        // rxdh(ss);
        Serial.printf(">>%s\n", rrcv->_payload);
    };
    sockConf->_port = 6464;
    sockConf->_TxBuf = txdata;
    UFO_CreateTask(TASK_SOCKET, sockConf);
    
#endif

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

// std::cout <<"\x1B[31mTexting\033[0m\n";
// std::cout <<"\x1B[1;41;37mTexting\033[0m\n";
// std::cout <<"\033[1;41;33mTexting\033[0m\n";
UFO_ErrorGeneric("tg", "errmsg");

#ifdef UFO_TEST_INA3221
    ina.InitSensor();
#endif

#ifdef UFO_TEST_INA219
    ina.InitSensor();
#endif


#ifdef UFO_TEST_HX711_SCALE
    hx711.begin( 4, 2);
#endif


#ifdef TEST_4INA
s_0x40.InitSensor();
// _mes.InitSensors(&driver, inaAddrArr, sz);
#endif


#ifdef TEST_INA
// create ina task
    SendToInaTask toina;
    toina._q = _queue;
    toina._d = &driver;
    RC_CreateTask(TASK_INA, &toina);
    delay(1000);
#endif
}


void loop()
{   
#ifdef UFO_TEST_LORA_2

    txdata->Msg(UFO_LORA_OFFSET_FOR_ADDR, "hyatinka", 9);

    lora.Iteration();
    delay(50);
#endif

#ifdef UFO_TEST_HX711_SCALE
// if (hx711.is_ready()) {
//     long reading = hx711.read();
//     Serial.print("HX711 reading: ");
//     Serial.println(reading);
//   } else {
//     Serial.println("HX711 not found.");
//   }
//   delay(1000);
if (hx711.wait_ready_timeout(1000)) {
    long reading = hx711.read();
    Serial.print("HX711 reading: ");
    Serial.println(reading);
  } else {
    Serial.println("HX711 not found.");
  }
  delay(1500);

#endif

#ifdef UFO_TEST_INA3221
    String s;
    ina.Update();
    UFO_INA3221_ChannelData d;
    for (size_t i = 0; i < 3; i++)
    {
        s = ESP_TAG;
        s += INA3221_TAG;
        s += (i + 1);
        d = ina.Get(i);
        s += '@';
        s += d._timestamp;
        s += ',';
        s += d._busVolt;
        s += ',';
        s += d._current;
        txdata->Msg(s.c_str(), s.length());
    }
    delay(20);

#endif
#ifdef UFO_TEST_INA219
    ina.Update();
    UFO_DataINA219 ch1 = ina.Get();
    
    Serial.print("Channel 1: \n Current: ");
    Serial.print(ch1._current);
    Serial.print("mA\n Voltage: ");
    Serial.print(ch1._busVolt);
    Serial.print("V\n Shunt Voltage: ");
    Serial.print(ch1._shuntVolt);
    Serial.println("mV");

    delay(100);
#endif
#ifdef UFO_TEST_UART
    if (port1.Available())
    {
        port1.Read(rxbf);
        std::cout << rxbf << '\n';
    }
    port1.SendMsg("hayhay\n");
    delay(100);
    
#endif

// #ifdef UFO_TEST_SOCK_COMM
//     String m(_cnt++);
//     m+= "_cnt";
//     txdata->Msg(m.c_str(), m.length());
//     delay(20);
// #endif


#ifdef TEST_4INA
    // uint32_t t = millis();
    // String s = _mes.Update();
    String s;
    s_0x40.Update();
    UFO_DataINA219 d = s_0x40.Get();
    s+= "ESP2";
    s+= "INA10";
    s+= '@';
    s+= d._timestamp;
    s+= ',';
    s+= d._busVolt;
    s+= ',';
    s+= d._current;
    Serial.println(s);
    txdata->Msg(s.c_str(), s.length());
    delay(1000);
    // Serial.println(millis()-t);
#endif

}

#pragma region\
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


#else   // realise mode



#ifdef UFO_TEST_CORE
#include "UFO_Core.h"
#endif

void setup(){

#ifdef UFO_TEST_CORE
    uint8_t flag =
        use_wifi |
        use_i2c |
        use_hw_uart1 |
        use_hw_uart2 |
        use_sock |
        use_lora |
        use_debug |
        use_debugColorful;

    UFO_Core& core = UFO_Core::GetInstance(flag);

return;



#endif
}

void loop(){

}





#endif //UFO_TEST_MODE
