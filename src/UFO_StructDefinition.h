#pragma once
#include "UFO_Config.h"
#include "UFO_WiDTx/UFO_WiDTxConf/UFO_WiDTxCommon/UFO_TrsmStructDef.h"
// #include "RC_Event.h"
#include "UFO_math/UFO_MathUtils.h"

struct UFO_LockedBase
{
    SemaphoreHandle_t _lock = nullptr;
    UFO_LockedBase(){
        _lock = xSemaphoreCreateMutex();
        if (!_lock)
        {
            //set critical
        }
        
    }
    ~UFO_LockedBase(){
        if (_lock)
        {
            vSemaphoreDelete(_lock);
        }
    }
};


struct UFO_DataIMU : UFO_LockedBase
{
    float
        Yaw =   0.f,
        Pitch = 0.f,
        Roll =  0.f;
};
struct UFO_DataBaro : UFO_LockedBase
{
    float _Tem = 0.f;
    float _Pre = 0.f;
    float _Lvl = 0.f;
};
struct UFO_DataMotorThrot : UFO_LockedBase
{
    uint16_t _M1 = 0;
    uint16_t _M2 = 0;
    uint16_t _M3 = 0;
    uint16_t _M4 = 0;
};


struct UFO_DataINA219__ : UFO_LockedBase
{
    float _current =0.f;
    float _busVolt =0.f;
    float _shuntVolt =0.f;
    float _power = 0.f;
};

//TODO
struct UFO_INA3221_Channel__ {
    float _current =0.f;
    float _busVolt =0.f;
    float _shuntVolt =0.f;
    float _power = 0.f;

};

struct UFO_DataINA3221 : UFO_LockedBase {

//TODO
    UFO_INA3221_Channel__ _chA;
    UFO_INA3221_Channel__ _chB;
    UFO_INA3221_Channel__ _chC;
};


struct UFO_TaskPtr {
    TaskHandle_t _sys = nullptr;

#ifdef UFO_USE_TASK_BARO
    TaskHandle_t _baro= nullptr;
#endif
#ifdef UFO_USE_TASK_IMU
    TaskHandle_t _imu= nullptr;
#endif
#ifdef UFO_USE_TASK_SOCK
    TaskHandle_t _sock= nullptr;
#endif
#ifdef UFO_USE_TASK_COMPASS //????
    TaskHandle_t _compass= nullptr;
#endif
#ifdef UFO_USE_TASK_LORA
    TaskHandle_t _lora = nullptr;
#endif
#ifdef UFO_USE_TAST_INA219
    TaskHandle_t _ina219 = nullptr;
#endif
#ifdef UFO_USE_TAST_INA3221
    TaskHandle_t _ina3221 = nullptr;
#endif

    TaskHandle_t _main = nullptr;
};
struct UFO_SystemStateFlag {
    uint8_t _state = 0xFF; //11 11 11 11{ ([wf][wf_e]) ([][]) ([][]) ([][])}
};

struct UFO_DataSystem {
    UFO_TaskPtr _taskPtr;
    UFO_SystemStateFlag _flag;
};

struct UFO_DataBlock {
private:
    UFO_DataSystem _sys;
    /*error catch system*/
public: 
#ifdef UFO_USE_TASK_BARO
    UFO_DataBaro _baro;
#endif
#ifdef UFO_USE_TASK_IMU
    UFO_DataIMU _imu;
#endif
#ifdef UFO_USE_TASK_COMPASS
    // TaskHandle_t _compass;
#endif

#ifdef UFO_USE_TAST_INA219
    UFO_DataINA219 _ina219
#endif
#ifdef UFO_USE_TAST_INA3221
    UFO_DataINA3221 _ina3221;
#endif

// main data-control-block 
#if defined(UFO_USE_TASK_SOCK) || defined(UFO_USE_TASK_LORA) || defined(UFO_USE_TASK_SOCK_LORA)
    UFO_TrsmControlBlock _trsmDCB;
#endif
// mb define if sock and lora are in different tasks 


    UFO_DataMotorThrot _motor;


UFO_DataSystem* __GetSys() { return &_sys; }

};