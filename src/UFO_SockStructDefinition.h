#pragma once

#include "UFO_Config.h"

#define UFO_SOCKET_BUFF_SIZE 256

enum UFO_SockType :uint8_t {
    UFO_SOCK_NO,
    UFO_SOCK_SERVER,
    UFO_SOCK_CLIENT,
};

struct SockDCB_t
{
    char _buff[UFO_SOCKET_BUFF_SIZE];
    uint8_t _errorCount = 0;
    int32_t _lastCallTick = 0;
    int16_t _len = 0;
};

class SockTxData_t
{
public: // public for a while


    SockDCB_t _dcb;
    SemaphoreHandle_t _lock;
    bool _ready = false;
    
    bool Msg(const char* pl){
        int16_t l = strlen(pl);
        if (l > UFO_SOCKET_BUFF_SIZE)
        {
            return false;       // false if overflow
        }
        if (xSemaphoreTake(_lock, TickType_t(1000)) == pdTRUE)
        {
            memcpy(_dcb._buff, pl, l);
            _dcb._len = l;
            _ready = true;
            xSemaphoreGive(_lock);
        }
        return true;
    }

    SockTxData_t(){
       _lock = xSemaphoreCreateMutex();
       if (!_lock)
       {
           // set critical  [no mem]
           Serial.print("critical error:: [xSemaphoreCreateMutex] no mem");
           Serial.print(" ");
           Serial.print(__FILE__);
           Serial.print("\t");
           Serial.println(__LINE__);
       }
       Serial.println("lock created");
    }

    ~SockTxData_t()
    {
        if (_lock)
        {
            vSemaphoreDelete(_lock);
        }
    }
};

struct UFO_SockConfigMinimal
{
    SockTxData_t* _TxBuf = nullptr;
    std::function<void(SockDCB_t* rcv)> _callbackFunc;
    uint16_t _port = 0;
    UFO_SockType _type = UFO_SOCK_NO;
    const char* _ip;
};

