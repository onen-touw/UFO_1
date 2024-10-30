#pragma once

#include "UFO_Config.h"

#define UFO_SOCKET_BUFF_SIZE 256

struct SockDCB_t
{
    char* _buff[UFO_SOCKET_BUFF_SIZE];
    int16_t _len = 0;
    int32_t _lastCallTick = 0;
};

class SockTxData_t
{
public: // public for a while


    SockDCB_t _dcb;
    SemaphoreHandle_t _lock;
    bool _ready = false;

    SemaphoreHandle_t GetLock() const { return _lock; }

    SockTxData_t(){
       _lock = xSemaphoreCreateMutex();
    }

    ~SockTxData_t()
    {
        if (_lock)
        {
            vSemaphoreDelete(_lock);
        }
    }
};


