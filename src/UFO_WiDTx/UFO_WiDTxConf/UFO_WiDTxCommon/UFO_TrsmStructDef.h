
#pragma once
#include "UFO_Config.h"

#define UFO_TRSM_BUFFER_SIZE 128


struct UFO_TrsmDataControlBlock
{
    char _payload[UFO_TRSM_BUFFER_SIZE];
    uint8_t _len = 0;
    uint8_t _errorCount = 0;
    int32_t _lastCallTick = 0;
    bool _ready = false;
};

struct UFO_TrsmControlBlock
{
private:
    UFO_TrsmDataControlBlock _data;
    SemaphoreHandle_t _lock;

public:
    UFO_TrsmControlBlock()
    {
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

     ~UFO_TrsmControlBlock()
    {
        if (_lock)
        {
            vSemaphoreDelete(_lock);
        }
    }

    SemaphoreHandle_t Lock(){
        return _lock;
    }

    UFO_TrsmDataControlBlock& GetDataBlock(){
        return _data;
    }


    bool Msg(uint8_t offset, const char *payload, int16_t size)
    {
        if (size > UFO_TRSM_BUFFER_SIZE- offset-1)
        {
            return false; // false if overflow
        }
        if (xSemaphoreTake(_lock, TickType_t(1000)) == pdTRUE)
        {
            memcpy((_data._payload + offset), payload, size);
            _data._len = size + offset;
            _data._ready = true;
            xSemaphoreGive(_lock);
            return true;
        }
        return false;
    }

    bool Msg(const char *payload, int16_t size)
    {
        if (size > UFO_TRSM_BUFFER_SIZE-1)
        {
            return false; // false if overflow
        }
        if (xSemaphoreTake(_lock, TickType_t(1000)) == pdTRUE)
        {
            memcpy(_data._payload, payload, size);
            _data._len = size;
            _data._ready = true;
            xSemaphoreGive(_lock);
            return true;
        }
        return false;
    }

    bool Msg(const char *payload)
    {
        int16_t l = strlen(payload);
        if (l > UFO_TRSM_BUFFER_SIZE-1)
        {
            return false; // false if overflow
        }
        if (xSemaphoreTake(_lock, TickType_t(1000)) == pdTRUE)
        {
            memcpy(_data._payload, payload, l);
            _data._len = l;
            _data._ready = true;
            xSemaphoreGive(_lock);
            return true;
        }
        return false;
    }

};
