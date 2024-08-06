#pragma once

#include "UFO_Config.h"
#include "UFO_TaskClassBase.h"
#include "AsyncUDP.h"
#include "RxTxDataHandler.h"

#define UFO_UDP_WAIT_NOTHIN_TIME_LIMIT ((uint64_t)10000)

class UFO_UDPSender : public UFO_TaskClassBase
{
private:
    int32_t _lastCallTimePoint = 0;
    AsyncUDP _udp;
    RxDataHandler _rx;
    IPAddress _addr;
    uint16_t _port = 8080;
    bool _con = false;

public:
    UFO_UDPSender()
    {
        // _lastCallTimePoint = millis();
        // _udp.connect(_addr, _port);
        // _con 
    }
    ~UFO_UDPSender() {}

    void Setup() final
    {
    }
    void Iteration() final
    {
    }
};