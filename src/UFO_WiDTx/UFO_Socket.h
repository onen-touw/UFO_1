#pragma once

#include "UFO_Config.h"
#include "lwip/sockets.h"
// #include "UFO_TaskClassBase.h"

#include "../UFO_TaskClassBase.h"
#include "UFO_WiDTxConf/UFO_SockDefs.h"


class UFO_Socket : public UFO_TaskClassBase
{
private:
    UFO_SockType _sType = UFO_SOCK_NO;

    char _addrStr[128];
    sockaddr_in _destAddr; 
    int32_t _port = -1;
    int32_t _sock = -1;

    socklen_t _socklen = 0;
    sockaddr_storage _sourceAddr; // Large enough for both IPv4 or IPv6

    int addr_family = 0;
    int ip_protocol = 0;

    bool __addrCatched = false;

    UFO_TrsmControlBlock* _trsm = nullptr;            //add setter
    UFO_TrsmDataControlBlock* _rcv = nullptr;
    int32_t _noResponceCount = 0;

    void* _alarm = nullptr;     // void* for a while          //add setter
    std::function<void(UFO_TrsmDataControlBlock* rcv)> _recvFunck;     //add setter

public:
    UFO_Socket(/* args */) {
       _socklen = sizeof(_sourceAddr);
       _port = 6464;        //default 
       _rcv = new UFO_TrsmDataControlBlock();
    }

    ~UFO_Socket()
    {
        if (_rcv)
        {
            delete _rcv;
        }
        this->Close();
    }

    void SetRecvFuck(std::function<void(UFO_TrsmDataControlBlock* rcv)> ff){
        _recvFunck = ff;
    }

    void SetTrsmCB(UFO_TrsmControlBlock* stdcb){
        _trsm = stdcb;
    }

    void SetType(UFO_SockType t){
        _sType = t;
    }

    void SetPort(int32_t port){

        Serial.println("Socket set port :");
        
        if (port < 0)
        {
            Serial.println("\n\tSocket Incorrect port");
            Serial.print("\t used default port: ");
            _port = 6464;
        }
        Serial.println(_port);
        _port = port;
    }

    void SetServerIP(const char* sip){
        _destAddr.sin_addr.s_addr = inet_addr(sip);
    }

    void SetConfig (UFO_SockConfigMinimal* conf) {
        _recvFunck = conf->_callbackFunc;
        _port = conf->_port;
        _sType = conf->_type;
        if (_sType == UFO_SOCK_CLIENT)
        {
            _destAddr.sin_addr.s_addr = inet_addr(conf->_ip);
        }
        _trsm = conf->_TxBuf;
    }

    virtual void Setup() final
    {
        esp_err_t err = ESP_OK;
        Serial.println("Socket setuping");
        Serial.println("\tChecking");

        if (_sType == UFO_SOCK_NO)
        {
            Serial.println("\t\tError: no sock type");
            while (true)
            {
                Serial.println("Critical error");
                delay(500);
            }
            return;
        }
        if (!_trsm)
        {
            Serial.println("\t\tError: trsm cb is null");
            while (true)
            {
                Serial.println("Critical error");
                delay(500);
            }
            return;
        }
        if (!_trsm->Lock())
        {
            Serial.println("\t\tError: trsm->lock cb is null");
            while (true)
            {
                Serial.println("Critical error");
                delay(500);
            }
            return;
        }
        
        if (!_recvFunck)
        {
            Serial.println("\t\tError: no rcv fuck");
            // Serial.println("\t\t\tpassed...");
            while (true)
            {
                Serial.println("Critical error");
                delay(500);
            }
            return;
        }
        Serial.println("\tgood");

        Serial.println("Socket: Server creating");

        if (_sock >= 0)
        {
            // setCritical
            Serial.println("\tcreating fail");
            err = ESP_FAIL;
            // return err;
        }

        if (_port < 0)
        {
            // set critical
            Serial.println("\tcreating fail: incorrect port");
            err = ESP_FAIL;
            // return err;
        }

        if (_sType == UFO_SOCK_SERVER)
        {
            _destAddr.sin_addr.s_addr = lwip_htonl(INADDR_ANY);
        }
        else {
            if (!_destAddr.sin_addr.s_addr)
            {
                Serial.println("\t\tError: no Server addr");
                while (true)
                {
                    Serial.println("Critical error");
                    delay(500);
                }
                return;
            }
            
        }
        Serial.println(_destAddr.sin_addr.s_addr);
        _destAddr.sin_family = AF_INET;
        _destAddr.sin_port = htons(_port);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;

        _sock = lwip_socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (_sock < 0)
        {
            // set critical
            Serial.println("\tsocket failed");
            err = ESP_FAIL;
            // return err;
        }
        Serial.println("\tsocket created");

        // Set timeout
        struct timeval timeout;
        timeout.tv_sec = 0;         // blocking lwip_recvfrom time
        timeout.tv_usec = 5000;      // 5 ms
        lwip_setsockopt(_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

        if (_sType == UFO_SOCK_SERVER)
        {
            int e = bind(_sock, (struct sockaddr *)&_destAddr, sizeof(_destAddr));
            if (e < 0)
            {
                Serial.print("\tsocket unable to bind:");
                Serial.println(e);
            }
            Serial.print("\tsocket bound, port:");
        }
        
        Serial.println(_port);
        // return err;
    }

    virtual void Iteration() final {
    if (xSemaphoreTake(_trsm->Lock(), TickType_t(100)) == pdTRUE)
    {
        // Serial.println("Sem");
        // debug
        UFO_TrsmDataControlBlock& _dcb = _trsm->GetDataBlock();
        if (_dcb._ready)
        {
            int e = 0;
            if (_sType == UFO_SOCK_SERVER)
            {
                e = lwip_sendto(_sock,_dcb._payload, _dcb._len, 0, (struct sockaddr *)&_sourceAddr, _socklen);
            }
            else{
                e = lwip_sendto(_sock, _dcb._payload, _dcb._len, 0, (struct sockaddr *)&_destAddr, sizeof(_destAddr));
            }            
            Serial.println("sended");
            // if (PC_debug) {
            // lwip_sendto(...,_pcAddr);
            // }

            if (e < 0)
            {
                Serial.println("Error occurred during sending");        //add counter
            }

            if (_dcb._errorCount > 0)
            {
                --_dcb._errorCount;
            }

            _dcb._ready = false;
            _dcb._len = 0;
            _dcb._lastCallTick = millis(); // millis for a while
            // Serial.println("Sem->ready->quit");
        }
        else
        {
            if (millis() - _dcb._lastCallTick > 200) // millis, 200 - for a while
            {
                ++_dcb._errorCount;
            }
        }
        xSemaphoreGive(_trsm->Lock());
    } 
    else {
        //debug
        Serial.println("-It1notSem");
    }

    int len = lwip_recvfrom(_sock, _rcv->_payload, UFO_TRSM_BUFFER_SIZE - 1, 0, (struct sockaddr *)&_sourceAddr, &_socklen);
    if (len > 0)
    {
    // Serial.println("rsv->funk");
        _recvFunck(_rcv);
        if (_rcv->_errorCount > 0)
        {
            --_rcv->_errorCount;
        }
    }
    else
    {
        uint32_t ms = millis();
        if (ms - _rcv->_lastCallTick > 500)
        {
            ++_rcv->_errorCount;
        }
        _rcv->_lastCallTick = ms;
    }
    if (_sourceAddr.ss_family == PF_INET)
    {
        inet_ntoa_r(((struct sockaddr_in *)&_sourceAddr)->sin_addr, _addrStr, sizeof(_addrStr) - 1);
    }

    if (_trsm->GetDataBlock()._errorCount > 10) // 10 for a while
    {
        // _alarm.doSmth
        // func*   onSendError(); 
        Serial.println("Alarm1!");
    }
    if (_rcv->_errorCount > 4)      //4 for a while
    {
        Serial.println("Alarm2!");
        // func*   onRecvError(); 
    }
}

void Close()
{
    if (_sock != -1)
    {
        Serial.println("Shutting down socket");
        lwip_shutdown(_sock, 0);
        lwip_close(_sock);
    }
}
};