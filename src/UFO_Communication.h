#pragma once

#include "UFO_TaskClassBase.h"
#include "UFO_Config.h"

#define UFO_UART_INTERFACE_COUNT (3+1)

enum UFO_Communication_InterfaceGroup {
    RXIG_NULL,
    RXIG_ON_AIR,
    RXIG_UART,

    RXIG_COUNT,
};

enum UFO_Communication_InterfaceType {
    RIOT_NULL,
    RIOT_WIFI,
    RIOT_LORA,      // not impl
    RIOT_MSP,
    RIOT_MAVLINK,    // not impl
    RIOT_IBUS,       // not impl
    RIOT_SBUS,       // not impl
};

struct UFO_CommunicationTimePoints {
    time_t _last = 0;
    time_t _interval = 0;
};


//----------------
#define UFO_PDOA_CODE_NAME_SIZE     4
#define UFO_PDOA_LENTH_SIZE         1
#define UFO_PDOA_CHECKSUM_SIZE      4
#define UFO_PDOA_ITEM_MAX_COUNT     8
#define UFO_PDOA_ITEM_COUNT_SIZE    1

#define UFO_PDOA_ITEM_LENTH_SIZE    1
#define UFO_PDOA_ITEM_TYPE_SIZE     1
#define UFO_PDOA_ITEM_PREFIX_SIZE   1
#define UFO_PDOA_CALC_ITEM_LENTH (i) (i+UFO_PDOA_ITEM_TYPE_SIZE+UFO_PDOA_ITEM_LENTH_SIZE + UFO_PDOA_ITEM_PREFIX_SIZE)
#define UFO_PDOA_CALC_PACKET_LENTH (item_packs_lenth) (item_packs_lenth + UFO_PDOA_CODE_NAME_SIZE +UFO_PDOA_LENTH_SIZE + UFO_PDOA_CHECKSUM_SIZE+UFO_PDOA_ITEM_COUNT_SIZE)

//---

//some enum of cmds list
enum UFO_CommnunicationCmdList:uint8_t {
    //todo do commands like in msp
};
// PDOA 
enum UFO_PDOA_RESULT {
    SUC,
    UNDEF,
    PACKET_OVERFLOW,
};

class UFO_PDOA
{
private:
    enum : uint8_t{
        MSG_TYPE_NULL = 0,
        MSG_TYPE_INT,
        MSG_TYPE_FLOAT,
        MSG_TYPE_BOOL,
        MSG_TYPE_STR,
        MSG_TYPE_CMD
    };
    String _packet;
    bool _ready = false; 
    String _items[UFO_PDOA_ITEM_MAX_COUNT];
    int _itemIter = 0;
    char _codeName[4] = {'a', 'b', 'c', 'd'};
public:
    UFO_PDOA(/* args */) {}
    ~UFO_PDOA() {}
    
    void SetCodeName(char _1, char _2, char _3, char _4){
        _codeName[0] = _1; 
        _codeName[1] = _2; 
        _codeName[2] = _3; 
        _codeName[3] = _4; 
    }

    bool CheckOverflow(){
        return _itemIter > UFO_PDOA_ITEM_MAX_COUNT;
    }

    void ClearAll(){
        _packet.clear();

        for (uint8_t i = 0; i < UFO_PDOA_ITEM_MAX_COUNT; ++i)
        {
            _items[i].clear();
        }
        
    }

    UFO_PDOA_RESULT CreateItem(int val)
    {
        _ready = false;
        if (CheckOverflow())
        {
            return PACKET_OVERFLOW;
        }
        String prepVal = String(val);
        _items[_itemIter++] = String(static_cast<char>(prepVal.length())) + prepVal + String(static_cast<char>(MSG_TYPE_INT));
        return SUC;
    }

    UFO_PDOA_RESULT CreateItem(bool val)
    {
        _ready = false;
        if (CheckOverflow())
        {
            return PACKET_OVERFLOW;
        }
        String prepVal = String(val);
        _items[_itemIter++] = String(static_cast<char>(prepVal.length())) + prepVal + String(static_cast<char>(MSG_TYPE_BOOL));
        return SUC;
    }
    UFO_PDOA_RESULT CreateItem(float val)
    {
        _ready = false;
        if (CheckOverflow())
        {
            return PACKET_OVERFLOW;
        }
        String prepVal = String(val, 3);
        _items[_itemIter++] = String(static_cast<char>(prepVal.length())) + prepVal + String(static_cast<char>(MSG_TYPE_FLOAT));
        return SUC;
    }

    
    UFO_PDOA_RESULT CreateItem(String val)
    {
        _ready = false;
        if (CheckOverflow())
        {
            return PACKET_OVERFLOW;
        }
        if (val.length() > 30)
        {
            return UNDEF;
        }
        
        _items[_itemIter++] = String(static_cast<char>(val.length())) + val + String(static_cast<char>(MSG_TYPE_STR));
        // Serial.println(val);
        // Serial.println(val.length());
        // Serial.println(_items[_itemIter++]);
        // Serial.println(_items[_itemIter++].length());
        return SUC;
    }

    UFO_PDOA_RESULT CreateItem(UFO_CommnunicationCmdList cmd){
        _ready = false;
        if (CheckOverflow())
        {
            return PACKET_OVERFLOW;
        }
        _items[_itemIter++] = String(static_cast<char>(1)) + String(static_cast<char>(cmd)) + String(static_cast<char>(MSG_TYPE_STR));
        return SUC;
    }

    UFO_PDOA_RESULT CreatePacket(){
        _packet = String(_codeName);
        uint8_t packLenth = 0;
        uint8_t packCount = _itemIter;
        _itemIter = 0;
        
        for (uint8_t i = 0; i < packCount; ++i)
        {
            packLenth+= static_cast<int8_t>(_items[i][0]);
        }
        _packet += static_cast<char>(packLenth);
        _packet += static_cast<char>(packCount);
        for (uint8_t i = 0; i < packCount; ++i)
        {
            _packet+= _items[i];
        }

        
        Serial.print("Packet Count: ");
        Serial.print(packCount);
        Serial.print(" Lenth: ");
        Serial.println(packLenth);
        
        
        _packet+= "0000";
        _ready = true;
        return SUC;
    }

    String& GetPacket(){
        if (!_ready)
        {
            _packet = String();
        }
        return _packet;
    }

    bool AreItemsFit(int8_t itemCount){
        return (UFO_PDOA_ITEM_MAX_COUNT - _itemIter) > itemCount;
    }
    int8_t GetCountOfFreeItems(){
        return (UFO_PDOA_ITEM_MAX_COUNT - _itemIter);
    }
    
};


class UFO_Communication_InterfaceMinamal
{
private:
    UFO_Communication_InterfaceGroup _group  = RXIG_NULL;
    UFO_Communication_InterfaceType _type = RIOT_NULL;
    String _packetRecieved;
    String _packetToSend;
    UFO_CommunicationTimePoints _timeRx;
    UFO_CommunicationTimePoints _timeTx;
    bool _bidirectional = false;
    uint8_t _errorRxCount = 0;
    uint8_t _errorTxCount = 0;
    
public:
    UFO_Communication_InterfaceMinamal(/* args */) {}
    ~UFO_Communication_InterfaceMinamal() {}

    virtual void Send(/* data, lenth,...*/) = 0; 
    virtual void Recieve(/* where */) = 0; 
};



class UFO_RC : public UFO_TaskClassBase
{
private:

public:
    UFO_RC(/* args */) {}
    ~UFO_RC() {}

    void Setup() final {

    }

    void Iteration() final {

    }
    
};