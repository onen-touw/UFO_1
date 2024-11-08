#pragma once
#include "UFO_Config.h"

#include <iostream>

enum TX_TYPE : uint8_t
{
    TX_NONE = '\0',
    TX_GYRO = 'G',
};

enum RX_TYPE : uint8_t
{
    RX_NONE = '\0',
    RX_FLY_PARAM = 'F',
    RX_SETTING = 'S',
};

enum TX_INDEX : uint8_t
{
    TX_IND_NONE = '\0',
    TX_IND_PITCH = 'P',
    TX_IND_YAW = 'Y',
    TX_IND_ROLL = 'R',
};

enum RX_INDEX : uint8_t
{
    RX_IND_NONE = '\0',
    RX_IND_THROTTLE = 'T',
    RX_IND_PITCH = 'P',
    RX_IND_YAW = 'Y',
    RX_IND_ROLL = 'R',
};

class TxDataHandler
{
public:
    TxDataHandler()
    {
    }

    ~TxDataHandler()
    {
        if (isCreated)
        {
            delete[] packet;
        }
    }

    String ConfirmPacket()
    {
        String toSend;
        toSend += (static_cast<char>(this->type));
        // toSend += ();
        toSend += '{';
        for (size_t i = 0; i < readyElCount; ++i)
        {
            toSend+=(static_cast<char>(packet[i].index));
            // toSend += (static_cast<unsigned char>(packet[i].index));
            toSend += ":";
            toSend += String(packet[i].value);
            toSend += ";";
        }
        toSend += "};";
        readyElCount = 0;
        return toSend;
    }

    bool AddToPacket(TX_INDEX index, int32_t val)
    {
        if (readyElCount >= size)
        {
            return false;
        }
        packet[readyElCount].index = index;
        packet[readyElCount].value = val;
        ++readyElCount;
        return true;
    }

    void CreatePacket(int32_t size, TX_TYPE type)
    {
        if (!isCreated)
        {
            this->size = size;
            packet = new _Data[this->size];
            isCreated = true;
        }
        else
        {
            if (this->size < size)
            {
                this->size = size;
                delete[] packet;
                packet = new _Data[size];
            }
        }
        this->type = type;
    }

private:
    struct _Data
    {
        TX_INDEX index = TX_INDEX::TX_IND_NONE;
        int32_t value = 0;
    };

    bool isCreated = false;
    int32_t size = 0;
    int32_t readyElCount = 0;
    _Data *packet = nullptr;
    TX_TYPE type = TX_TYPE::TX_NONE;
};

class RxDataHandler
{

private: 
    int32_t iter = 0;
    bool isValid = true;
    RX_TYPE curType = RX_TYPE::RX_NONE;
    // int32_t itemCount = 0;
    std::function<void(RX_INDEX, int32_t)> handlFunck;

public:
    RxDataHandler() {}
    RxDataHandler(std::function<void(RX_INDEX, int32_t)> funck) : handlFunck(funck) {}

    ~RxDataHandler() {}

    void Addhandler(std::function<void(RX_INDEX, int32_t)> funck)
    {
        handlFunck = funck;
    }

    void operator()(String &str)
    {
        iter = 0;
        if (!str.length())
        {
            isValid = false;
            return;
        }

        while (str[iter] != ';')
        {
            if (!isValid)
            {
                break;
            }
            _ParseType(str);
            _ParsePacket(str);
        }
    }

    bool valid() const
    {
        return isValid;
    }


private:

    

    void _ParseType(String &str)
    {

        curType = static_cast<RX_TYPE>(str[iter]);

        if (str[++iter] != '{')
        {
            isValid = false;
        }
        ++iter;
    }

    void _ParseItem(String &str)
    {
        if (!isValid)
        {
            return;
        }
        RX_INDEX ind = _ParseIndex(str);
        int32_t val = _ParseValue(str);
        handlFunck(ind, val);

        // std::cout << (char)ind << "\t" << val << "\n";
        // std::cout << "=============\n";
    }

    void _ParsePacket(String &str)
    {
        if (!isValid)
        {
            return;
        }
        while (str[iter] != '}')
        {
            _ParseItem(str);
            if (!isValid)
            {
                return;
            }
        }
        ++iter;
    }

    RX_INDEX _ParseIndex(String &str)
    {
        if (!isValid)
        {
            return RX_INDEX::RX_IND_NONE;
        }

        RX_INDEX curIndex = static_cast<RX_INDEX>(str[iter]);
        if (str[++iter] != ':')
        {
            isValid = false;
        }
        ++iter;
        return curIndex;
    }

    int32_t _ParseValue(String &str)
    {
        if (!isValid)
        {
            return INT32_MIN;
        }
        char ch = str[iter];
        int32_t val = 0;
        while (ch != ';')
        {
            val *= 10;
            val += (ch - '0');
            ch = str[++iter];
        }
        ++iter;

        return val;
    }


};
