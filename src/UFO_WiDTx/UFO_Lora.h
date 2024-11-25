#pragma once

#include "UFO_WiDTxConf/UFO_LoraDefs.h"
#include "../UFO_TaskClassBase.h"

class UFO_Lora_e220440T22D : public UFO_TaskClassBase
{
private:
    
    UFO_TrsmControlBlock* _ctrlBlk;
    UFO_LoraSettings _conf;
    bool _ready = false;
    UFO_TrsmDataControlBlock* _rcv;
    std::function<void(UFO_TrsmDataControlBlock* rcv)> _callbackFunc;

public:
    UFO_Lora_e220440T22D(/* args */) {

        _rcv = new UFO_TrsmDataControlBlock();

    }
    ~UFO_Lora_e220440T22D() {
        if (!_rcv)
        {
            delete _rcv;
        }
    }

    void SetTrsmCB(UFO_TrsmControlBlock *loracb)
    {
        _ctrlBlk = loracb;
    }

    //255 - broadcast
    void SetAddr(uint8_t a){
        _conf._targAddr._addl =  a;
    }

    void SetChannel(uint8_t c){
        _conf._targAddr._chan = constrain(c, 0, 83);
    }

    void SetConfig(UFO_LoraConfigMinimal conf){
        _conf = conf._settings;
        _conf._targAddr._chan = constrain(_conf._targAddr._chan, 0, 83);
        _conf._selfAddr._chan = constrain(_conf._selfAddr._chan, 0, 83);
        _ctrlBlk = conf._ctrlBlk;
        _callbackFunc = conf._callback;
    }

    virtual void Setup() final {
        UFO_gpioConfig(UFO_LORA_AUX_PIN, gpio_mode_t::GPIO_MODE_INPUT);
        UFO_gpioConfig(UFO_LORA_M0_PIN, gpio_mode_t::GPIO_MODE_OUTPUT);
        UFO_gpioConfig(UFO_LORA_M1_PIN, gpio_mode_t::GPIO_MODE_OUTPUT);

        Serial.println("lora:: Setup start check");
       if (!_ctrlBlk)
        {
            Serial.println("\t\tError: _ctrlBlk is null");
            while (true)
            {
                Serial.println("Critical error");
                delay(500);
            }
            return;
        }
        if (!_ctrlBlk->Lock())
        {
            Serial.println("\t\tError: _ctrlBlk->lock is null");
            while (true)
            {
                Serial.println("Critical error");
                delay(500);
            }
            return;
        }
        
        if (!_callbackFunc)
        {
            Serial.println("\t\tError: no _callbackFunc func");
            // Serial.println("\t\t\tpassed...");
            while (true)
            {
                Serial.println("Critical error");
                delay(500);
            }
            return;
        }

        if (_conf._targAddr._chan > 84)
        {
            _conf._targAddr._chan = 20;
            Serial.println("Incorrect chanel; Setted to 20");
        }

        if (Serial2.baudRate() != 9600)
        {
            Serial.println("br isnt 9600");
        }
        

        Serial.println("lora:: Setup end check");

        Serial.println("lora:: Setup start");

        _conf.GenCfg();
        Serial.println("lora:: gen cfg");

        __SetMode(LORA_MODE_CMD);
        delay(10);
        Serial.println("lora:: set mode");
        
        __WriteCMD(_conf._cfg);
        Serial.println("lora:: write cmd");
        __ReadCMD();
        Serial.println("lora:: read cmd");

        delay(40);

        char c[] = { READ_CONFIGURATION, 0, 8};
        __WriteMSG(c, 3);
        __ReadCMD();            //todo (parse and compare)
        Serial.println("lora:: read cfg");

        __SetMode(LORA_MODE_NORMAL);
        delay(40);

        // todo: check if 
        Serial.println("lora:: Setup end");
    }
    
    virtual void Iteration () final {
        __ReadMSG();
        __WriteBlock();
        // char c[10] = {};
        // c[0] = 0xff;
        // c[1] = 0xff;
        // c[2] = 2;
        // c[3] = 'f';
        // c[4] = 'a';
        // c[5] = 'a';
        // c[6] = 'f';
        // __WriteMSG(c, 7);
    }
    

#pragma region
private:
    esp_err_t /*__attribute__((optimize("O0")))*/ __WaitAUX_True(int64_t ttime)
    {
        int64_t t = UFO_CoreTimeMilli();
        // make darn sure millis() is not about to reach max data type limit and start over
        if (((unsigned long)(t + ttime)) == 0)
        {
            t = 0;
        }

        while (gpio_get_level(UFO_LORA_AUX_PIN) == 0)
        {
            if ((UFO_CoreTimeMilli() - t) > ttime)
            {
                Serial.println("Timeout error!");
                return ESP_FAIL;
            }
            delay(1);
            // NOP();
        }
        return ESP_OK;
    }

    void __WriteCMD(char *cmd)
    {
        if (_conf._mode != LORA_MODE_CMD)
        {
            Serial.println("LORA ERROR: writing cmd when mode is not CMD");
            return;
        }
        
        if (gpio_get_level(UFO_LORA_AUX_PIN) == 0)
        {
            Serial.println("LORA ERROR: writing cmd when aux is 0");
            return;
        }

        char ccmd[11];
        ccmd[0] = UFO_LoraCMD::WRITE_CFG_PWR_DWN_SAVE;
        ccmd[1] = 0;
        ccmd[2] = 8;
        for (size_t i = 3; i < 11; i++)
        {
            ccmd[i] = cmd[i - 3ull];
        }
        __WriteMSG(ccmd, 11);
    }


    void __AddTargetAddr(char* s) {
        //WARNING 
        //_ctrlBlk->Lock() - should be locked before call this func
        s[0] = _conf._targAddr._addh;
        s[1] = _conf._targAddr._addl;
        s[2] = _conf._targAddr._chan;
    }

    int16_t __WriteBlock(){
        int16_t len = 0;
        if (xSemaphoreTake(_ctrlBlk->Lock(), 300))
        {
            UFO_TrsmDataControlBlock dcb = _ctrlBlk->GetDataBlock();
            if (dcb._ready)
            {
                __AddTargetAddr(dcb._payload);
 
                len = Serial2.write(dcb._payload, dcb._len);
                if (len != dcb._len)
                {
                    Serial.println("Error occurred during sending"); // add counter
                }
                dcb._ready = false;
                dcb._len = 0;
                // _ctrlBlk._data._lastCallTick = ...
            }
            xSemaphoreGive(_ctrlBlk->Lock());
            delay(25);
            __WaitAUX_True(500);
        }
        return len;
    }


    int16_t __WriteMSG(char* msg, uint32_t size){
        int16_t len = 0;
        if (xSemaphoreTake(_ctrlBlk->Lock(), 300))
        {
            len = Serial2.write(msg, size);
            if (len != size)
            {
                Serial.println("Error occurred during sending"); // add counter
            }
            xSemaphoreGive(_ctrlBlk->Lock());
            delay(25);
            __WaitAUX_True(500);
        }
        return len;
    }

    void __ReadMSG(){
        // int  i = Serial2.available();
        // if (i > 1)
        // {
        //     Serial.print("catch: ");
        //     // __Func();
        //     Serial.println(Serial2.readString());
        //     __WaitAUX_True(1000);
        // }
        // __FlushPort();

        memset(_rcv->_payload, 0,_rcv->_len);
        uint8_t i = 0;
        while (Serial2.available()>1)
        {
            _rcv->_payload[i] = Serial2.read();
            ++i;
        }
        _rcv->_len = i;
        __WaitAUX_True(1000);
        __FlushPort();
        if (i > 0)
        {
            _callbackFunc(_rcv);
        }
        
    }

    void __ReadCMD(){
        char buf[11]; // for a while
        if (Serial2.available() > 1)
        {
            Serial2.readBytes(buf, 11);
        }
        Serial.printf("recv: %s\n", buf);
        __FlushPort();
    }

    void __FlushPort(){
        while (Serial2.available() > 0)
        {
            Serial2.read();
        }
    }

    void __SetMode(UFO_LoraMode m)
    {
        _conf._mode = m;
        switch (_conf._mode)
        {
        case LORA_MODE_NORMAL:
            gpio_set_level(UFO_LORA_M0_PIN, 0);
            gpio_set_level(UFO_LORA_M1_PIN, 0);
            break;
        case LORA_MODE_CMD:
            gpio_set_level(UFO_LORA_M0_PIN, 1);
            gpio_set_level(UFO_LORA_M1_PIN, 1);
            break;
        case LORA_MODE_WORTX:
            // todo
            break;
        default:
            gpio_set_level(UFO_LORA_M0_PIN, 0);
            gpio_set_level(UFO_LORA_M1_PIN, 0);
            break;
        }
        delay(40);
        __WaitAUX_True(200);
    }

#pragma endregion 
};
