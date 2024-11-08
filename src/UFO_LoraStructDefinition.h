#pragma once

#include "UFO_Config.h"
#include "UFO_Utils.h"

#define UFO_LORA_BUFFER_SIZE 180
#define UFO_LORA_M0_PIN     (gpio_num_t::GPIO_NUM_2)
#define UFO_LORA_M1_PIN     (gpio_num_t::GPIO_NUM_5)
#define UFO_LORA_AUX_PIN    (gpio_num_t::GPIO_NUM_4)


enum UFO_LoraResponseStatus : uint8_t
{
    LORA_SUCCESS = 1,
    LORA_SUCCESS = 1,
    LORA_ERR_UNKNOWN, /* something shouldn't happened */
    LORA_ERR_NOT_SUPPORT,
    LORA_ERR_NOT_IMPLEMENT,
    LORA_ERR_NOT_INITIAL,
    LORA_ERR_INVALID_PARAM,
    LORA_ERR_DATA_SIZE_NOT_MATCH,
    LORA_ERR_BUF_TOO_SMALL,
    LORA_ERR_TIMEOUT,
    LORA_ERR_HARDWARE,
    LORA_ERR_HEAD_NOT_RECOGNIZED,
    LORA_ERR_NO_RESPONSE_FROM_DEVICE,
    LORA_ERR_WRONG_UART_CONFIG,
    LORA_ERR_WRONG_FORMAT,
    LORA_ERR_PACKET_TOO_BIG
};

enum UFO_LoraSerialBoundRate : uint32_t
{
    LORA_BDRT_1200 = 1200,
    LORA_BDRT_2400 = 2400,
    LORA_BDRT_4800 = 4800,
    LORA_BDRT_9600 = 9600,
    LORA_BDRT_19200 = 19200,
    LORA_BDRT_38400 = 38400,
    LORA_BDRT_57600 = 57600,
    LORA_BDRT_115200 = 115200,
};

enum UFO_LoraParity : uint8_t
{
    LORA_MODE_00_8N1 = 0b00,
    LORA_MODE_01_8O1 = 0b01,
    LORA_MODE_10_8E1 = 0b10,
    LORA_MODE_11_8N1 = 0b11
};

enum UFO_LoraSubpacketSetting : uint8_t
{
    LORA_SPS_200_00 = 0b00,
    LORA_SPS_128_01 = 0b01,
    LORA_SPS_064_10 = 0b10,
    LORA_SPS_032_11 = 0b11
};

// todo
enum UFO_LoraRSSI_AmbientNoise : uint8_t
{
    LORA_RSSI_AMBIENT_NOISE_ENABLED = 0b1,
    LORA_RSSI_AMBIENT_NOISE_DISABLED = 0b0
};

enum UFO_LoraWOR_Period : uint8_t
{
    LORA_WOR_500_000 = 0b000,
    LORA_WOR_1000_001 = 0b001,
    LORA_WOR_1500_010 = 0b010,
    LORA_WOR_2000_011 = 0b011,
    LORA_WOR_2500_100 = 0b100,
    LORA_WOR_3000_101 = 0b101,
    LORA_WOR_3500_110 = 0b110,
    LORA_WOR_4000_111 = 0b111
};

enum UFO_LoraTransmitionPower : uint8_t
{
    LORA_POWER_22dbm = 0b00,
    LORA_POWER_17dbm = 0b01,
    LORA_POWER_13dbm = 0b10,
    LORA_POWER_10dbm = 0b11
};

enum REGISTER_ADDRESS : uint8_t{
	LORA_REG_ADDRESS_CFG			= 0x00,
	LORA_REG_ADDRESS_SPED 		    = 0x02,
	LORA_REG_ADDRESS_TRANS_MODE 	= 0x03,
	LORA_REG_ADDRESS_CHANNEL 	    = 0x04,
	LORA_REG_ADDRESS_OPTION	 	    = 0x05,
	LORA_REG_ADDRESS_CRYPT	 	    = 0x06,
	LORA_REG_ADDRESS_PID		 	= 0x08
};




struct UFO_LoraDataCB
{
    char _payload[UFO_LORA_BUFFER_SIZE];
    uint8_t _len = 0;
    uint8_t _errorCount = 0;
    int32_t _lastCallTick = 0;

    
};

struct UFO_LoraTargetInfo
{
    char _addrHigh = ' ';
    char _addrLow = ' ';
    char _chan = 0;
    
};

struct UFO_LoraConfig
{

    
};

struct UFO_LoraControlBlock
{
    UFO_LoraTargetInfo _target;
    SemaphoreHandle_t _lock;
    UFO_LoraConfig _conf;
    UFO_LoraDataCB _data;
    bool _ready = false;

    bool Msg(const char *pl)
    {
        int16_t l = strlen(pl);
        if (l > UFO_LORA_BUFFER_SIZE)
        {
            return false; // false if overflow
        }
        if (xSemaphoreTake(_lock, TickType_t(1000)) == pdTRUE)
        {
            memcpy(_data._payload, pl, l);
            _data._len = l;
            _ready = true;
            xSemaphoreGive(_lock);
        }
        return true;
    }

    UFO_LoraControlBlock()
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

     ~UFO_LoraControlBlock()
    {
        if (_lock)
        {
            vSemaphoreDelete(_lock);
        }
    }
};