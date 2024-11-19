#pragma once 

#include "UFO_Config.h"
#include "driver/uart.h"


// todo pins

struct UFO_UartPins
{
    gpio_num_t _tx = gpio_num_t::GPIO_NUM_NC;
    gpio_num_t _rx = gpio_num_t::GPIO_NUM_NC;
};
#define UFO_UART_RX_BUF_SIZE 255

struct UFO_UartMinimal
{
    QueueHandle_t _eventQueue;   
    SemaphoreHandle_t _lock;
};


class UFO_Uart
{
private:
    int8_t _unum = -1;
    UFO_UartPins _pins;
    UFO_UartMinimal _minimal;
    // uint32_t _baud = 0;
    uart_config_t _conf;
    TaskHandle_t _task;
    bool _inited = false;

public:
    UFO_Uart(const UFO_Uart&) = delete;
    UFO_Uart& operator = (const UFO_Uart &) = delete;

    UFO_Uart() {
        if (_inited)
        {
            return;
        }
        if (!_minimal._lock)
        {
            _minimal._lock = xSemaphoreCreateMutex();
        }
    }
    ~UFO_Uart() {}

    // do not use in task
    esp_err_t Init(int8_t unum)
    {
        esp_err_t err = ESP_OK;

        if (xSemaphoreTake(_minimal._lock, 1000))
        {
            if (_inited)
            {
                return ESP_FAIL;
            }

            if (uart_is_driver_installed(_unum))
            {
                // Set critical
                return ESP_FAIL;
            }

            uart_config_t uart_config;
            uart_config.baud_rate = 115200,
            uart_config.data_bits = UART_DATA_8_BITS,
            uart_config.parity = UART_PARITY_DISABLE,
            uart_config.stop_bits = UART_STOP_BITS_1,
            uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            uart_config.source_clk = UART_SCLK_APB; // ESP32, ESP32S2

            err = uart_driver_install(_unum, UFO_UART_RX_BUF_SIZE, 0, 0, NULL, 0);
            if (err != ESP_OK)
            {
                // serCritical
                return err;
            }

            err = uart_param_config(_unum, &uart_config);
            if (err != ESP_OK)
            {
                // serCritical
                return err;
            }

            err = uart_set_pin(UART_NUM_1, _pins._tx, _pins._rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
            if (err != ESP_OK)
            {
                // serCritical
                return err;
            }
            xSemaphoreGive(_minimal._lock);
        }

        if (err != ESP_OK)
        {
            // setCritical
        }

        // xTaskCreatePinnedToCore( _EventTask, "uartETask", 2048, this, 5, &_task, 0); //for what???
        return err;
    }

    int16_t Write(char* data){
        uint16_t l = 0;
        uint16_t sz = strlen(data);
        // some checks
        if (xSemaphoreTake(_minimal._lock, 1000))
        {
            l = uart_write_bytes(_unum, data, sz);
            xSemaphoreGive(_minimal._lock);
        }
        return l;     
    }

    int16_t Write(char *data, uint16_t sz)
    {
        uint16_t l = 0;
        if (xSemaphoreTake(_minimal._lock, 1000))
        {
            l = uart_write_bytes(_unum, data, sz);
            xSemaphoreGive(_minimal._lock);
        }
        return l;
    }

    uint16_t Available(){

        size_t av = 0;
        if (xSemaphoreTake(_minimal._lock, 1000)){
            uart_get_buffered_data_len(_unum, &av);
            //what about peek
            xSemaphoreGive(_minimal._lock);
        }

        return av;
    }


private:

    static void _EventTask(void* arg){
        // pointer
        UFO_Uart* u = reinterpret_cast<UFO_Uart*>(arg);
        uart_event_t e;
        QueueHandle_t q = u->_minimal._eventQueue;
        if (q != nullptr) {
            while (true)
            {
                if (xQueueReceive(q, &e, 5000))
                {
                    switch (e.type)
                    {
                    case UART_DATA:
                        //what??
                        break;
                    
                    default:
                        break;
                    }
                }
            }
        }
    }
};