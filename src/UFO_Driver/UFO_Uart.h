#pragma once 

#include "UFO_Config.h"
#include "driver/uart.h"
#include "hal/uart_ll.h"
#include <iostream>


// todo pins
// #define U0TXD_GPIO_NUM  (1)
// #define U0RXD_GPIO_NUM  (3)
// #define U0CTS_GPIO_NUM  (19)
// #define U0RTS_GPIO_NUM  (22)

// #define U1TXD_GPIO_NUM  (10)
// #define U1RXD_GPIO_NUM  (9)
// #define U1CTS_GPIO_NUM  (6)
// #define U1RTS_GPIO_NUM  (11)

// #define U2TXD_GPIO_NUM  (17)
// #define U2RXD_GPIO_NUM  (16)
// #define U2CTS_GPIO_NUM  (8)
// #define U2RTS_GPIO_NUM  (7)
struct UFO_UartPins
{
    gpio_num_t _tx = gpio_num_t::GPIO_NUM_NC;
    gpio_num_t _rx = gpio_num_t::GPIO_NUM_NC;
};
#define UFO_UART_RX_BUF_SIZE (256*2)
// #define UFO_UART_EVENTS


struct UFO_UartMinimal
{
    QueueHandle_t _eventQueue;   
    SemaphoreHandle_t _lock;
#ifdef UFO_UART_EVENTS
    TaskHandle_t _task;
#endif
};



class UFO_Uart
{
private:
    int8_t _unum = -1;
    UFO_UartPins _pins;
    UFO_UartMinimal _minimal;
    bool _inited = false;   
    uint32_t _timeoutMs = 100;      //base
    uint8_t _timeoutRx = 2;         //base
    uint8_t _txFiFoFull = 120;      //base

public:
    UFO_Uart(const UFO_Uart&) = delete;
    UFO_Uart& operator = (const UFO_Uart &) = delete;

    UFO_Uart(int8_t unum = 0)
    {
        if (_inited)
        {
            return;
        }

        if (unum < 0 || unum > UART_NUM_MAX - 1)
        {
            return;
        }

        if (!_minimal._lock)
        {
            _minimal._lock = xSemaphoreCreateMutex();
        }
        _unum = unum;
    }

    ~UFO_Uart() {
        if (_inited)
        {
            Deinit();
            // __Deinit();
        }
        
        if (_minimal._lock)
        {
            vSemaphoreDelete(_minimal._lock);
        }
    }

    // do not use in task
    esp_err_t Init()
    {
        esp_err_t err = ESP_OK;

        if (_unum < 0)
        {
            return ESP_FAIL;
        }
        if (_inited)
        {
            return ESP_FAIL;
        }
        
        if (xSemaphoreTake(_minimal._lock, 1000))
        {
            err = __Init();   
            xSemaphoreGive(_minimal._lock);
        }

        if (err!= ESP_FAIL)
        {
            _inited = true;
        }
        #ifdef UFO_UART_EVENTS
        // xTaskCreatePinnedToCore( _EventTask, "uartETask", 2048, this, 5, &_minimal._task, 0); //for what???
        #endif


        return err;
    }

    size_t Send(char* data){
        size_t l = 0;
        size_t sz = strlen(data);
        // some checks
        if (xSemaphoreTake(_minimal._lock, 1000))
        {
            l = uart_write_bytes(_unum, data, sz);
            xSemaphoreGive(_minimal._lock);
        }
        return l;     
    }
    
    esp_err_t SetBaudRate(uint32_t br){
        esp_err_t err = ESP_OK;
        if (xSemaphoreTake(_minimal._lock, 1000))
        {
            if (!_inited)
            {
                err = ESP_FAIL;
                xSemaphoreGive(_minimal._lock);
                return err;
            }
            err = uart_set_baudrate(_unum, br);
            xSemaphoreGive(_minimal._lock);
        }
        return err;
    }

    size_t SendWithBreak(const char* data){
        size_t l = 0;
        size_t sz = strlen(data);
        // some checks
        if (xSemaphoreTake(_minimal._lock, 1000))
        {
            l = uart_write_bytes_with_break(_unum, data, sz, 1);
            xSemaphoreGive(_minimal._lock);
        }
        return l;     
    }
    size_t SendMsg(const char* data){
        size_t l = 0;
        size_t sz = strlen(data);
        // some checks
        if (xSemaphoreTake(_minimal._lock, 1000))
        {
            l = uart_write_bytes(_unum, data, sz);
            uart_wait_tx_done (_unum, (TickType_t)100);
            xSemaphoreGive(_minimal._lock);
        }
        return l;     
    }


    // The write-function does not guarantee that this msg (data) will be
    // send independently on other trash(other data) in txFiFo buffer
    size_t Write(const char *data)
    {
        size_t l = 0;
        size_t sz = strlen(data);
        // some checks
        if (xSemaphoreTake(_minimal._lock, 1000))
        {
            l = uart_write_bytes(_unum, data, sz);
            xSemaphoreGive(_minimal._lock);
        }
        return l;
    }

    size_t Send(char *data, size_t sz)
    {
        uint16_t l = 0;
        if (xSemaphoreTake(_minimal._lock, 1000))
        {
            l = uart_write_bytes(_unum, data, sz);
            xSemaphoreGive(_minimal._lock);
        }
        return l;
    }

    size_t Available(){
        size_t av = 0;
        if (xSemaphoreTake(_minimal._lock, 1000)){
            uart_get_buffered_data_len(_unum, &av);
            //what about peek
            xSemaphoreGive(_minimal._lock);
        }
        return av;
    }

    size_t Read(char* buff, size_t sz){
        
        size_t w = 0;
        if (xSemaphoreTake(_minimal._lock, 1000)){
            w = uart_read_bytes(_unum, buff, sz, pdMS_TO_TICKS(_timeoutMs));
            //what about peek
            xSemaphoreGive(_minimal._lock);
        }
        return w;
    }

    size_t Read(char *buff)
    {

        size_t w = 0, av = 0;

        if (xSemaphoreTake(_minimal._lock, 1000))
        {
            uart_get_buffered_data_len(_unum, &av);;
            w = uart_read_bytes(_unum, buff, av, pdMS_TO_TICKS(_timeoutMs));
            // what about peek
            xSemaphoreGive(_minimal._lock);
        }
        return w;
    }

    void Flush(){
        if (xSemaphoreTake(_minimal._lock, 1000))
        {
            while (!uart_ll_is_tx_idle(UART_LL_GET_HW(_unum)))
            {}
            uart_flush_input(_unum);
            xSemaphoreGive(_minimal._lock);
        }
    }

    void Deinit(){
        if (xSemaphoreTake(_minimal._lock, 1000))
        {
            if (_inited)
            {
                Flush();
                uart_driver_delete(_unum);
            }
            #ifdef UFO_UART_EVENTS
                vTaskDelete(_minimal._task);
            #endif

            xSemaphoreGive(_minimal._lock);
        }
    }

private:

    esp_err_t __Init(){
        esp_err_t err = ESP_OK;

        if (uart_is_driver_installed(_unum))
        {
            return ESP_FAIL;
        }
        std::cout << "starting uart install\n";
        std::cout << "uart number: " << (int)_unum << '\n';
        delay(200);
        err = uart_driver_install(_unum, UFO_UART_RX_BUF_SIZE, 0, 0, nullptr, 0);
        // err = uart_driver_install(_unum, UFO_UART_RX_BUF_SIZE * 2, 0, 20, &_minimal._eventQueue, 0);
        if (err != ESP_OK)
        {
            std::cout << "ufo_uart::uart_driver_install error\n"; // for a while (out in uart0)
            // serCritical
            return err;
        }
        std::cout << "1\n";

        uart_config_t ucfg;
        ucfg.baud_rate = 115200,
        ucfg.data_bits = UART_DATA_8_BITS,
        ucfg.parity = UART_PARITY_DISABLE,
        ucfg.stop_bits = UART_STOP_BITS_1,
        ucfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        ucfg.source_clk = UART_SCLK_APB; // ESP32, ESP32S2
        ucfg.rx_flow_ctrl_thresh = 112;
        err = uart_param_config(_unum, &ucfg);
        if (err != ESP_OK)
        {
            std::cout << "ufo_uart::uart_param_config error\n"; // for a while (out in uart0)
            // serCritical
            return err;
        }
        std::cout << "2\n";
        
        if (_pins._rx < 0 || _pins._tx < 0)
        {
            err = __PinCfg();
        }
        if (err != ESP_OK)
        {
            std::cout << "ufo_uart::__PinCfg error\n"; // for a while (out in uart0)
            // serCritical
            return err;
        }
        err = uart_set_pin(_unum, _pins._tx, _pins._rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        if (err != ESP_OK)
        {
            std::cout << "ufo_uart::uart_set_pin error\n"; // for a while (out in uart0)
            // serCritical
            return err;
        }
        std::cout << "3\n";

        err = uart_set_rx_timeout(_unum, _timeoutRx);
        if (err != ESP_OK)
        {
            std::cout << "ufo_uart::uart_set_rx_timeout error\n"; // for a while (out in uart0)
            // serCritical
            return err;
        }
        std::cout << "4\n";

        err = uart_set_rx_full_threshold(_unum, _txFiFoFull);
        if (err != ESP_OK)
        {
            std::cout << "ufo_uart::uart_set_rx_full_threshold error\n"; // for a while (out in uart0)
            // serCritical
            return err;
        }
        std::cout << "5\n";
        return err;
    }
    
    esp_err_t __PinCfg(){
        esp_err_t e = ESP_OK;
        switch (_unum)
        {
        case UART_NUM_0:
            _pins._rx = static_cast<gpio_num_t>(U0RXD_GPIO_NUM);
            _pins._tx = static_cast<gpio_num_t>(U0TXD_GPIO_NUM);
            break;
        case UART_NUM_1:
            return ESP_FAIL; // temp 
            // _pins._rx = static_cast<gpio_num_t>(U1RXD_GPIO_NUM);    16,17
            // _pins._tx = static_cast<gpio_num_t>(U1TXD_GPIO_NUM);
            break;
        case UART_NUM_2:
            _pins._rx = static_cast<gpio_num_t>(U2RXD_GPIO_NUM);
            _pins._tx = static_cast<gpio_num_t>(U2TXD_GPIO_NUM);
            break;
        
        default:
            e = ESP_FAIL;
            break;
        }
        return e;
    }

#ifdef UFO_UART_EVENTS
    static void __EventTask(void* arg){
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
#endif
    
};