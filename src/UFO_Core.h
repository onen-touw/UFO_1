#pragma once
#include <iostream>

#include "UFO_Config.h"
#include "UFO_Utils.h"

#include "UFO_StructDefinition.h"

#include "UFO_Driver/UFO_I2C_driver.h"
#include "UFO_Driver/UFO_Uart.h"
// #include "UFO_Error.h"
#include "UFO_WiFi.h"

/*
type :
    uint8_t 
bit num:    0       1       2           3           4       5       6           7
flag bit:   0       0       0           0           0       0       0           0
function:   wifi    i2c     hw_uart1    hw_uart2    sock    lora    debug   debColor
TODO place for SD flag
TODO place for web page flag
*/
enum UFO_AppFlag : uint8_t {
    use_wifi            = UFO_SetBit(0),        // basic
    use_i2c             = UFO_SetBit(1),        // basic
    use_hw_uart1        = UFO_SetBit(2),        
    use_hw_uart2        = UFO_SetBit(3),
    use_sock            = UFO_SetBit(4),        // sock channel (to RC, other esp or PC)
    use_lora            = UFO_SetBit(5),        // to RC only
    use_debug           = UFO_SetBit(6),        // lora rc?  sock rc? uart PC? sock PC?
    use_debugColorful   = UFO_SetBit(7),
    // use_SD              = UFO_SetBit(7), // TODO
};
enum class UFO_AppBiNum : uint8_t {
    wifi_bit            = 0,
    i2c_bit             ,
    uart1_bit           ,
    uart2_bit           ,
    sock_bit            ,
    lora_bit            ,
    debug_bit           ,
    debugColorful_bit   ,
};



// TODO to  config
#define UFO_USE_WIFI
#define UFO_USE_HW_UART


#ifdef UFO_USE_HW_UART
struct UFO_HW_Uart {
    UFO_Uart* _uart1 = nullptr;  // TODO set pins for uart1 ??(19,18)??
    UFO_Uart* _uart2 = nullptr; // tx2, rx2 (16,17)
};
#endif


class UFO_Core /*   :TODO private  UFO_traceback*/
{
private:
    TaskHandle_t _idle = nullptr;
    
    UFO_I2C_Driver _i2cDriver;
    UFO_Uart* _port = nullptr;  // tx, rx pin

    // UFO_ErrorControlBlock _e;

#ifdef UFO_USE_HW_UART
    UFO_HW_Uart _hwPort;
#endif

private:
    UFO_Core(uint8_t f) {
        uint8_t state = 0b0;

        std::cout << "core:: start creating\n";
        

        std::cout << "core:: port createding\n";
        _port = new UFO_Uart(0);
        if (_port != nullptr){
            std::cout << "core:: \t\tport created\n";
        }
        
        std::cout << "core:: checking flag\n";

        if ((f >> static_cast<uint8_t>(UFO_AppBiNum::debug_bit)) && 0b1)
        {
            std::cout << "core:: \t\tflag: debug detected\n";
            // init trsm control block
            // create sock task
        }

        if ((f >> static_cast<uint8_t>(UFO_AppBiNum::i2c_bit)) && 0b1)
        {
            std::cout << "core:: \t\tflag: use_i2c detected\n";

            // TODO trace
            esp_err_t err = _i2cDriver.Init(UFO_I2C_port::UFO_I2C_HARDWARE);
            if (err!=ESP_FAIL)
            {
                std::cout << "core:: \t\ti2c driver started\n";
            }
            
        }

#ifdef UFO_USE_HW_UART  // TODO test it
        std::cout << "core:: hw_uart:\n";

        if ((f >> static_cast<uint8_t>(UFO_AppBiNum::uart1_bit)) && 0b1)
        {
            std::cout << "core:: flag: hw_uart1 detected\n";
            
            // TODO trace
            _hwPort._uart1 = new UFO_Uart(1);
            if (_hwPort._uart1 != nullptr)
            {
                std::cout << "core:: hw_uart1 driver started\n";
                // _e.error();
                // _e.trace;
            }
            
        }
        if ((f >> static_cast<uint8_t>(UFO_AppBiNum::uart2_bit)) && 0b1)
        {
            std::cout << "core:: flag: hw_uart2 detected\n";
            // TODO trace
            _hwPort._uart2 = new UFO_Uart(2);
            if (_hwPort._uart2 != nullptr)
            {
                std::cout << "core:: hw_uart1 driver started\n";
                // _e.error();
                // _e.trace;
            }
        }
#endif

#ifdef UFO_USE_WIFI
        std::cout << "core:: wifi:\n";

        bool wf_flag = false;
        if ((f >> static_cast<uint8_t>(UFO_AppBiNum::wifi_bit)) && 0b1 )
        {
            std::cout << "core:: flag: wifi detected\n";
            UFO_WiFi_t wf = UFO_WiFi_GetInstance(); 
            esp_err_t err =  wf.Init();

            if (err!=ESP_FAIL)
            {
                std::cout << "core:: wifi driver started\n";
            }
            // if Init() {wf_flag = true;}
            
        }

#endif


// tasks
            // check sock flag
            // check wf again { if(wf_flag) }
            // check lora flag
            if ((f >> static_cast<uint8_t>(UFO_AppBiNum::sock_bit)) && 0b1 )
            {
                std::cout << "core:: flag: sock detected\n";
                // init trsm control block
                // create sock task
            }

            



    }

private:
    UFO_DataSystem _sys;
    /*error catch system*/
public: 
#ifdef UFO_USE_TASK_BARO
    UFO_DataBaro _baro;
#endif
#ifdef UFO_USE_TASK_IMU
    UFO_DataIMU _imu;
#endif
#ifdef UFO_USE_TASK_COMPASS
    // TaskHandle_t _compass;
#endif

#if defined(UFO_USE_TASK_SOCK) || defined(UFO_USE_TASK_LORA) || defined(UFO_USE_TASK_SOCK_LORA)
    UFO_TrsmControlBlock _trsmDCB;
#endif
    UFO_DataMotorThrot _motor;

public:
    UFO_Core(const UFO_Core&) = delete;
    const UFO_Core& operator =(const UFO_Core&) = delete;
    // UFO_Core& operator =(UFO_Core&&) = default;  

    static UFO_Core& GetInstance(uint8_t flagStart)
    {
        static UFO_Core core(flagStart);
        std::cout << "UFO Core created\n";
        return core;
    }


    UFO_DataSystem *__GetSys() { return &_sys; }
    
    ~UFO_Core()
    {
       
    }

    void InitializeSys();


    void GetPtrDataStorage()
    {
    }

private:
    int __CreateIdleTask(){
        xTaskCreatePinnedToCore(
            __IdleTask,
            "UIDL",
            1024,
            this,
            5,
            &_idle,
            0);
    }

    static void __IdleTask(void* arg){
        UFO_DataSystem* sys = reinterpret_cast<UFO_DataSystem*>(arg);

        while (true)
        {
            // if sys.error.critical || eif sys.error.IsCountingOut 
                //=>  sys.error.say(), __Terminate(), break;
            // if sys.error.warning => sys.error.say();
        }
        // say::program ended because of error: eTag, eNum, eText 
        // sat::restart chip by btn or reconnect power wire
        while (true)
        {
            
        }
        

    }

    static bool __Terminate() {

    }

};