#pragma once
#include "UFO_Config.h"

#include "driver/i2c.h"

// I2C matrix
// #define UFO_I2C_WARE_TYPES 2
// static UFO_I2C_PIN_MATRIX[scl_hardware] = [ [sda_hardware, scl_hardware]  [sda_software scl_software]  ]

#define UFO_I2C_HARDWARE_SDA ((uint8_t)21)
#define UFO_I2C_HARDWARE_SCL ((uint8_t)22)
#define UFO_I2C_MAX_FREQ 1000000UL
#define UFO_I2C_TIMEOUT 0xFFFFF

enum UFO_I2C_port : uint8_t
{
    UFO_I2C_HARDWARE,
    UFO_I2C_SOFTWARE,
};

// todo : create driver basic_class
//  init(begin/setup)
// todo : set address
class UFO_I2C_Driver /* public : UFO_Exceptions */
{
protected:
    SemaphoreHandle_t _lock;
    uint32_t _freq = 0;
    UFO_I2C_port _port;
    uint32_t _timeOutMillis = 50;
    bool _inited = false;

public:
    // protected:
    UFO_I2C_Driver(/* args */) {}

    ~UFO_I2C_Driver()
    {
        if (_inited)
        {
            __Deinit();
        }
        if (_lock)
        {
            vSemaphoreDelete(_lock);
        }
    }
    bool Initialized()
    {
        return _inited;
    }
    esp_err_t Init(UFO_I2C_port port, uint8_t pinSDA = UFO_I2C_HARDWARE_SDA, uint8_t pinSCL = UFO_I2C_HARDWARE_SCL)
    {
        esp_err_t ret = ESP_OK;
        if (_inited)
        {
            return ret;
        }

        if (port == UFO_I2C_HARDWARE)
        {
            pinSDA = UFO_I2C_HARDWARE_SDA;
            pinSCL = UFO_I2C_HARDWARE_SCL;
        }
        else
        {
            if (pinSDA == UFO_I2C_HARDWARE_SDA || pinSDA == UFO_I2C_HARDWARE_SCL)
            {
                ret = ESP_FAIL;
                // set critical [incorrect pins]
                return ret;
            }
            if (pinSCL == UFO_I2C_HARDWARE_SDA || pinSCL == UFO_I2C_HARDWARE_SCL)
            {
                ret = ESP_FAIL;
                // set critical [incorrect pins]
                return ret;
            }
        }

        _lock = xSemaphoreCreateMutex();
        if (!_lock)
        {
            // set critical  [no mem]
            Serial.print("critical error:: [xSemaphoreCreateMutex] no mem");
            Serial.print(" ");
            Serial.print(__FILE__);
            Serial.print("\t");
            Serial.println(__LINE__);
            return ret;
        }

        if (xSemaphoreTake(_lock, portMAX_DELAY) != pdTRUE)
        {
            Serial.print("critical error:: could not acquire lock");
            Serial.print(__FILE__);
            Serial.print("\t");
            Serial.println(__LINE__);
            // set critical  [could not acquire lock ]
            //  ret = ;
        }
        ret = __Init(port, pinSDA, pinSCL);
        if (ret == ESP_OK)
        {
            _inited = true;
        }

        xSemaphoreGive(_lock);
        return ret;
    }
    esp_err_t Write(uint8_t addr, uint8_t *buf, uint32_t size)
    {
        esp_err_t ret = ESP_FAIL;
        ret = ESP_FAIL;
        if (!_inited || _lock == NULL || xSemaphoreTake(_lock, portMAX_DELAY) != pdTRUE)
        {
            // set critical ["could not acquire lock"]
            Serial.print("critical error:: could not acquire lock");
            Serial.print(__FILE__);
            Serial.print("\t");
            Serial.println(__LINE__);
            return ret;
        }
        ret = __Write(addr, buf, size);
        if (ret != ESP_OK)
        {
            // set warning []
            Serial.println("Driver:: __Write error");
        }
        xSemaphoreGive(_lock);
        return ret;
    }

    // 0 = good; 1 = bad
    uint8_t ZeroWrite(uint8_t addr)
    {
        esp_err_t ret = ESP_FAIL;
        ret = ESP_FAIL;
        if (!_inited || _lock == NULL || xSemaphoreTake(_lock, portMAX_DELAY) != pdTRUE)
        {
            // set critical ["could not acquire lock"]
            Serial.print("critical error:: could not acquire lock");
            Serial.print(__FILE__);
            Serial.print("\t");
            Serial.println(__LINE__);
            return ret;
        }
        uint32_t sz = 0;
        uint8_t buf[] = {0};
        ret = __Write(addr, buf, sz);
        xSemaphoreGive(_lock);
        return ret;
    }

    esp_err_t Read(uint16_t address, uint8_t *buff, size_t size)
    {
        esp_err_t ret = ESP_FAIL;
        if (!_inited || _lock == NULL || xSemaphoreTake(_lock, portMAX_DELAY) != pdTRUE)
        {
            // set critical ["could not acquire lock"]
            Serial.print("critical error:: could not acquire lock");
            Serial.print(__FILE__);
            Serial.print("\t");
            Serial.println(__LINE__);
            return ret;
        }
        // todo::
        Serial.println("i2c_master_read_from_device");
        ret = i2c_master_read_from_device(static_cast<i2c_port_t>(_port), address, buff, size, _timeOutMillis / portTICK_RATE_MS);
        if (ret != ESP_OK)
        {
            // set warning []
            Serial.println("Driver:: Read error");
        }
        xSemaphoreGive(_lock);
        return ret;
    }


    esp_err_t WriteRead(uint8_t address, uint8_t *wbuff, size_t wsize, uint8_t *rbuff, size_t rsize)
    {
        esp_err_t ret = ESP_FAIL;
        if (!_inited || _lock == NULL || xSemaphoreTake(_lock, portMAX_DELAY) != pdTRUE)
        {
            Serial.print("critical error:: could not acquire lock");
            Serial.print(__FILE__);
            Serial.print("\t");
            Serial.println(__LINE__);
            // set critical []
            return ret;
        }
        // todo::
        ret = i2c_master_write_read_device(static_cast<i2c_port_t>(_port), address, wbuff, wsize, rbuff, rsize, _timeOutMillis / portTICK_RATE_MS);

        // ret = __Write(address, wbuff, wsize);
        // if (ret != ESP_OK)
        // {
        //     // set warning []
        // }
        // ret = __Read(address, rbuff, rsize);

        if (ret != ESP_OK)
        {
            Serial.print("Warning:: i2c_master_write_read_device error");
            Serial.print(__FILE__);
            Serial.print("\t");
            Serial.println(__LINE__);
            // set warning
        }
        xSemaphoreGive(_lock);
        return ret;
    }

private:
    // protected:
    esp_err_t __Deinit()
    {
        esp_err_t ret = ESP_OK;
        if (!_inited)
        {
            return ret;
        }

        ret = ESP_FAIL;
        if (_lock == NULL || xSemaphoreTake(_lock, portMAX_DELAY) != pdTRUE)
        {
            Serial.print("critical error:: could not acquire lock");
            Serial.print(__FILE__);
            Serial.print("\t");
            Serial.println(__LINE__);
            // set critical ["could not acquire lock"]
            return ret;
        }
        ret = i2c_driver_delete(static_cast<i2c_port_t>(_port));
        if (ret != ESP_OK)
        {
            // set critical  [i2c driver deleting failed]
        }
        xSemaphoreGive(_lock);
        return ret;
    }

    

    esp_err_t __Write(uint8_t &addr, uint8_t *buff, uint32_t &size)
    {
        esp_err_t ret = ESP_FAIL;
        i2c_cmd_handle_t cmd = NULL;
        uint8_t cmd_buff[I2C_LINK_RECOMMENDED_SIZE(1)] = {0};
        cmd = i2c_cmd_link_create_static(cmd_buff, I2C_LINK_RECOMMENDED_SIZE(1));

        ret = i2c_master_start(cmd);
        if (ret != ESP_OK)
        {
            return ret;
        }
        ret = i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        if (ret != ESP_OK)
        {
            return ret;
        }
        if (size)
        {
            ret = i2c_master_write(cmd, buff, size, true);
            if (ret != ESP_OK)
            {
                return ret;
            }
        }
        ret = i2c_master_stop(cmd);
        if (ret != ESP_OK)
        {
            return ret;
        }
        ret = i2c_master_cmd_begin(static_cast<i2c_port_t>(_port), cmd, _timeOutMillis / portTICK_RATE_MS);
        return ret;
    }

    esp_err_t __Init(UFO_I2C_port &port, uint8_t &pinSDA, uint8_t &pinSCL)
    {
        esp_err_t ret = ESP_OK;
        if (_inited)
        {
            return ret;
        }

        if (!_freq)
        {
            _freq = 400000UL;
        }

        _port = port;

        i2c_config_t conf = {};
        conf.mode = I2C_MODE_MASTER;
        conf.scl_io_num = static_cast<gpio_num_t>(pinSCL);
        conf.sda_io_num = static_cast<gpio_num_t>(pinSDA);
        conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
        conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
        conf.master.clk_speed = _freq;
        conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

        ret = i2c_param_config(static_cast<i2c_port_t>(_port), &conf);
        if (ret != ESP_OK)
        {
            // set critical  [i2c configurating failed]
            return ret;
        }

        ///
        // @param slv_rx_buf_len Receiving buffer size. Only slave mode will use this value, it is ignored in master mode.
        // @param slv_tx_buf_len Sending buffer size. Only slave mode will use this value, it is ignored in master mode.
        ret = i2c_driver_install(static_cast<i2c_port_t>(_port), conf.mode, 0, 0, 0);
        if (ret != ESP_OK)
        {
            // set critical  [i2c driver installing failed]
            return ret;
        }
        ret = i2c_set_timeout(static_cast<i2c_port_t>(_port), UFO_I2C_TIMEOUT);
        if (ret != ESP_OK)
        {
            // set critical  [i2c setting timeout failed]
            return ret;
        }
        return ret;
    }
};
