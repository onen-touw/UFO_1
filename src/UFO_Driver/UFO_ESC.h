#pragma once

#include "UFO_Config.h"
#include <driver/ledc.h>



#define UFO_PWM_TIMER_MODE          LEDC_HIGH_SPEED_MODE
// #define UFO_PWM_TIMER_GROUP         ((uint8_t)1)
#define UFO_PWM_TIMER_PER_OUT       ((uint8_t)2)
#define UFO_PWM_FREQ                ((uint8_t)50)
#define UFO_CLK                     LEDC_AUTO_CLK
#define UFO_PWM_RESOLUTION          ((uint8_t)16)
#define UFO_ESC_MAX_OUTPUT_COUNT    ((uint8_t)4)

static uint8_t UFO_PWM_TIMER_USAGE = 0;

struct UFO_ESC_output
{
    ledc_timer_t timer = LEDC_TIMER_MAX;
    ledc_channel_t channel = LEDC_CHANNEL_MAX;
    uint8_t pin = 255;
    uint16_t duty = 0;
};


class UFO_ESC_driver
{
private:
    /* data */
    UFO_ESC_output* _output = nullptr;
    uint8_t _size = UFO_ESC_MAX_OUTPUT_COUNT;
    
public:
    UFO_ESC_driver(/* args */) {}
    ~UFO_ESC_driver() {
        if (_output)
        {
            delete[] _output;
        }
        
    }

    void Setup(uint8_t* pins, uint8_t size){
        _size = size;
        if (_size > UFO_ESC_MAX_OUTPUT_COUNT)
        {
            // set Critical 
            Serial.println("error: _size > UFO_ESC_MAX_OUTPUT_COUNT");
            return;
        }
        _output = new UFO_ESC_output[_size]();
        for (size_t i = 0; i < _size; ++i)
        {
            _output[i].timer = static_cast<ledc_timer_t>(i%UFO_PWM_TIMER_PER_OUT);
            _output[i].channel = static_cast<ledc_channel_t>(i);
            _output[i].pin = pins[i];

            pinMode(_output[i].pin, OUTPUT);

            Serial.print("ESC_DRIVER_SETUP: ");
            Serial.println(_output[i].pin);
            

            ledc_timer_config_t timerCfg = {
                .speed_mode = UFO_PWM_TIMER_MODE,
                .duty_resolution = static_cast<ledc_timer_bit_t>(UFO_PWM_RESOLUTION),
                .timer_num = _output[i].timer,
                .freq_hz = UFO_PWM_FREQ,
                .clk_cfg = UFO_CLK
                };

            if (ledc_timer_config(&timerCfg) != ESP_OK)
            {
                // set Critical 
                Serial.println("error: ledc_timer_config");
                return;
            }

            _output[i].duty = ledc_get_duty(UFO_PWM_TIMER_MODE, static_cast<ledc_channel_t>(_output[i].channel));
            ledc_channel_config_t chanCfg = {
                .gpio_num = _output[i].pin,
                .speed_mode = UFO_PWM_TIMER_MODE,
                .channel = _output[i].channel,
                .intr_type = LEDC_INTR_DISABLE,
                .timer_sel = _output[i].timer,
                .duty = _output[i].duty,
                .hpoint = 0
                };
            if (ledc_channel_config(&chanCfg) != ESP_OK)
            {
                // set Critical
                Serial.println("error: ledc_channel_config");
                return;
            }
        }
    }

    void Write(uint8_t esc, int16_t val){
        if (esc >= _size || esc < 0)
        {
            // set Critical
            Serial.println("error: esc _size conflict");
            return;
        }
        _output[esc].duty = val;
        esp_err_t err =  ledc_set_duty(LEDC_HIGH_SPEED_MODE, _output[esc].channel, _output[esc].duty);
        err = ledc_update_duty(LEDC_HIGH_SPEED_MODE, _output[esc].channel);

        if (err != ESP_OK)
        {
            Serial.println("error: ledc_set_duty/ledc_update_duty");
        }
        
        
    }
};


// -- setup




// ledc_timer_config_t timerCfg = {
//     .speed_mode = UFO_PWM_TIMER_GROUP,
//     .duty_resolution = UFO_PWM_RESOLUTION,
//     .timer_num = ,
//     .freq_hz = UFO_PWM_FREQ,
//     .clk_cfg = UFO_CLK
// };

// if(ledc_timer_config(&ledc_timer) != ESP_OK)
//     {
//         log_e("ledc setup failed!");
//         return 0;
//     }


// uint32_t duty = ledc_get_duty(group,channel);
// ledc_channel_config_t chanCfg = {
//         .speed_mode     = group,
//         .channel        = channel,
//         .timer_sel      = timer,
//         .intr_type      = LEDC_INTR_DISABLE,
//         .gpio_num       = pin,
//         .duty           = duty,
//         .hpoint         = 0
// };







// -- setup


// --write

    // ledc_set_duty(group, channel, duty);
    // ledc_update_duty(group, channel);



// --write




