#pragma once

#include "UFO_Config.h"
#include <driver/ledc.h>


#define UFO_PWM_TIMER_MODE          LEDC_HIGH_SPEED_MODE
// #define UFO_PWM_TIMER_GROUP         ((uint8_t)1)
#define UFO_PWM_TIMER_PER_OUT       ((uint8_t)2)
#define UFO_PWM_FREQ                ((uint8_t)50)
#define UFO_CLK                     LEDC_AUTO_CLK
#define UFO_PWM_RESOLUTION          ((uint8_t)16)

class UFO_ESC_driver
{
private:
    ledc_timer_t _timer = LEDC_TIMER_MAX;
    ledc_channel_t _channel = LEDC_CHANNEL_MAX;
    uint8_t _pin = 255;
    uint16_t _duty = 0;

public:
    UFO_ESC_driver(/* args */) {}
    ~UFO_ESC_driver() {}

    void Setup(uint8_t escID, uint8_t pin)
    {
        _timer = static_cast<ledc_timer_t>(escID % UFO_PWM_TIMER_PER_OUT);
        _channel = static_cast<ledc_channel_t>(escID);
        _pin = pin;

        pinMode(_pin, OUTPUT);

        gpio_config_t conf = {
            .pin_bit_mask = (1ULL << pin),
            .mode = GPIO_MODE_DISABLE,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        if (gpio_config(&conf) != ESP_OK)
        {
            // set Critical
            Serial.print("error: GPIO config");
            return;
        }

        Serial.print("ESC_DRIVER_SETUP: ");
        Serial.println(_pin);

        ledc_timer_config_t timerCfg = {
            .speed_mode = UFO_PWM_TIMER_MODE,
            .duty_resolution = static_cast<ledc_timer_bit_t>(UFO_PWM_RESOLUTION),
            .timer_num = _timer,
            .freq_hz = UFO_PWM_FREQ,
            .clk_cfg = UFO_CLK
        };

        if (ledc_timer_config(&timerCfg) != ESP_OK)
        {
            // set Critical
            Serial.println("error: ledc_timer_config");
            return;
        }

        _duty = ledc_get_duty(UFO_PWM_TIMER_MODE, static_cast<ledc_channel_t>(_channel));
        ledc_channel_config_t chanCfg = {
            .gpio_num = _pin,
            .speed_mode = UFO_PWM_TIMER_MODE,
            .channel = _channel,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = _timer,
            .duty = _duty,
            .hpoint = 0
        };
        if (ledc_channel_config(&chanCfg) != ESP_OK)
        {
            // set Critical
            Serial.println("error: ledc_channel_config");
            return;
        }
    }

    void Write(int16_t val)
    {
        _duty = val;
        esp_err_t err = ledc_set_duty(LEDC_HIGH_SPEED_MODE, _channel, _duty);
        err = ledc_update_duty(LEDC_HIGH_SPEED_MODE, _channel);

        if (err != ESP_OK)
        {
            Serial.println("error: ledc_set_duty/ledc_update_duty");
        }
    }

    uint16_t GetDuty()
    {
        return _duty;
    }
};
