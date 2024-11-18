#pragma once

#include "UFO_Config.h"



esp_err_t UFO_gpioConfig(gpio_num_t pin, gpio_mode_t mode){

    gpio_config_t conf = {
        .pin_bit_mask = 1ULL << pin,
        .mode = mode,           //GPIO_MODE_INPUT | GPIO_MODE_OUTPUT
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    return gpio_config(&conf);
}

int64_t UFO_CoreTimeMicro(){
    return esp_timer_get_time();
}

int64_t UFO_CoreTimeMilli(){
    return static_cast<int64_t> (esp_timer_get_time()/1000ULL);
}