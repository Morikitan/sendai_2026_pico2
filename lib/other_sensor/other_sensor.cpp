#include "display.hpp"
#include "other_sensor.hpp"
#include "../config.hpp"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include <stdio.h>

void ColorLEDSetup(){

}

void CurrentSensorSetup(){
    adc_init();
    adc_gpio_init(current_sensor_left_pin);
    adc_gpio_init(current_sensor_right_pin);
    adc_gpio_init(current_sensor_DC_pin);
}

//input : 使うピンの設定
//        left  : 0
//        right : 1
//        DC    : 2 
void UseCurrentSensor(uint input){
    adc_select_input(input);
    uint16_t result = adc_read();
}