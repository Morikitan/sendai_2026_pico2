#include "display.hpp"
#include "other_sensor.hpp"
#include "../config.hpp"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include <stdio.h>

uint8_t color;

void ColorLEDSetup(){
    gpio_init(color_sensor_LED_pin);
    gpio_init(color_sensor_SCL_pin);
    gpio_init(color_sensor_SDA_pin);
    gpio_set_dir(color_sensor_LED_pin,GPIO_OUT);
    gpio_set_function(color_sensor_SCL_pin,GPIO_FUNC_I2C);
    gpio_set_function(color_sensor_SDA_pin,GPIO_FUNC_I2C);
}

void UseColorSensor(){

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
void UseCurrentSensor(unsigned int input){
    adc_select_input(input);
    uint16_t result = adc_read();
}