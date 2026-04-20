#include "display.hpp"
#include "others.hpp"
#include "ws2812.pio.h"
#include "../config.hpp"
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include <stdio.h>

#define IS_RGBW false
#define NUM_PIXELS 1

std::string SerialWatch;

PIO pio_colorLED = pio1;
int sm_colorLED = 0;
uint offset_colorLED;

//メインマイコンのピン設定
void PinSetup(){
    gpio_init(tactile_switch_pin1);
    gpio_init(tactile_switch_pin2);
    gpio_init(tactile_switch_pin3);
    gpio_init(buzzer_pin);
    gpio_init(touch_sensor_back_left_pin);
    gpio_init(touch_sensor_back_right_pin);
    gpio_init(touch_sensor_front_left_pin);
    gpio_init(touch_sensor_front_right_pin);
    gpio_set_dir(tactile_switch_pin1,GPIO_OUT);
    gpio_set_dir(tactile_switch_pin2,GPIO_OUT);
    gpio_set_dir(tactile_switch_pin3,GPIO_OUT);
    gpio_set_dir(buzzer_pin,GPIO_OUT);
    gpio_set_dir(touch_sensor_back_left_pin,GPIO_IN);
    gpio_set_dir(touch_sensor_back_right_pin,GPIO_IN);
    gpio_set_dir(touch_sensor_front_left_pin,GPIO_IN);
    gpio_set_dir(touch_sensor_front_right_pin,GPIO_IN);
}

void ColorLEDSetup(){
    offset_colorLED = pio_add_program(pio_colorLED, &ws2812_program);
    ws2812_program_init(pio_colorLED, sm_colorLED, offset_colorLED, color_LED_pin, 800000, IS_RGBW);
}

void UseColorLED(uint8_t red,uint8_t green,uint8_t blue){
    pio_sm_put_blocking(pio_colorLED, 0, ((uint32_t)(red) << 16) | ((uint32_t)(green) << 24) | (uint32_t)(blue) << 8);
}

//正面0度時計回りの度数法の角度を座標平面の弧度法(正面π/2反時計回り)に変換する。
//定義域は 0 <= θ < 2π
float radian(float angle){
    if(angle <= 90){
        return (angle * -1.0 + 90) * 3.1415 / 180;
    }else{
        return (angle * -1.0 + 450) * 3.1415 / 180;
    }
}