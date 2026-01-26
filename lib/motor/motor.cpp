#include "display.hpp"
#include "motor.hpp"
#include "../config.hpp"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include <stdio.h>

uint stepper_right_slice_num;
uint stepper_left_slice_num;

//速度のパラメタ
float freq = 200.0f;
float max_freq = 500.0f;
float step = 10.0f;

void StepperSetup(){
    //a
    gpio_init(stepper_left_clock_pin);
    gpio_init(stepper_left_direction_pin);
    gpio_init(stepper_right_clock_pin);
    gpio_init(stepper_right_direction_pin);
    gpio_init(stepper_reset_pin);
    gpio_init(stepper_sleep_pin);
    gpio_set_function(stepper_left_clock_pin,GPIO_FUNC_PWM);
    gpio_set_function(stepper_right_clock_pin,GPIO_FUNC_PWM);
    gpio_set_dir(stepper_left_direction_pin,GPIO_OUT);
    gpio_set_dir(stepper_right_direction_pin,GPIO_OUT);
    gpio_set_dir(stepper_reset_pin,GPIO_OUT);
    gpio_set_dir(stepper_sleep_pin,GPIO_OUT);

    //a
    stepper_left_slice_num = pwm_gpio_to_slice_num(stepper_left_clock_pin);
    stepper_right_slice_num = pwm_gpio_to_slice_num(stepper_right_clock_pin);

    //ステッパーのリセット
    gpio_put(stepper_reset_pin, 1);
    sleep_ms(10);
    gpio_put(stepper_reset_pin, 0);
    gpio_put(stepper_sleep_pin,0);
    //ステッパーのモードと向きの設定
    gpio_put(stepper_left_direction_pin, 1);
    gpio_put(stepper_right_direction_pin, 0);
}

void unnamed(unsigned int slice_num,unsigned int gpio, float freq_hz){
    const uint wrap = 10000;                // 解像度
    float clkdiv = 150000000 / (freq_hz * wrap);
    if (clkdiv < 1.0f) clkdiv = 1.0f;
    if (clkdiv > 255.0f) clkdiv = 255.0f;

    pwm_set_clkdiv(slice_num, clkdiv);
    pwm_set_wrap(slice_num, wrap);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(gpio), wrap / 2);
    pwm_set_enabled(gpio, true);
}