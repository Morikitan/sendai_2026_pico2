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
    //sliceの設定
    stepper_left_slice_num = pwm_gpio_to_slice_num(stepper_left_clock_pin);
    stepper_right_slice_num = pwm_gpio_to_slice_num(stepper_right_clock_pin);
    //ステッパーのモード設定
    gpio_init(10);
    gpio_init(11);
    gpio_init(21);
    gpio_set_dir(10,GPIO_OUT);
    gpio_set_dir(11,GPIO_OUT);
    gpio_set_dir(21,GPIO_OUT);
    gpio_put(10,1);
    gpio_put(11,0);
    gpio_put(21,0);
    //ステッパーのリセット
    gpio_put(stepper_reset_pin, 1);
    sleep_ms(10);
    gpio_put(stepper_reset_pin, 0);
    gpio_put(stepper_sleep_pin,0);
    //ステッパーの向きの設定
    gpio_put(stepper_left_direction_pin, 1);
    gpio_put(stepper_right_direction_pin, 1);
    SetStepperOff(1);
    SetStepperOff(2);
}

// slice_num : stepper_left_slice_numかstepper_right_slice_num
// gpio : stepper_left_clock_pinかstepper_right_clock_pin
// freq_hz : 速さの変数。500までは回る
void SetStepperSpeed(unsigned int slice_num, unsigned int gpio, float freq_hz){
    if(freq_hz == 0){
        pwm_set_enabled(slice_num,false);
        return;
    }
    const uint wrap = 10000;                // 解像度
    float clkdiv = 150000000 / (freq_hz * wrap);
    if (clkdiv > 255.0f) clkdiv = 255.0f;

    pwm_set_clkdiv(slice_num, clkdiv);
    pwm_set_wrap(slice_num, wrap);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(gpio), wrap / 2);
    pwm_set_enabled(slice_num, true);
}

//motor 左なら1 右なら2
void SetStepperOff(int motor){
    if(motor == 1){
        pwm_set_enabled(stepper_left_slice_num, false);
    }else if(motor == 2){
        pwm_set_enabled(stepper_right_slice_num, false);
    }
}

void SetStepperON(){
    gpio_put(stepper_sleep_pin,0);
}

//不可逆
void SetStepperSleep(){
    gpio_put(stepper_sleep_pin,1);
}

//speed1 : 左ステッパーのスピード
//speed2 : 右ステッパーのスピード
void MainMotorState(int speed1,int speed2){
    if(speed1 > 0){
        SetStepperSpeed(stepper_left_slice_num,stepper_left_clock_pin,speed1);
        gpio_put(stepper_left_direction_pin, 0);
    }else if(speed1 == 0){
        SetStepperSpeed(stepper_left_slice_num,stepper_left_clock_pin,0);
        gpio_put(stepper_left_direction_pin, 0);
    }else{
        SetStepperSpeed(stepper_left_slice_num,stepper_left_clock_pin,abs(speed1));
        gpio_put(stepper_left_direction_pin, 1);
    }

    if(speed2 > 0){
        SetStepperSpeed(stepper_right_slice_num,stepper_right_clock_pin,speed2);
        gpio_put(stepper_right_direction_pin, 1);
    }else if(speed2 == 0){
        SetStepperSpeed(stepper_right_slice_num,stepper_right_clock_pin,0);
        gpio_put(stepper_right_direction_pin, 1);
    }else{
        SetStepperSpeed(stepper_right_slice_num,stepper_right_clock_pin,abs(speed2));
        gpio_put(stepper_right_direction_pin, 0);
    }
}

void SuctionSetup(){
    gpio_init(motor_suction_pin);
    gpio_set_function(motor_suction_pin,GPIO_FUNC_PWM);
}

//吸引用モーター
void SetSuctionMotorSpeed(uint duty){
    //周波数をf[Hz]とすると
    //(pico2)150×1000×1000 = f × clkdiv × (warp + 1) clkdiv = 588.235
    //(pico) 125×1000×1000 = f × clkdiv × (warp + 1) clkdiv = 488.281
    //よって今は f = 1.0[kHz]
    uint slice_num = pwm_gpio_to_slice_num(motor_suction_pin);
    uint channel = pwm_gpio_to_channel(motor_suction_pin);
    pwm_set_clkdiv(slice_num, 150.0);
    pwm_set_wrap(slice_num, 999);
    pwm_set_chan_level(slice_num, channel, duty);
    pwm_set_enabled(slice_num, true);
}