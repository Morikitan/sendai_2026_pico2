#include "display.hpp"
#include "main_to_sub.hpp"
#include "servo.hpp"
#include "../config.hpp"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include <stdio.h>

void ServoSetup(){
    //ピンの初期化
    gpio_init(servo_left_basket_pin);
    gpio_init(servo_center_basket_pin);
    gpio_init(servo_right_basket_pin);
    gpio_init(servo_left_claw_pin);
    gpio_init(servo_right_claw_pin);
    gpio_init(servo_arm_left_and_right_pin);
    gpio_init(servo_arm_up_and_down_pin);
    gpio_set_function(servo_left_basket_pin,GPIO_FUNC_PWM);
    gpio_set_function(servo_center_basket_pin,GPIO_FUNC_PWM);
    gpio_set_function(servo_right_basket_pin,GPIO_FUNC_PWM);
    gpio_set_function(servo_left_claw_pin,GPIO_FUNC_PWM);
    gpio_set_function(servo_right_claw_pin,GPIO_FUNC_PWM);
    gpio_set_function(servo_arm_left_and_right_pin,GPIO_FUNC_PWM);
    gpio_set_function(servo_arm_up_and_down_pin,GPIO_FUNC_PWM);
}

//サーボの角度を設定するプログラム
//pin : ピン番号
//angle : 整数の角度(0°～180°(GPIO8,10は160°まで?))
void SetServoAngle(unsigned int gpio,int angle){
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_set_enabled(slice_num,false);
    if(gpio == servo_left_claw_pin || gpio == servo_right_claw_pin || gpio == servo_left_basket_pin || gpio == servo_right_basket_pin || gpio == servo_center_basket_pin){
        if(10 <= angle && angle <= 160){
            //MG90S
            // クロック分周器を100.0に設定 (150MHz / 100 = 1.5MHz)
            pwm_set_clkdiv(slice_num, 100.0f);
            // ラップ値を29,999に設定 (周期を50Hz = 20msにする) (1.5MHz / 30,000 = 50Hz)
            pwm_set_wrap(slice_num, 29999);
            //750 → 0° 3750 → 180°
            pwm_set_gpio_level(gpio,750 + (uint16_t)((3750 - 750) * angle / 180.0f));
            //pwmをオンにする
            pwm_set_enabled(slice_num,true);
        }
    }else if(gpio == servo_arm_left_and_right_pin || gpio == servo_arm_up_and_down_pin){
        if(10 <= angle && angle <= 170){
            //MG996R
            // クロック分周器を100.0に設定 (150MHz / 100 = 1.5MHz)
            pwm_set_clkdiv(slice_num, 100.0f);
            // ラップ値を29,999に設定 (周期を50Hz = 20msにする) (1.5MHz / 30,000 = 50Hz)
            pwm_set_wrap(slice_num, 29999);
            //750 → 0° 3750 → 180°
            pwm_set_gpio_level(gpio,750 + (uint16_t)((3750 - 750) * angle / 180.0f));
            //pwmをオンにする
            pwm_set_enabled(slice_num,true);
        }
    }
}

void SetServoOff(unsigned int gpio){
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_set_enabled(slice_num,false);
}

void ThrowCan(){
    SetServoAngle(servo_center_basket_pin,60);
    sleep_ms(2000);
    SetServoAngle(servo_center_basket_pin,160);
    sleep_ms(500);
    SetServoOff(servo_center_basket_pin);
}

void CatchCanFromSub(){
    SetServoAngle(servo_arm_left_and_right_pin,90);
    SetServoAngle(servo_left_claw_pin,160);
    SetServoAngle(servo_right_claw_pin,20);
    SetServoAngle(servo_arm_up_and_down_pin,160);
    sleep_ms(750);
    SetServoAngle(servo_arm_up_and_down_pin,163);
    sleep_ms(3000);
    //少し前進する？
    SetServoAngle(servo_right_claw_pin,140);
    SetServoAngle(servo_left_claw_pin,40);
    sleep_ms(1000);
    SetServoAngle(servo_arm_up_and_down_pin,45);
    // GetColorFromSub();
    // if(color == 2 || color == 3)canNumber += 1;
    sleep_ms(1000);
    SetServoAngle(servo_left_claw_pin,160);
    SetServoAngle(servo_right_claw_pin,20);
    sleep_ms(1000);
    SetServoAngle(servo_arm_up_and_down_pin,90);
    // if(canNumber == 1 && (color == 2 || color == 3)){
        // sleep_ms(250);
        // SetServoAngle(servo_center_basket_pin,120);
        // sleep_ms(500);
        // SetServoAngle(servo_center_basket_pin,160);
        // sleep_ms(500);
        // SetServoOff(servo_center_basket_pin);
    // }
}

