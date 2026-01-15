#include "display.hpp"
#include "servo.hpp"
#include "../config.hpp"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include <stdio.h>

void ServoSetup(){
    //ピンの初期化
    gpio_init(servo_left_basket_pin);
    gpio_init(servo_sentor_basket_pin);
    gpio_init(servo_right_basket_pin);
    gpio_init(servo_left_claw_pin);
    gpio_init(servo_right_claw_pin);
    gpio_init(servo_arm_left_and_right_pin);
    gpio_init(servo_arm_up_and_down_pin);
    gpio_set_function(servo_left_basket_pin,GPIO_FUNC_PWM);
    gpio_set_function(servo_sentor_basket_pin,GPIO_FUNC_PWM);
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
    if(0 <= angle && angle <= 180){
        uint slice_num = pwm_gpio_to_slice_num(gpio);
        pwm_set_enabled(slice_num,false);
        if(gpio == 8 || gpio == 10){
            //MG90S
            // クロック分周器を100.0に設定 (150MHz / 100 = 1.5MHz)
            pwm_set_clkdiv(slice_num, 100.0f);
            // ラップ値を29,999に設定 (周期を50Hz = 20msにする) (1.5MHz / 30,000 = 50Hz)
            pwm_set_wrap(slice_num, 29999);
            //600 → 0° 1200 → 180°
            pwm_set_gpio_level(gpio,(uint16_t)(600 + 600 * angle / 180));
        }
    }
}