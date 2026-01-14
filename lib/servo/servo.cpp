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

void SetArmAngle(int angle){

}