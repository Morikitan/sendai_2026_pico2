#include "action.hpp"
#include "camera.hpp"
#include "display.hpp"
#include "gyro.hpp"
#include "line.hpp"
#include "main_to_line.hpp"
#include "main_to_sub.hpp"
#include "motor.hpp"
#include "other_sensor.hpp"
#include "others.hpp"
#include "servo.hpp"
#include "../config.hpp"
#include "pico/stdlib.h"
#include <math.h>
#include <stdio.h>

void CatchBall(){
    //
}

void CatchPetBottle(){
    //ペットボトルを探す挙動
    SetServoAngleFromMain(servo_arm_left_and_right_pin,90);
    SetServoAngleFromMain(servo_left_claw_pin,90);
    SetServoAngleFromMain(servo_right_claw_pin,90);
    SetServoAngleFromMain(servo_arm_up_and_down_pin,160);
    sleep_ms(2000);
    
    SetServoAngleFromMain(servo_left_claw_pin,40);
    SetServoAngleFromMain(servo_right_claw_pin,140);
    sleep_ms(500);
    SetServoAngleFromMain(servo_arm_left_and_right_pin,145);
    sleep_ms(500);
    SetServoAngleFromMain(servo_arm_up_and_down_pin,90);
    sleep_ms(2000);
    SetServoAngleFromMain(servo_arm_left_and_right_pin,155);
    sleep_ms(250);
    SetServoAngleFromMain(servo_arm_up_and_down_pin,90);
    sleep_ms(2000);
    SetServoAngleFromMain(servo_arm_left_and_right_pin,90);
    sleep_ms(1000);
    //指定の場所まで移動
    for(int i = 90; i <= 140;i += 10){
        sleep_ms(200);
        SetServoAngleFromMain(servo_arm_up_and_down_pin,i);
    }
    sleep_ms(2000);
    SetServoAngleFromMain(servo_left_claw_pin,90);
    SetServoAngleFromMain(servo_right_claw_pin,90);
    sleep_ms(2000);
}
