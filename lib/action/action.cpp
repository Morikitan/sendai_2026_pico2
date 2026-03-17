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

//ブザーを鳴らす　count:回数　length:0で短1で長
void BuzzerRing(int times, int length){
    int sleeptimes;
    if(length == 1){
        sleeptimes = 100;
    }else if(length == 0){
        sleeptimes = 50;
    }
    for (int  i = 0; i < times; i++){
        gpio_put(buzzer_pin,1);
        sleep_ms(sleeptimes);
        gpio_put(buzzer_pin,0);
        sleep_ms(sleeptimes);
    }    
}

void LineTrace(){
    GetDataFromLineToMain();
    if(frontLineSensor[0] == true && frontLineSensor[2] == true){
        MainMotorState(250,250);
        BuzzerRing(2,0);
        sleep_ms(100);
    }
    if(frontLineSensor[0] == true && frontLineSensor[2] == false){
        //左に曲がる
        MainMotorState(-125,250);
    }else if(frontLineSensor[0] == false && frontLineSensor[2] == true){
        //右に曲がる
        MainMotorState(250,-125);
    }else{
        MainMotorState(250,250);
    }
    sleep_ms(5);
}

void RotationToAngle(int target_angle){
    SetStepperON();
    //目標角度を0度から360度の間に補正
    target_angle = target_angle % 360;
    if (target_angle < 0) {
        target_angle += 360;
    }
    while(true){
        GetGyroAngleFromSub();
        //rotation_angleは回転角
        float rotation_angle = target_angle - angleX;
        if(rotation_angle < 0){
            rotation_angle += 360; 
        }
        if(rotation_angle < 2 || rotation_angle > 358){
            break;
        }
        //回転角が180°未満なら時計回り,180度以上なら反時計回り
        if(rotation_angle < 180){
            MainMotorState(150,-150);
        }else if(rotation_angle >= 180){
            MainMotorState(-150,150);
        }
        sleep_ms(5);
    }
    MainMotorState(0,0);
    SetStepperSleep();
}

void CatchBall(){
    //ボールを探す挙動

    //tofで探す
    uint32_t tofTime = time_us_32();
    do{
        SetStepperON();
        MainMotorState(100,100);
        GetDistanceFromSub();
        if(distance > 235){
            tofTime = time_us_32();
        }
    }while(tofTime + 100000 < time_us_32());
    SetStepperSleep();
    MainMotorState(0,0);
    SetServoAngleFromMain(servo_arm_up_and_down_pin,155);
    SetServoAngleFromMain(servo_arm_left_and_right_pin,90);
    SetServoAngleFromMain(servo_left_claw_pin,90);
    SetServoAngleFromMain(servo_right_claw_pin,90);
    sleep_ms(1000);
    SetServoAngleFromMain(servo_arm_up_and_down_pin,163);
    sleep_ms(1000);
    SetSuctionMotorSpeedFromMain(90);
    sleep_ms(2000);
    SetServoAngleFromMain(servo_arm_up_and_down_pin,90);
    sleep_ms(1000);
    SetServoAngleFromMain(servo_arm_up_and_down_pin,60);
    uint32_t colorTime = time_us_32();
    color = 0;
    while(true){
        GetColorFromSub();
        if(color == 1 || color == 3){
            SetServoAngleFromMain(servo_arm_left_and_right_pin,120);
            break;
        }else if(color == 2){
            SetServoAngleFromMain(servo_arm_left_and_right_pin,60);
            break;
        }
        if(colorTime + 1000000 < time_us_32()){
            //タイムアウト
            SetServoAngleFromMain(servo_arm_left_and_right_pin,90);
            SetServoAngleFromMain(servo_arm_up_and_down_pin,150);
            sleep_ms(1000);
            SetSuctionMotorSpeedFromMain(0);
            sleep_ms(1000);
            return;
        }
        sleep_ms(10);
    }
    sleep_ms(1000);
    SetSuctionMotorSpeedFromMain(0);
    sleep_ms(2000);
    SetServoAngleFromMain(servo_arm_up_and_down_pin,40);
    sleep_ms(1000);
    SetServoAngleFromMain(servo_arm_up_and_down_pin,90);
}

void CatchCan(){
    SetServoAngleFromMain(servo_arm_left_and_right_pin,90);
    SetServoAngleFromMain(servo_left_claw_pin,160);
    SetServoAngleFromMain(servo_right_claw_pin,20);
    SetServoAngleFromMain(servo_arm_up_and_down_pin,155);
    sleep_ms(1000);
    SetServoAngleFromMain(servo_arm_up_and_down_pin,163);
    sleep_ms(3000);
    //少し前進する？
    SetServoAngleFromMain(servo_right_claw_pin,140);
    SetServoAngleFromMain(servo_left_claw_pin,41);
    sleep_ms(1000);
    SetServoAngleFromMain(servo_arm_up_and_down_pin,50);
    GetColorFromSub();
    if(color == 1 || color == 3)canNumber += 1;
    sleep_ms(1000);
    SetServoAngleFromMain(servo_left_claw_pin,160);
    SetServoAngleFromMain(servo_right_claw_pin,20);
    sleep_ms(1000);
    SetServoAngleFromMain(servo_arm_up_and_down_pin,90);
    if(canNumber == 1 && (color == 1 || color == 3)){
        sleep_ms(250);
        SetServoAngleFromMain(servo_sentor_basket_pin,120);
        sleep_ms(750);
        SetServoAngleFromMain(servo_sentor_basket_pin,160);
        sleep_ms(500);
        SetServoOffFromMain(servo_sentor_basket_pin);
    }
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

void UseAllSensor(){
    UseCamera();
    GetDataFromLineToMain();
    GetGyroAngleFromSub();
    GetDistanceFromSub();
    GetColorFromSub();
    GetCurrentFromSub();
}