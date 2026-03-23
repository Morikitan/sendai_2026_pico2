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

int lineNumber;
int preLineAngle;
int lineAngle;
int lastLineTime; //ミリ秒
bool isLineBuzzerOn;

void LineTraceSetup(){
    lineNumber = 0;
    preLineAngle = 0;
    lineAngle = 0;
    lastLineTime;
}

//ブザーを鳴らす　count:回数　length:0で短1で長
void BuzzerRing(int times, int length){
    int sleeptimes;
    if(length == 1){
        sleeptimes = 200;
    }else if(length == 0){
        sleeptimes = 100;
    }
    for (int  i = 0; i < times; i++){
        gpio_put(buzzer_pin,1);
        sleep_ms(sleeptimes);
        gpio_put(buzzer_pin,0);
        sleep_ms(sleeptimes);
    }    
}

void MainMove(){
    if(lineNumber < 4){
        LineTrace();
    }else if(lineNumber < 5){
        while(preLineAngle != 90){
            LineTrace();
        }
        RotationToAngle(90);
        gpio_put(buzzer_pin,0);
        uint32_t now = time_us_32();
        while(!gpio_get(touch_sensor_front_left_pin)){
            if(angleX < 89.5){
                MainMotorState(400,395);
            }else if(angleX > 90.5){
                MainMotorState(395,400);
            }else{
                MainMotorState(400,400);
            }
            GetGyroAngleFromSub();
        }
        GetDataFromLineToMain();
        while(!circleLineSensor[15]){
            MainMotorState(-200,-200);
            GetDataFromLineToMain();
        }
        MainMotorState(200,200);
        sleep_ms(200);
        RotationToAngle(180);
        while(!gpio_get(touch_sensor_back_left_pin)){
            MainMotorState(-200,-200);
        }
        MainMotorState(0,0);
        SetStepperOff(1);
        SetStepperOff(2);
        sleep_ms(100000000);
    }
}

void LineTrace(){
    GetDataFromLineToMain();
    GetGyroAngleFromSub();
    if(frontLineSensor[0] == true && frontLineSensor[1]==true && frontLineSensor[2] == true){
        if(time_us_32() / 1000 > lastLineTime + 1000){
            gpio_put(buzzer_pin,1);
            lineNumber++;
            isLineBuzzerOn = true;
            lastLineTime = time_us_32() / 1000;
        }
    }
    if(isLineBuzzerOn){
        if(time_us_32() / 1000 > lastLineTime + 500){
            gpio_put(buzzer_pin,0);
        }
    }
    if(frontLineSensor[0] == true && frontLineSensor[2] == false){
        //左に曲がる
        if((lineAngle == 0 && preLineAngle == 270) || (lineAngle == preLineAngle + 90)){
            MainMotorState(125,125);
        }else{
            MainMotorState(-125,250);
        }
    }else if(frontLineSensor[0] == false && frontLineSensor[2] == true && !((lineAngle == 270 && preLineAngle == 0) || (lineAngle == preLineAngle - 90))){
        //右に曲がる
        MainMotorState(124,-62);
    }else{
        if((lineAngle == 270 && preLineAngle == 0) || (lineAngle == preLineAngle - 90)){
            //左に曲がる
            MainMotorState(50,250);
        }else if((lineAngle == 0 && preLineAngle == 270) || (lineAngle == preLineAngle + 90)){
            //右に曲がる
            MainMotorState(125,25);
        }else{
            MainMotorState(400,400);
        }
    }
    //lineAngleの設定
    if((lineAngle == 0 && (20 < angleX && angleX <= 180)) || (lineAngle != 0 && angleX > lineAngle + 20)){
        if(lineAngle == 270)lineAngle = 0;
        else lineAngle += 90;
    }else if((lineAngle == 0 && (180 <= angleX && angleX < 340)) || (lineAngle != 0 && angleX < lineAngle - 20)){
        if(lineAngle == 0)lineAngle = 270;
        else lineAngle -= 90;
    }
    //preLineAngleの設定
    if(lineAngle != preLineAngle){
        if(lineAngle == preLineAngle + 90 && lineAngle - 5 < angleX){
            preLineAngle = lineAngle;
        }else if(lineAngle == 0 && preLineAngle == 270 && (355 < angleX || angleX < 180)){
            preLineAngle = lineAngle;
        }else if(lineAngle == preLineAngle - 90 && (angleX < lineAngle + 5 || (lineAngle = 0 && angleX > 180))){
            preLineAngle = lineAngle;
        }else if(lineAngle == 270 && preLineAngle == 0 && (180 < angleX && angleX < 275)){
            preLineAngle = lineAngle;
        }
    }
    sleep_ms(1);
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
    RotationToAngle(270);
    sleep_ms(500);
    while(!gpio_get(touch_sensor_front_left_pin) && !gpio_get(touch_sensor_front_right_pin)){
            MainMotorState(200,200);
    }
    for(int i = 90; i <= 160;i += 10){
        MainMotorState(-10,-10);
        sleep_ms(200);
        SetServoAngleFromMain(servo_arm_up_and_down_pin,i);
    }
    MainMotorState(0,0);
    SetStepperSleep();
    sleep_ms(500);
    SetServoAngleFromMain(servo_left_claw_pin,90);
    SetServoAngleFromMain(servo_right_claw_pin,90);
    sleep_ms(1000);
    SetServoAngleFromMain(servo_arm_up_and_down_pin,60);
    sleep_ms(500);
    SetServoAngleFromMain(servo_arm_left_and_right_pin,90);
    sleep_ms(1000);
}

void UseAllSensor(){
    UseCamera();
    GetDataFromLineToMain();
    GetGyroAngleFromSub();
    GetDistanceFromSub();
    GetColorFromSub();
    GetCurrentFromSub();
}