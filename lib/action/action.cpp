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
float lineVector;

#define RightCircleLine circleLineSensor[11] + circleLineSensor[12] + circleLineSensor[13] + circleLineSensor[14] + circleLineSensor[15] + circleLineSensor[16] + circleLineSensor[17] + circleLineSensor[18] + circleLineSensor[19]
#define LeftCircleLine circleLineSensor[1] + circleLineSensor[2] + circleLineSensor[3] + circleLineSensor[4] + circleLineSensor[5] + circleLineSensor[6] + circleLineSensor[7] + circleLineSensor[8] + circleLineSensor[9]
#define RightFrontCircleLine circleLineSensor[16] + circleLineSensor[17] + circleLineSensor[18]
#define LeftFrontCircleLine circleLineSensor[2] + circleLineSensor[3] + circleLineSensor[4]

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
        NewLineTrace();
    }else if(lineNumber < 5){
        while(preLineAngle != 90){
            NewLineTrace();
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

//円形ラインセンサを使ったライントレース
void NewLineTrace(){
    GetDataFromLineToMain();
    GetGyroAngleFromSub();
    GetCircleLineVector(20,true,true);
    if(RightFrontCircleLine > 0 && LeftFrontCircleLine > 0){
        //十字の感知
        if(time_us_32() / 1000 > lastLineTime + 1000){
            gpio_put(buzzer_pin,1);
            lineNumber++;
            isLineBuzzerOn = true;
            lastLineTime = time_us_32() / 1000;
        }
    }else if(RightCircleLine > 0 && LeftCircleLine == 0){
        //右に曲がる
        MainMotorState(250,-125);
    }else if(RightCircleLine == 0 && LeftCircleLine > 0){
        //左に曲がる
        MainMotorState(-125,250);
    }else{
        MainMotorState(400,400);
    }
    if(isLineBuzzerOn){
        if(time_us_32() / 1000 > lastLineTime + 500){
            gpio_put(buzzer_pin,0);
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
        SetServoAngleFromMain(servo_centor_basket_pin,120);
        sleep_ms(750);
        SetServoAngleFromMain(servo_centor_basket_pin,160);
        sleep_ms(500);
        SetServoOffFromMain(servo_centor_basket_pin);
    }
}

//objectは赤ボールが1,青ボールが2,缶が3
void TrashfromBasket(int object){
    RotationToAngle(90);
    //ゴールまで下がる
    MainMotorState(-200,-200);
    GetDataFromLineToMain();
    sleep_ms(500);
    while(!circleLineSensor[7] || !circleLineSensor[12]){
        sleep_ms(10);
        GetDataFromLineToMain();
    }
    MainMotorState(0,0);
    SetStepperSleep();
    int basket;
    if(object == 1){
        basket = servo_left_basket_pin;
    }else if(object == 2){
        basket = servo_right_basket_pin;
    }else if(object == 3){
        basket = servo_centor_basket_pin;
    }
    SetServoAngleFromMain(basket,80);
    sleep_ms(1000);
    SetServoAngleFromMain(basket,160);
    SetStepperON();    
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

float result;
int VectorNumber;
float VectorAbsoluteValue;
float jujiAngle;
//円形ラインセンサのベクトルの和の向きを計算する
//number : ラインセンサの数
//isFrontLine : 真正面0°にラインセンサがあるのか
//isGetJuji : 十字を取得するか
//最後は時計回りの0～360で出力
float GetCircleLineVector(int number,bool isFrontLine,bool isGetJuji){
    if(number == 0) return 0.0;
    //データを使える形に変換する

    int DoneLineSensor[number];
    float Vector[number];
    int Weight[number];
    VectorNumber = 0;
    for(int i = 0;i < number;i++){
        DoneLineSensor[i] = false;
        Vector[i] = 0;
        Weight[i] = 0;
    }

    for(int i = 0;i < number;i++){
      if(circleLineSensor[i] > 0 && DoneLineSensor[i] == false){
        if(i == 0){
          //LineSensor[0]だけ時計回り側にあるセンサを考える
          int k = number - 1;
          while(k >= 1 && circleLineSensor[k] > 0){
            DoneLineSensor[k] = true;
            k--;
          }
          Vector[VectorNumber] -= (number - 1 - k) * (180.0 / number);
          Weight[VectorNumber] += number - 1 - k;
        }
        int j = 1;
        while(i + j <= number - 1 && circleLineSensor[i+j] > 0){
            DoneLineSensor[i + j] = true;
            j++;
        }
        Vector[VectorNumber] += (j - 1)*(180.0 / number) + (360.0 / number) * i;
        Weight[VectorNumber] += j;

        VectorNumber++;
        DoneLineSensor[i] = true;
        if(VectorNumber >= number) break;
      }
    }

    //ベクトルの合成をする
  float VectorX = 0;
  float VectorY = 0;
  for(int i = 0;i < VectorNumber;i++){
    VectorX -= sin(Vector[i] / 180.0 * 3.1415);
    VectorY += cos(Vector[i] / 180.0 * 3.1415);
    // if(serialWatch == "vec" || serialWatch == "lin"){
    //   printf("%d : %f ",i,Vector[i]);
    // }
  }
  if(VectorNumber == 0){
    VectorX = 999;
    VectorY = 999;
  }else{
    VectorX /= (float)VectorNumber;
    VectorY /= (float)VectorNumber;
  }
  if(isFrontLine == true) result = atan2(VectorY,VectorX) / 3.1415 * -180 + 90;
  else result = atan2(VectorY,VectorX) / 3.1415 * -180 + 90 - (180.0 / number);

  while(result < 0) result += 360.0;
  while(result >= 360) result -= 360.0;
  VectorAbsoluteValue = sqrt(VectorX * VectorX + VectorY * VectorY);

  if(serialWatch == "vec"){
    printf(" 向き : ");
    if(VectorX == 999 && VectorY == 999){
      printf("ラインの上にいない!!\n");
    }else if(VectorX == 0 && VectorY == 0){
      printf("真ん中\n");
    }else{
      printf("%f\n",result);
    }
  }

  //例外処理
  if(VectorX == 999 && VectorY == 999){
    //ラインがない
    result = -999.9;
  }
  if((VectorNumber == 2 && Vector[0] == 0.0 && Vector[1] == 180.0) || (VectorNumber == 1 && Vector[0] == 0) || (VectorNumber == 1 && Vector[0] == 180)){
    //直線上
    result = 999.9;
  }

  //十字の検知
  if(isGetJuji == true){
    if(VectorNumber == 3){
        float jujiVectorX,jujiVectorY;
        jujiAngle = 0;
        if((Vector[0] <= 60 || Vector[0] >= 300) && (Vector[1] <= 60 || Vector[1] >= 300) && (120 <= Vector[2] && Vector[2] <= 240)){
            //0,1が前で2が後ろ
            VectorX = ((sin(Vector[0] / 180.0 * 3.1415) + sin(Vector[1] / 180.0 * 3.1415)) / 2.0 + sin(Vector[2] / 180.0 * 3.1415)) / -2.0;
            VectorY = ((cos(Vector[0] / 180.0 * 3.1415) + cos(Vector[1] / 180.0 * 3.1415)) / 2.0 + cos(Vector[2] / 180.0 * 3.1415)) / 2.0;
            jujiVectorX = (sin(Vector[0] / 180.0 * 3.1415) + sin(Vector[1] / 180.0 * 3.1415)) / -2.0;
            jujiVectorY = (cos(Vector[0] / 180.0 * 3.1415) + cos(Vector[1] / 180.0 * 3.1415)) / 2.0;
        }else if((Vector[0] <= 60 || Vector[0] >= 300) && (Vector[2] <= 60 || Vector[2] >= 300) && (120 <= Vector[1] && Vector[1] <= 240)){
            //0,2が前で1が後ろ
            VectorX = ((sin(Vector[0] / 180.0 * 3.1415) + sin(Vector[2] / 180.0 * 3.1415)) / 2.0 + sin(Vector[1] / 180.0 * 3.1415)) / -2.0;
            VectorY = ((cos(Vector[0] / 180.0 * 3.1415) + cos(Vector[2] / 180.0 * 3.1415)) / 2.0 + cos(Vector[1] / 180.0 * 3.1415)) / 2.0;
            jujiVectorX = (sin(Vector[0] / 180.0 * 3.1415) + sin(Vector[2] / 180.0 * 3.1415)) / -2.0;
            jujiVectorY = (cos(Vector[0] / 180.0 * 3.1415) + cos(Vector[2] / 180.0 * 3.1415)) / 2.0;
        }else if((Vector[0] <= 60 || Vector[0] >= 300) && (120 <= Vector[2] && Vector[2] <= 240) && (120 <= Vector[2] && Vector[2] <= 240)){
            //0が前で1,2が後ろ
            VectorX = ((sin(Vector[1] / 180.0 * 3.1415) + sin(Vector[2] / 180.0 * 3.1415)) / 2.0 + sin(Vector[0] / 180.0 * 3.1415)) / -2.0;
            VectorY = ((cos(Vector[1] / 180.0 * 3.1415) + cos(Vector[2] / 180.0 * 3.1415)) / 2.0 + cos(Vector[0] / 180.0 * 3.1415)) / 2.0;
            jujiVectorX = (sin(Vector[1] / 180.0 * 3.1415) + sin(Vector[2] / 180.0 * 3.1415)) / -2.0;
            jujiVectorY = (cos(Vector[1] / 180.0 * 3.1415) + cos(Vector[2] / 180.0 * 3.1415)) / 2.0;
        }else if((Vector[2] <= 60 || Vector[2] >= 300) && (120 <= Vector[2] && Vector[2] <= 240) && (120 <= Vector[1] && Vector[1] <= 240)){
            //2が前で0,1が後ろ
            VectorX = ((sin(Vector[0] / 180.0 * 3.1415) + sin(Vector[1] / 180.0 * 3.1415)) / 2.0 + sin(Vector[2] / 180.0 * 3.1415)) / -2.0;
            VectorY = ((cos(Vector[0] / 180.0 * 3.1415) + cos(Vector[1] / 180.0 * 3.1415)) / 2.0 + cos(Vector[2] / 180.0 * 3.1415)) / 2.0;
            jujiVectorX = (sin(Vector[0] / 180.0 * 3.1415) + sin(Vector[1] / 180.0 * 3.1415)) / -2.0;
            jujiVectorY = (cos(Vector[0] / 180.0 * 3.1415) + cos(Vector[1] / 180.0 * 3.1415)) / 2.0;
        }else{
            jujiAngle = 999.9;
        }
        if(jujiAngle < 999){
            VectorAbsoluteValue = sqrt(VectorX * VectorX + VectorY * VectorY);
            if(isFrontLine == true) result = atan2(VectorY,VectorX) / 3.1415 * -180 + 90;
            else result = atan2(VectorY,VectorX) / 3.1415 * -180 + 90 - (180.0 / number);
            if(isFrontLine == true) jujiAngle = atan2(jujiVectorY,jujiVectorX) / 3.1415 * -180 + 90;
            else jujiAngle = atan2(jujiVectorY,jujiVectorX) / 3.1415 * -180 + 90 - (180.0 / number);
            while(result < 0) result += 360.0;
            while(result >= 360) result -= 360.0;
            while(jujiAngle < 0) jujiAngle += 360.0;
            while(jujiAngle >= 360) jujiAngle -= 360.0;
            if(serialWatch == "lin"){
                if(isUseDisplay){
                    DrawLineOnDisplay(32+(int)(jujiVectorY * 29.0),32+(int)(jujiVectorX * 29.0),(int)(sqrt(1-(jujiVectorX * jujiVectorX + jujiVectorY * jujiVectorY)) * 29),(jujiAngle) / 180.0 * 3.1415 + 1.5708);
                    DrawLineOnDisplay(32+(int)(jujiVectorY * 29.0),32+(int)(jujiVectorX * 29.0),(int)(sqrt(1-sqrt(jujiVectorX * jujiVectorX + jujiVectorY * jujiVectorY)) * 29),(jujiAngle + 180) / 180.0 * 3.1415 + 1.5708);
                }else{
                    printf(" jujiVector : %.2f ",jujiAngle);
                }
            }
        }
    }else if(VectorNumber == 4){
        float jujiVectorX,jujiVectorY;
        if(Vector[0] <= 45 || Vector[0] > 315){
            //Vector[0]と[2]が縦方向
            VectorX = (sin(Vector[0] / 180.0 * 3.1415) + sin(Vector[2] / 180.0 * 3.1415)) / -2.0;
            VectorY = (cos(Vector[0] / 180.0 * 3.1415) + cos(Vector[2] / 180.0 * 3.1415)) / 2.0;
            jujiVectorX = (sin(Vector[1] / 180.0 * 3.1415) + sin(Vector[3] / 180.0 * 3.1415)) / -2.0;
            jujiVectorY = (cos(Vector[1] / 180.0 * 3.1415) + cos(Vector[3] / 180.0 * 3.1415)) / 2.0;
        }else{
            //Vector[1]と[3]が縦方向
            VectorX = (sin(Vector[1] / 180.0 * 3.1415) + sin(Vector[3] / 180.0 * 3.1415)) / -2.0;
            VectorY = (cos(Vector[1] / 180.0 * 3.1415) + cos(Vector[3] / 180.0 * 3.1415)) / 2.0;
            jujiVectorX = (sin(Vector[0] / 180.0 * 3.1415) + sin(Vector[2] / 180.0 * 3.1415)) / -2.0;
            jujiVectorY = (cos(Vector[0] / 180.0 * 3.1415) + cos(Vector[2] / 180.0 * 3.1415)) / 2.0;
        }
        VectorAbsoluteValue = sqrt(VectorX * VectorX + VectorY * VectorY);
        if(isFrontLine == true) result = atan2(VectorY,VectorX) / 3.1415 * -180 + 90;
        else result = atan2(VectorY,VectorX) / 3.1415 * -180 + 90 - (180.0 / number);
        if(isFrontLine == true) jujiAngle = atan2(jujiVectorY,jujiVectorX) / 3.1415 * -180 + 90;
        else jujiAngle = atan2(jujiVectorY,jujiVectorX) / 3.1415 * -180 + 90 - (180.0 / number);
        while(result < 0) result += 360.0;
        while(result >= 360) result -= 360.0;
        while(jujiAngle < 0) jujiAngle += 360.0;
        while(jujiAngle >= 360) jujiAngle -= 360.0;
        if(serialWatch == "lin"){
            if(isUseDisplay){
                // snprintf(displayBuffer,displayBufferSize,"%.2f",jujiAngle);
                // WriteTextOnDisplay(64,60,displayBuffer,10,false,false);
                DrawLineOnDisplay(32+(int)(jujiVectorY * 29.0),32+(int)(jujiVectorX * 29.0),(int)(sqrt(1-(jujiVectorX * jujiVectorX + jujiVectorY * jujiVectorY)) * 29),(jujiAngle) / 180.0 * 3.1415 + 1.5708);
                DrawLineOnDisplay(32+(int)(jujiVectorY * 29.0),32+(int)(jujiVectorX * 29.0),(int)(sqrt(1-sqrt(jujiVectorX * jujiVectorX + jujiVectorY * jujiVectorY)) * 29),(jujiAngle + 180) / 180.0 * 3.1415 + 1.5708);
            }else{
                printf(" jujiVector : %.2f ",jujiAngle);
            }
        }
    }
  }else{
    jujiAngle = 999.9;
  }

  if(serialWatch == "lin"){
    if(isUseDisplay){
        snprintf(displayBuffer,displayBufferSize,"%.2f",result);
        WriteTextOnDisplay(64,60,displayBuffer,10,false,false);
        if(result > 999){
            DrawLineOnDisplay(7,32,50,0.0);
        }else if(result != -999.9){
            DrawLineOnDisplay(32+(int)(VectorY * 29.0),32+(int)(VectorX * 29.0),(int)(sqrt(1-VectorAbsoluteValue * VectorAbsoluteValue) * 29),(result) / 180.0 * 3.1415 + 1.5708);
            DrawLineOnDisplay(32+(int)(VectorY * 29.0),32+(int)(VectorX * 29.0),(int)(sqrt(1-VectorAbsoluteValue * VectorAbsoluteValue) * 29),(result + 180) / 180.0 * 3.1415 + 1.5708);
        }
    }else{
        printf(" vector : %.2f ",result);
    }
  }
  return result;
}