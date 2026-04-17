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

#define centorX 168 // カメラの中心のX座標

int lineNumber;
int preLineAngle;
int lineAngle;//0,90,180,270が入る本来、車体が直線上であるべき角度
int lastLineTime; //ミリ秒
bool isLineBuzzerOn;
float circleLineAngle;//実際の線の角度1
float catchObjectDistance;//進んだ距離

//GetCircleLineVectorの変数
float result;
int VectorNumber;
float VectorAbsoluteValue;
float jujiAngle;
float TjiAngle;
int objectNumber;

#define RightCircleLine circleLineSensor[11] + circleLineSensor[12] + circleLineSensor[13] + circleLineSensor[14] + circleLineSensor[15] + circleLineSensor[16] + circleLineSensor[17] + circleLineSensor[18] + circleLineSensor[19]
#define LeftCircleLine circleLineSensor[1] + circleLineSensor[2] + circleLineSensor[3] + circleLineSensor[4] + circleLineSensor[5] + circleLineSensor[6] + circleLineSensor[7] + circleLineSensor[8] + circleLineSensor[9]
#define RightFrontCircleLine circleLineSensor[16] + circleLineSensor[17] + circleLineSensor[18]
#define LeftFrontCircleLine circleLineSensor[2] + circleLineSensor[3] + circleLineSensor[4]

int speed;

void LineTraceSetup(){
    lineNumber = 0;
    preLineAngle = 0;
    lineAngle = 0;
    lastLineTime = 0;
    jujiAngle = 0;
    TjiAngle = 0;
    speed = 30;
    objectNumber = 0;
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

uint32_t firstTime;
void MainMove(){
    if(lineNumber == 0 || lineNumber == 1){ //ライン上まで移動
        if(lineNumber == 0) firstTime = time_us_32();
        do{
            PrintDisplayMode();
            GetDataFromLineToMain();
            GetGyroAngleFromSub();
            if(angleX > 180) angleX -= 360;
            if((time_us_32() - firstTime) / 2000 > 400){
                MainMotorState(400 - (int)(angleX * 10),400 + (int)(angleX * 10));
            }else{
                MainMotorState((int)(time_us_32() - firstTime) / 2000 - (int)(angleX * 10),(int)(time_us_32() - firstTime) / 2000 + (int)(angleX * 10));
            }
            SendBufferToDisplay();
        }while(RightFrontCircleLine == 0 || LeftFrontCircleLine == 0);
        do{
            PrintDisplayMode();
            GetDataFromLineToMain();
            GetGyroAngleFromSub();
            if(angleX > 180) angleX -= 360;
            if((time_us_32() - firstTime) / 2000 > 400){
                MainMotorState(400 - (int)(angleX * 10),400 + (int)(angleX * 10));
            }else{
                MainMotorState((int)(time_us_32() - firstTime) / 2000 - (int)(angleX * 10),(int)(time_us_32() - firstTime) / 2000 + (int)(angleX * 10));
            }
            SendBufferToDisplay();
        }while(RightFrontCircleLine > 0 || LeftFrontCircleLine > 0);
        gpio_put(buzzer_pin,1);
        sleep_ms(200);
        gpio_put(buzzer_pin,0);
        lineNumber++;
        if(lineNumber == 2){
            firstTime = time_us_32();
            while((time_us_32() - firstTime) / 1000 > 750){
                PrintDisplayMode();
                StraightLineTrace(0);
                SendBufferToDisplay();
            }
        }
    }else if(lineNumber < 4){ //ライントレース
        NewLineTrace();
    }else if(lineNumber < 5){ //ライントレース～カメラ使用位置まで
        if(time_us_32() / 1000 < lastLineTime + 40000 / speed){
            NewLineTrace();
        }else{
            StraightLineTrace(90);
            if(circleLineAngle < -999){
                lineNumber = 5;
            }
        }
    }else if(lineNumber < 6){ //缶とボールとってからペットボトルとる
        MainMotorState(0,0);
        gpio_put(buzzer_pin,1);
        sleep_ms(500);
        gpio_put(buzzer_pin,0);
        RotationToAngle(180);
        OnWall();
        MainMotorState(0,0);
        //ボールと缶を合わせて4つとって線上に復帰する
        int t = 0;
        while(objectNumber <= 4){
            int obj_num;
            if(t % 2 == 0){
                obj_num = RotationToObject(true);
            }else{
                obj_num = RotationToObject(false);
            }
            t++;
            if(cameraInformation[obj_num].obj_id == 3 || cameraInformation[obj_num].obj_id == 4){
                CatchBall();
            }else if(cameraInformation[obj_num].obj_id == 5 || cameraInformation[obj_num].obj_id == 6){
                CatchCan();
            }else{

            }
            if(3 <= cameraInformation[obj_num].obj_id && cameraInformation[obj_num].obj_id <= 6){
                BackToLine();
                OnWall();
                objectNumber++;
            }else{
                BackToLine();
                OnWall();
            }
        }
        CatchPetBottle();
        BackToLine();
        lineNumber++;
        preLineAngle = 180;
        lineAngle = 180;
        sleep_ms(500);
    }else if(lineNumber < 8){ //後ろライントレース
        BackLineTrace();
    }else if(lineNumber < 9){ //青ボールの排出
        MainMotorState(0,0);
        gpio_put(buzzer_pin,1);
        sleep_ms(500);
        gpio_put(buzzer_pin,0);
        TrashfromBasket(2);
        lineNumber++;
        preLineAngle = 90;
        lineAngle = 90;
        sleep_ms(500);
        lastLineTime = time_us_32() / 1000;
    }else if(lineNumber < 10){//後ろライントレース
        BackLineTrace();
    }else if(lineNumber < 11){//缶の排出
        MainMotorState(0,0);
        gpio_put(buzzer_pin,1);
        sleep_ms(500);
        gpio_put(buzzer_pin,0);
        TrashfromBasket(3);
        lineNumber++;
        preLineAngle = 180;
        lineAngle = 180;
        sleep_ms(500);
        lastLineTime = time_us_32() / 1000;
    }else if(lineNumber < 12){//ライントレース
        NewLineTrace();
    }else if(lineNumber < 13){//赤ボールの排出
        MainMotorState(0,0);
        gpio_put(buzzer_pin,1);
        sleep_ms(500);
        gpio_put(buzzer_pin,0);
        TrashfromBasket(1);
        lineNumber++;
        preLineAngle = 0;
        lineAngle = 0;
    }else if(lineNumber < 14){//島と壁の間を抜ける
        PassTheSpace();
        lineNumber++;
    }else if(lineNumber < 15){//残りを全部取りきる

    }
}

/*
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
*/

//円形ラインセンサを使ったライントレース
void NewLineTrace(){
    GetDataFromLineToMain();
    GetGyroAngleFromSub();
    circleLineAngle = GetCircleLineVector(20,true,true);
    if(VectorNumber == 4 || (TjiAngle < 999 && lineNumber > 3)){
        //十字の感知
        if(time_us_32() / 1000 > lastLineTime + 40000 / speed){
            gpio_put(buzzer_pin,1);
            lineNumber++;
            isLineBuzzerOn = true;
            lastLineTime = time_us_32() / 1000;
        }
    }
    if(90 < circleLineAngle && circleLineAngle < 180){
        //右に曲がる
        MainMotorState((int)(12.5 * speed),-(int)(6.25 * speed));
    }else if(180 < circleLineAngle && circleLineAngle < 270){
        //左に曲がる
        MainMotorState(-(int)(6.25 * speed),(int)(12.5 * speed));
    }else if(VectorAbsoluteValue > 0.4){
        //個々の角度設定が90 - x の関係で間違っている可能性
        if((85 < circleLineAngle && circleLineAngle < 90) || 285 < circleLineAngle ){
            //右に曲がる
            MainMotorState((int)(6.25 * speed),-(int)(6.25 * speed));
        }else if(circleLineAngle < 75 || (270 < circleLineAngle && circleLineAngle < 275)){
            //左に曲がる
            MainMotorState(-(int)(6.25 * speed),(int)(6.25 * speed));
        }else{
            MainMotorState((int)(10 * speed),(int)(10 * speed));
        }
    }else{
        if(lineAngle == 0 && angleX > 180){
            MainMotorState((int)((10 - (angleX - 360) * 0.25) * speed),(int)((10 + (angleX - 360) * 0.25) * speed));
        }else{
            MainMotorState((int)((10 - (angleX - lineAngle) * 0.25) * speed),(int)((10 + (angleX - lineAngle) * 0.25) * speed));
        }
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
        }else if(lineAngle == preLineAngle - 90 && (angleX < lineAngle + 5 || (lineAngle == 0 && angleX > 180))){
            preLineAngle = lineAngle;
        }else if(lineAngle == 270 && preLineAngle == 0 && (180 < angleX && angleX < 275)){
            preLineAngle = lineAngle;
        }
    }
}

void BackLineTrace(){
    GetDataFromLineToMain();
    GetGyroAngleFromSub();
    circleLineAngle = GetCircleLineVector(20,true,true);
    if(VectorNumber == 4 || TjiAngle < 999){
        //十字の感知
        if(time_us_32() / 1000 > lastLineTime + 40000 / speed){
            gpio_put(buzzer_pin,1);
            lineNumber++;
            isLineBuzzerOn = true;
            lastLineTime = time_us_32() / 1000;
        }
    }
    if(270 < circleLineAngle && circleLineAngle < 360){
        //右に曲がる
        MainMotorState((int)(6.25 * speed),-(int)(12.5 * speed));
    }else if(0 < circleLineAngle && circleLineAngle < 90){
        //左に曲がる
        MainMotorState(-(int)(12.5 * speed),(int)(6.25 * speed));
    }else if(VectorAbsoluteValue > 0.4){
        if((265 < circleLineAngle && circleLineAngle < 270) || (105 < circleLineAngle && circleLineAngle < 180)){
            //右に曲がる
            MainMotorState((int)(6.25 * speed),-(int)(6.25 * speed));
        }else if((180 < circleLineAngle && circleLineAngle < 255) || (90 < circleLineAngle && circleLineAngle < 95)){
            //左に曲がる
            MainMotorState(-(int)(6.25 * speed),(int)(6.25 * speed));
        }else{
            MainMotorState(-(int)(10 * speed),-(int)(10 * speed));
        }
    }else{
        if(lineAngle == 0 && angleX > 180){
            MainMotorState((int)((-10 - (angleX - 360) * 0.25) * speed),(int)((-10 + (angleX - 360) * 0.25) * speed));
        }else{
            MainMotorState((int)((-10 - (angleX - lineAngle) * 0.25) * speed),(int)((-10 + (angleX - lineAngle) * 0.25) * speed));
        }
    }
    if(serialWatch == "oth"){
        snprintf(displayBuffer,displayBufferSize,"%d",lineAngle);
        WriteTextOnDisplay(5,30,displayBuffer,12,false,false);
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
        }else if(lineAngle == preLineAngle - 90 && (angleX < lineAngle + 5 || (lineAngle == 0 && angleX > 180))){
            preLineAngle = lineAngle;
        }else if(lineAngle == 270 && preLineAngle == 0 && (180 < angleX && angleX < 275)){
            preLineAngle = lineAngle;
        }
    }
}

//ボールや缶を取った後、線に復帰するプログラム
void BackToLine(){
    GetGyroAngleFromSub();
    int backAngle;
    if(catchObjectDistance < 200000){
        //進まな過ぎて線を検知できないとき
        if(0 <= angleX && angleX < 180){
            //左側にいる
            RotationToAngle(180);
            MainMotorState(100,100);
            sleep_ms(2000u - catchObjectDistance / 100);
            RotationToAngle(90);
            backAngle = 90;
        }else {
            //右側にいる
            RotationToAngle(180);
            MainMotorState(100,100);
            sleep_ms(2000u - catchObjectDistance / 100);
            RotationToAngle(270);
            backAngle = 270;
        }
    }else{
        if(0 <= angleX && angleX < 180){
            //左側にいる
            RotationToAngle(90);
            backAngle = 90;
        }else {
            //右側にいる
            RotationToAngle(270);
            backAngle = 270;
        }
    }
    sleep_ms(250);
    GetDataFromLineToMain();
    circleLineAngle = GetCircleLineVector(20,true,true);
    firstTime = time_us_32();
    uint32_t nowTime = time_us_32() / 1000;
    while(nowTime + 100 > time_us_32() / 1000){
        PrintDisplayMode();
        GetGyroAngleFromSub();
        GetDataFromLineToMain();
        circleLineAngle = GetCircleLineVector(20,true,true);
        if((time_us_32() - firstTime) / 2000 > 150){
            MainMotorState(- 150 - (int)((angleX - backAngle) * 10),- 150 + (int)((angleX - backAngle) * 10));
        }else{
            MainMotorState((int)(time_us_32() - firstTime) / -2000 - (int)((angleX - backAngle) * 10),(int)(time_us_32() - firstTime) / -2000 + (int)((angleX - backAngle) * 10));
        }
        if(circleLineAngle < -999 || VectorAbsoluteValue > 0.4 || (30 < (int)circleLineAngle % 180 && (int)circleLineAngle % 180 < 150 && lineNumber < 3)){
            nowTime = time_us_32() / 1000;
        }
        SendBufferToDisplay();
    }
    MainMotorState(0,0);
    sleep_ms(250);
    RotationToAngle(180);
}

//線に沿って前進するプログラム
//angle : ジャイロで常に向いているべき角度
void StraightLineTrace(int angle){
    GetDataFromLineToMain();
    GetGyroAngleFromSub();
    circleLineAngle = GetCircleLineVector(20,true,true);
    if(90 < circleLineAngle && circleLineAngle < 180 && (angleX - angle < -5 && angle != 0 || (180 < angleX && angleX < 355 && angle == 0))){
        //右に曲がる
        MainMotorState((int)(12.5 * speed),-(int)(6.25 * speed));
    }else if(180 < circleLineAngle && circleLineAngle < 270 && angleX - angle > 5){
        //左に曲がる
        MainMotorState(-(int)(6.25 * speed),(int)(12.5 * speed));
    }else if(VectorAbsoluteValue > 0.4){
        if((85 < circleLineAngle && circleLineAngle < 90) || 285 < circleLineAngle ){
            //右に曲がる
            MainMotorState((int)(6.25 * speed),-(int)(6.25 * speed));
        }else if(circleLineAngle < 75 || (270 < circleLineAngle && circleLineAngle < 275)){
            //左に曲がる
            MainMotorState(-(int)(6.25 * speed),(int)(6.25 * speed));
        }else{
            MainMotorState((int)(10 * speed),(int)(10 * speed));
        }
    }else{
        if(angle == 0 && angleX > 180){
            MainMotorState((int)((10 - (angleX - 360) * 0.25) * speed),(int)((10 + (angleX - 360) * 0.25) * speed));
        }else{
            MainMotorState((int)((10 - (angleX - angle) * 0.25) * speed),(int)((10 + (angleX - angle) * 0.25) * speed));
        }
    }
}

void RotationToAngle(int target_angle){
    SetStepperON();
    //目標角度を0度から360度の間に補正
    target_angle = target_angle % 360;
    if (target_angle < 0) {
        target_angle += 360;
    }
    firstTime = time_us_32();
    while(true){
        PrintDisplayMode();
        GetGyroAngleFromSub();
        //rotation_angleは回転角
        float rotation_angle = target_angle - angleX;
        if(rotation_angle < 0){
            rotation_angle += 360; 
        }
        if(rotation_angle < 0.5 || rotation_angle > 359.5){
            break;
        }
        //回転角が180°未満なら時計回り,180度以上なら反時計回り
        if(rotation_angle < 180){
            if((time_us_32() - firstTime) / 2000 > rotation_angle * 5 + 30){
                if(rotation_angle * 5 + 30 > 300){
                    MainMotorState(300,-300);
                }else{
                    MainMotorState((int)(rotation_angle * 5 + 30),-(int)(rotation_angle * 5 + 30));
                }
            }else{
                MainMotorState((int)(time_us_32() - firstTime) / 2000,-(int)(time_us_32() - firstTime) / 2000);
            }
        }else if(rotation_angle >= 180){
            if((time_us_32() - firstTime) / 2000 > (360 - rotation_angle) * 5 + 30){
                if((360 - rotation_angle) * 5 + 30 > 300){
                    MainMotorState(-300,300);
                }else{
                    MainMotorState(-(int)((360 - rotation_angle) * 5 + 30),(int)((360 - rotation_angle) * 5 + 30));
                }
            }else{
                MainMotorState(-(int)(time_us_32() - firstTime) / 2000,(int)(time_us_32() - firstTime) / 2000);
            }
        }
        SendBufferToDisplay();
    }
    MainMotorState(0,0);
}

//0°と360°の境目で使用禁止
//angle : trueなら右側を向く
int RotationToObject(bool angle){
    GetGyroAngleFromSub();
    firstTime = time_us_32();
    int isGreaterThanCentorX; // 0が初期状態。1が大きい。2が小さい。
    int decidedObj_id;
    while(true){
        PrintDisplayMode();
        int num_objects = UseCamera();
        int Y = 999,number = 0;
        for(int i = 0;i < num_objects;i++){
            if(Y > cameraInformation[i].y && 3 <= cameraInformation[i].obj_id && cameraInformation[i].obj_id <= 6){
                if(isGreaterThanCentorX == 0 || (((isGreaterThanCentorX == 1 && cameraInformation[i].x > 168) || (isGreaterThanCentorX == 2 && cameraInformation[i].x < 168)) && (cameraInformation[i]. obj_id == decidedObj_id || (cameraInformation[i]. obj_id == 5 && decidedObj_id == 6) || (cameraInformation[i]. obj_id == 6 && decidedObj_id == 5)))){
                    if(firstTime / 1000 + 3000 > time_us_32() / 1000 || cameraInformation[i].x > 168){
                        number = i;
                        Y = cameraInformation[i].y;
                        if(isGreaterThanCentorX == 0){
                            if(cameraInformation[i].x > centorX){
                                isGreaterThanCentorX = 1;
                            }else{
                                isGreaterThanCentorX = 2;
                            }
                            decidedObj_id = cameraInformation[i].obj_id;
                        }
                    }
                }
            }
        }
        if(Y == 999){
            MainMotorState(0,0);
            SendBufferToDisplay();
            continue;
        }
        if(cameraInformation[number].x > centorX){
            //左側にある　→　反時計回りに回転
            
            if((cameraInformation[number].x - centorX) * 2 > 400){
                MainMotorState(-400,400);
            }else if((cameraInformation[number].x - centorX) * 2 < 25){
                MainMotorState(-25,25);
            }else{
                MainMotorState((cameraInformation[number].x - centorX) * -2,(cameraInformation[number].x - centorX) * 2);
            }
        }else{
            //右側にある　→　時計回りに回転
            if((centorX - cameraInformation[number].x) * 2 > 400){
                MainMotorState(400,-400);
            }else if((centorX - cameraInformation[number].x) * 2 < 25){
                MainMotorState(25,-25);
            }else{
                MainMotorState((centorX - cameraInformation[number].x) * 2,(centorX - cameraInformation[number].x) * -2);
            }
        }
        if(centorX - 3 < cameraInformation[number].x && cameraInformation[number].x < centorX + 3){
            MainMotorState(0,0);
            return number;
        }
        SendBufferToDisplay();
    }
}

void CatchBall(){
    //tofで探す
    SetServoAngleFromMain(servo_arm_up_and_down_pin,60);
    SetServoAngleFromMain(servo_arm_left_and_right_pin,90);
    uint32_t tofTime = time_us_32();
    SetStepperON();
    int num_objects,d,number;
    catchObjectDistance = 0.0;
    uint32_t prenow,now;
    prenow = time_us_32();
    do{
        PrintDisplayMode();
        num_objects = UseCamera();
        d = 999;
        for(int i = 0;i < num_objects;i++){
            if(d > abs(cameraInformation[i].x - centorX) && 3 <= cameraInformation[i].obj_id && cameraInformation[i].obj_id <= 4){
                number = i;
                d = abs(cameraInformation[i].x - centorX);
            }
        }
        GetGyroAngleFromSub();
        now = time_us_32();
        catchObjectDistance += 100 * sin((angleX - 90) / 180.0 * 3.1416) * (now - prenow) / 1000.0;
        prenow = now;
        if(d = 999){
            MainMotorState(0,0);
            SendBufferToDisplay();
            continue;
        }
        if(cameraInformation[number].x > centorX + 5){
            MainMotorState(95,105);
        }else if(cameraInformation[number].x < centorX - 5){
            MainMotorState(105,95);
        }else{
            MainMotorState(100,100);
        }
        GetDistanceFromSub();
        if(distance > 300){
            tofTime = time_us_32();
        }
        SendBufferToDisplay();
    }while(tofTime + 100000 > time_us_32());
    MainMotorState(0,0);
    SetServoAngleFromMain(servo_arm_up_and_down_pin,155);
    SetServoAngleFromMain(servo_left_claw_pin,160);
    SetServoAngleFromMain(servo_right_claw_pin,20);
    sleep_ms(1000);
    SetServoAngleFromMain(servo_arm_up_and_down_pin,168);
    sleep_ms(1000);
    SetSuctionMotorSpeedFromMain(150);
    sleep_ms(1000);
    MainMotorState(100,100);
    sleep_ms(500);
    MainMotorState(0,0);
    sleep_ms(500);
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
        if(colorTime + 10000000 < time_us_32()){
            //タイムアウト
            SetServoAngleFromMain(servo_arm_left_and_right_pin,90);
            SetServoAngleFromMain(servo_arm_up_and_down_pin,150);
            sleep_ms(1000);
            SetSuctionMotorSpeedFromMain(0);
            sleep_ms(1000);
            SetServoAngleFromMain(servo_arm_up_and_down_pin,60);
            return;
        }
        sleep_ms(10);
    }
    sleep_ms(1000);
    SetSuctionMotorSpeedFromMain(0);
    sleep_ms(2000);
    SetServoAngleFromMain(servo_arm_up_and_down_pin,40);
    sleep_ms(1000);
    SetServoAngleFromMain(servo_arm_up_and_down_pin,60);
}

void CatchCan(){
    //tofで探す
    SetServoAngleFromMain(servo_arm_up_and_down_pin,60);
    SetServoAngleFromMain(servo_arm_left_and_right_pin,90);
    SetServoAngleFromMain(servo_left_claw_pin,160);
    SetServoAngleFromMain(servo_right_claw_pin,20);
    uint32_t tofTime = time_us_32();
    SetStepperON();
    int num_objects,d,number;
    catchObjectDistance = 0.0;
    uint32_t prenow,now;
    prenow = time_us_32();
    do{
        PrintDisplayMode();
        num_objects = UseCamera();
        d = 999;
        for(int i = 0;i < num_objects;i++){
            if(d > abs(cameraInformation[i].x - centorX) && 3 <= cameraInformation[i].obj_id && cameraInformation[i].obj_id <= 4){
                number = i;
                d = abs(cameraInformation[i].x - centorX);
            }
        }
        GetGyroAngleFromSub();
        now = time_us_32();
        catchObjectDistance += 100 * sin((angleX - 90) / 180.0 * 3.1416) * (now - prenow) / 1000.0;
        prenow = now;
        if(d = 999){
            MainMotorState(0,0);
            SendBufferToDisplay();
            continue;
        }
        if(cameraInformation[number].x > centorX + 5){
            MainMotorState(95,105);
        }else if(cameraInformation[number].x < centorX - 5){
            MainMotorState(105,95);
        }else{
            MainMotorState(100,100);
        }
        GetDistanceFromSub();
        if(distance > 300){
            tofTime = time_us_32();
        }
        SendBufferToDisplay();
    }while(tofTime + 100000 > time_us_32());
    MainMotorState(0,0);
    SetServoAngleFromMain(servo_arm_up_and_down_pin,155);
    sleep_ms(1000);
    SetServoAngleFromMain(servo_arm_up_and_down_pin,168);
    sleep_ms(250);
    //少し前進する？
    firstTime = time_us_32();
    while(time_us_32() / 1000 < firstTime / 1000 + 1000){
        PrintDisplayMode();
        GetGyroAngleFromSub();
        if((time_us_32() - firstTime) / 2000 > 100){
            MainMotorState(100 ,100);
        }else{
            MainMotorState((int)(time_us_32() - firstTime) / 2000,(int)(time_us_32() - firstTime) / 2000);
        }
        SendBufferToDisplay();
    }
    MainMotorState(0,0);
    sleep_ms(1000);
    SetServoAngleFromMain(servo_arm_up_and_down_pin,50);
    GetColorFromSub();
    if(color == 1 || color == 3)canNumber += 1;
    sleep_ms(1000);
    SetServoAngleFromMain(servo_left_claw_pin,160);
    SetServoAngleFromMain(servo_right_claw_pin,20);
    sleep_ms(1000);
    SetServoAngleFromMain(servo_arm_up_and_down_pin,60);
}

//objectは赤ボールが1,青ボールが2,缶が3
//赤ボールの時だけ終了時が十字条件を満たさない
void TrashfromBasket(int object){
    RotationToAngle(90);
    GetDataFromLineToMain();
    circleLineAngle = GetCircleLineVector(20,true,true);
    firstTime = time_us_32();
    //ゴールまで下がる
    while(!circleLineSensor[7] || !circleLineSensor[12] || (time_us_32() - firstTime) / 1000 < 750){
        PrintDisplayMode();
        GetGyroAngleFromSub();
        GetDataFromLineToMain();
        circleLineAngle = GetCircleLineVector(20,true,true);
        if((time_us_32() - firstTime) / 2000 > 200){
            MainMotorState(- 200 - (int)((angleX - 90) * 10),- 200 + (int)((angleX - 90) * 10));
        }else{
            MainMotorState((int)(time_us_32() - firstTime) / -2000 - (int)((angleX - 90) * 10),(int)(time_us_32() - firstTime) / -2000 + (int)((angleX - 90) * 10));
        }
        SendBufferToDisplay();
    }
    MainMotorState(0,0);
    // SetStepperSleep();
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
    sleep_ms(500);
    // SetStepperON();
    GetDataFromLineToMain();
    circleLineAngle = GetCircleLineVector(20,true,true);
    firstTime = time_us_32();
    if(object == 1){
        while(circleLineSensor[5] + circleLineSensor[6] == 0  || (time_us_32() - firstTime) / 1000 < 750){
            PrintDisplayMode();
            GetGyroAngleFromSub();
            GetDataFromLineToMain();
            circleLineAngle = GetCircleLineVector(20,true,true);
            if((time_us_32() - firstTime) / 2000 > 200){
                MainMotorState(200 - (int)((angleX - 90) * 10),200 + (int)((angleX - 90) * 10));
            }else{
                MainMotorState((int)(time_us_32() - firstTime) / -2000 - (int)((angleX - 90) * 10),(int)(time_us_32() - firstTime) / -2000 + (int)((angleX - 90) * 10));
            }
            SendBufferToDisplay();
        }
        MainMotorState(0,0);
        sleep_ms(250);
        RotationToAngle(0);
    }else{
        while(VectorNumber != 4  || (time_us_32() - firstTime) / 1000 < 750){
            PrintDisplayMode();
            GetGyroAngleFromSub();
            GetDataFromLineToMain();
            circleLineAngle = GetCircleLineVector(20,true,true);
            if((time_us_32() - firstTime) / 2000 > 200){
                MainMotorState(200 - (int)((angleX - 90) * 10),200 + (int)((angleX - 90) * 10));
            }else{
                MainMotorState((int)(time_us_32() - firstTime) / -2000 - (int)((angleX - 90) * 10),(int)(time_us_32() - firstTime) / -2000 + (int)((angleX - 90) * 10));
            }
            SendBufferToDisplay();
        }
        MainMotorState(0,0);
        sleep_ms(250);
        if(object == 3){
            RotationToAngle(180);
            sleep_ms(250);
        }
    }
}

void CatchPetBottle(){
    sleep_ms(100);
    GetDistanceFromSub();
    firstTime = time_us_32();
    uint32_t deltaTime = 0;
    uint32_t preTime = time_us_32() / 1000;
    while(deltaTime < 250){
        PrintDisplayMode();
        GetDistanceFromSub();
        GetGyroAngleFromSub();
        if((time_us_32() - firstTime) / 2000 > 100){
            MainMotorState(100 - (int)((angleX - 180) * 10),100 + (int)((angleX - 180) * 10));
        }else{
            MainMotorState((int)(time_us_32() - firstTime) / 2000 - (int)((angleX - 180) * 10),(int)(time_us_32() - firstTime) / -2000 + (int)((angleX - 180) * 10));
        }
        SendBufferToDisplay();
        if(distance < 330){
            deltaTime += time_us_32() / 1000 - preTime;
        }else{
            deltaTime = 0;
        }
        preTime = time_us_32() / 1000;
    }
    MainMotorState(0,0);
    
    SetServoAngleFromMain(servo_arm_left_and_right_pin,90);
    SetServoAngleFromMain(servo_left_claw_pin,90);
    SetServoAngleFromMain(servo_right_claw_pin,90);
    SetServoAngleFromMain(servo_arm_up_and_down_pin,160);
    sleep_ms(1000);
    SetServoAngleFromMain(servo_arm_up_and_down_pin,163);
    sleep_ms(1000);

    firstTime = time_us_32() / 1000;
    while(time_us_32() / 1000 < firstTime + 2000){
        PrintDisplayMode();
        GetGyroAngleFromSub();
        if((time_us_32() - firstTime) / 2000 > 100){
            MainMotorState(100 - (int)((angleX - 180) * 10),100 + (int)((angleX - 180) * 10));
        }else{
            MainMotorState((int)(time_us_32() - firstTime) / 2000 - (int)((angleX - 180) * 10),(int)(time_us_32() - firstTime) / 2000 + (int)((angleX - 180) * 10));
        }
        SendBufferToDisplay();
    }
    MainMotorState(0,0);

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
    sleep_ms(500);
    SetServoAngleFromMain(servo_left_claw_pin,50);
    SetServoAngleFromMain(servo_right_claw_pin,130);
    sleep_ms(1000);
    MainMotorState(-200,-200);
    sleep_ms(100);
    MainMotorState(0,0);
    SetServoAngleFromMain(servo_left_claw_pin,90);
    SetServoAngleFromMain(servo_right_claw_pin,90);
    sleep_ms(1000);
    SetServoAngleFromMain(servo_arm_up_and_down_pin,60);
    sleep_ms(500);
    SetServoAngleFromMain(servo_arm_left_and_right_pin,90);
    sleep_ms(1000);
}

//カメラを使う位置に戻る。線上にいる前提
void OnWall(){
    firstTime = time_us_32();
    while(!gpio_get(touch_sensor_back_left_pin)){
        PrintDisplayMode();
        GetGyroAngleFromSub();
        if((time_us_32() - firstTime) / 2000 > 200){
            MainMotorState(- 200 - (int)((angleX - 180) * 10),- 200 + (int)((angleX - 180) * 10));
        }else{
            MainMotorState((int)(time_us_32() - firstTime) / -2000 - (int)((angleX - 180) * 10),(int)(time_us_32() - firstTime) / -2000 + (int)((angleX - 180) * 10));
        }
        SendBufferToDisplay();
    }
}

//島と壁の間を抜ける処理
void PassTheSpace(){
    firstTime = time_us_32();
    while((time_us_32() - firstTime) / 1000 < 500){
        PrintDisplayMode();
        GetGyroAngleFromSub();
        GetDataFromLineToMain();
        if(angleX > 180) angleX -= 360;
        if((time_us_32() - firstTime) / 2000 > 200){
            MainMotorState(-200 - (int)(angleX * 10),-200 + (int)(angleX * 10));
        }else{
            MainMotorState((int)(time_us_32() - firstTime) / -2000 - (int)(angleX * 10),(int)(time_us_32() - firstTime) / -2000 + (int)(angleX * 10));
        }
        SendBufferToDisplay();
    }
    MainMotorState(0,0);
    sleep_ms(100);
    RotationToAngle(90);
    sleep_ms(250);
    firstTime = time_us_32();
    while((time_us_32() - firstTime) / 1000 < 1500){
        PrintDisplayMode();
        GetDataFromLineToMain();
        GetGyroAngleFromSub();
        if((time_us_32() - firstTime) / 2000 > 400){
            MainMotorState(400 - (int)((angleX-90) * 10),400 + (int)((angleX-90) * 10));
        }else{
            MainMotorState((int)(time_us_32() - firstTime) / 2000 - (int)((angleX-90) * 10),(int)(time_us_32() - firstTime) / 2000 + (int)((angleX-90) * 10));
        }
        SendBufferToDisplay();
    }
    firstTime = time_us_32();
    while((time_us_32() - firstTime) / 2000 < 400){
        PrintDisplayMode();
        GetDataFromLineToMain();
        GetGyroAngleFromSub();
        
        MainMotorState(400 - (int)(time_us_32() - firstTime) / 2000 - (int)((angleX - 90) * 10),400 - (int)(time_us_32() - firstTime) / 2000 + (int)((angleX - 90) * 10));

        SendBufferToDisplay();
    }
    MainMotorState(0,0);
}

void UseAllSensor(){
    UseCamera();
    GetDataFromLineToMain();
    GetGyroAngleFromSub();
    GetDistanceFromSub();
    GetColorFromSub();
    GetCurrentFromSub();
}

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
    VectorAbsoluteValue = 0.0;
  }
  if((VectorNumber == 2 && Vector[0] == 0.0 && Vector[1] == 180.0) || (VectorNumber == 1 && Vector[0] == 0) || (VectorNumber == 1 && Vector[0] == 180)){
    //直線上
    result = 999.9;
  }

  //十字の検知
  if(isGetJuji == true){
    if(VectorNumber == 3){
        float jujiVectorX,jujiVectorY;
        TjiAngle = 999.9;
        jujiAngle = 999.9;
        float comVector;
        if((Vector[0] <= 60 || Vector[0] >= 300) && (Vector[1] <= 60 || Vector[1] >= 300) && (120 <= Vector[2] && Vector[2] <= 240)){
            //0,1が前で2が後ろ
            if(Vector[0] > 180) comVector = (Vector[0] - 360.0 + Vector[1])/2.0;
            else comVector = (Vector[0] + Vector[1])/2.0;
            VectorX = (sin(comVector / 180.0 * 3.1415) + sin(Vector[2] / 180.0 * 3.1415)) / -2.0;
            VectorY = (cos(comVector / 180.0 * 3.1415) + cos(Vector[2] / 180.0 * 3.1415)) / 2.0;
            jujiVectorX = (sin(Vector[0] / 180.0 * 3.1415) + sin(Vector[1] / 180.0 * 3.1415)) / -2.0;
            jujiVectorY = (cos(Vector[0] / 180.0 * 3.1415) + cos(Vector[1] / 180.0 * 3.1415)) / 2.0;
            jujiAngle = 0;
        }else if((Vector[0] <= 60 || Vector[0] >= 300) && (Vector[2] <= 60 || Vector[2] >= 300) && (120 <= Vector[1] && Vector[1] <= 240)){
            //0,2が前で1が後ろ
            if(Vector[0] > 180) comVector = (Vector[0] - 360.0 + Vector[2] - 360.0)/2.0;
            else comVector = (Vector[0] + Vector[2] - 360.0)/2.0;
            VectorX = (sin(comVector / 180.0 * 3.1415) + sin(Vector[1] / 180.0 * 3.1415)) / -2.0;
            VectorY = (cos(comVector / 180.0 * 3.1415) + cos(Vector[1] / 180.0 * 3.1415)) / 2.0;
            jujiVectorX = (sin(Vector[0] / 180.0 * 3.1415) + sin(Vector[2] / 180.0 * 3.1415)) / -2.0;
            jujiVectorY = (cos(Vector[0] / 180.0 * 3.1415) + cos(Vector[2] / 180.0 * 3.1415)) / 2.0;
            jujiAngle = 0;
        }else if((Vector[0] <= 60 || Vector[0] >= 300) && (120 <= Vector[2] && Vector[2] <= 240) && (120 <= Vector[1] && Vector[1] <= 240)){
            //0が前で1,2が後ろ
            comVector = (Vector[1] + Vector[2])/2.0;
            VectorX = (sin(comVector / 180.0 * 3.1415) + sin(Vector[0] / 180.0 * 3.1415)) / -2.0;
            VectorY = (cos(comVector / 180.0 * 3.1415) + cos(Vector[0] / 180.0 * 3.1415)) / 2.0;
            jujiVectorX = (sin(Vector[1] / 180.0 * 3.1415) + sin(Vector[2] / 180.0 * 3.1415)) / -2.0;
            jujiVectorY = (cos(Vector[1] / 180.0 * 3.1415) + cos(Vector[2] / 180.0 * 3.1415)) / 2.0;
            jujiAngle = 0;
        }else if((Vector[2] <= 60 || Vector[2] >= 300) && (120 <= Vector[0] && Vector[0] <= 240) && (120 <= Vector[1] && Vector[1] <= 240)){
            //2が前で0,1が後ろ
            comVector = (Vector[1] + Vector[0])/2.0;
            VectorX = (sin(comVector / 180.0 * 3.1415) + sin(Vector[2] / 180.0 * 3.1415)) / -2.0;
            VectorY = (cos(comVector / 180.0 * 3.1415) + cos(Vector[2] / 180.0 * 3.1415)) / 2.0;
            jujiVectorX = (sin(Vector[0] / 180.0 * 3.1415) + sin(Vector[1] / 180.0 * 3.1415)) / -2.0;
            jujiVectorY = (cos(Vector[0] / 180.0 * 3.1415) + cos(Vector[1] / 180.0 * 3.1415)) / 2.0;
            jujiAngle = 0;
        }else if((Vector[0] <= 60 || Vector[0] >= 300) && (45 <= Vector[1] && Vector[1] <= 135) && (225 <= Vector[2] && Vector[2] <= 315)){
            //0が前のT字
            VectorX = sin(Vector[0] / 180.0 * 3.1415) * -1.0;
            VectorY = cos(Vector[0] / 180.0 * 3.1415);
            jujiVectorX = (sin(Vector[1] / 180.0 * 3.1415) + sin(Vector[2] / 180.0 * 3.1415)) / -2.0;
            jujiVectorY = (cos(Vector[1] / 180.0 * 3.1415) + cos(Vector[2] / 180.0 * 3.1415)) / 2.0;
            TjiAngle = 0;
        }else if((Vector[2] <= 60 || Vector[2] >= 300) && (45 <= Vector[1] && Vector[1] <= 135) && (225 <= Vector[0] && Vector[0] <= 315)){
            //2が前のT字
            VectorX = sin(Vector[2] / 180.0 * 3.1415) * -1.0;
            VectorY = cos(Vector[2] / 180.0 * 3.1415);
            jujiVectorX = (sin(Vector[1] / 180.0 * 3.1415) + sin(Vector[0] / 180.0 * 3.1415)) / -2.0;
            jujiVectorY = (cos(Vector[1] / 180.0 * 3.1415) + cos(Vector[0] / 180.0 * 3.1415)) / 2.0;
            TjiAngle = 0;
        }else if((120 <= Vector[1] && Vector[1] <= 240) && (45 <= Vector[0] && Vector[0] <= 135) && (225 <= Vector[2] && Vector[2] <= 315)){
            //1が後ろのT字
            VectorX = sin(Vector[1] / 180.0 * 3.1415) * -1.0;
            VectorY = cos(Vector[1] / 180.0 * 3.1415);
            jujiVectorX = (sin(Vector[2] / 180.0 * 3.1415) + sin(Vector[0] / 180.0 * 3.1415)) / -2.0;
            jujiVectorY = (cos(Vector[2] / 180.0 * 3.1415) + cos(Vector[0] / 180.0 * 3.1415)) / 2.0;
            TjiAngle = 0;
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
        }else if(TjiAngle < 999){
            VectorAbsoluteValue = 0;
            if(isFrontLine == true) result = atan2(VectorY,VectorX) / 3.1415 * -180;
            else result = atan2(VectorY,VectorX) / 3.1415 * -180 - (180.0 / number);
            if(isFrontLine == true) TjiAngle = atan2(jujiVectorY,jujiVectorX) / 3.1415 * -180 + 90;
            else TjiAngle = atan2(jujiVectorY,jujiVectorX) / 3.1415 * -180 + 90 - (180.0 / number);
            while(result < 0) result += 360.0;
            while(result >= 360) result -= 360.0;
            while(TjiAngle < 0) TjiAngle += 360.0;
            while(TjiAngle >= 360) TjiAngle -= 360.0;
            if(serialWatch == "lin"){
                if(isUseDisplay){
                    DrawLineOnDisplay(32+(int)(jujiVectorY * 29.0),32+(int)(jujiVectorX * 29.0),(int)(sqrt(1-sqrt(jujiVectorX * jujiVectorX + jujiVectorY * jujiVectorY)) * 29),(TjiAngle) / 180.0 * 3.1415 + 1.5708);
                    DrawLineOnDisplay(32+(int)(jujiVectorY * 29.0),32+(int)(jujiVectorX * 29.0),(int)(sqrt(1-sqrt(jujiVectorX * jujiVectorX + jujiVectorY * jujiVectorY)) * 29),(TjiAngle + 180) / 180.0 * 3.1415 + 1.5708);
                }else{
                    printf(" jujiVector : %.2f ",TjiAngle);
                }
            }
        }
    }else if(VectorNumber == 4){
        TjiAngle = 999.9;
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
                DrawLineOnDisplay(32+(int)(jujiVectorY * 29.0),32+(int)(jujiVectorX * 29.0),(int)(sqrt(1-sqrt(jujiVectorX * jujiVectorX + jujiVectorY * jujiVectorY)) * 29),(jujiAngle) / 180.0 * 3.1415 + 1.5708);
                DrawLineOnDisplay(32+(int)(jujiVectorY * 29.0),32+(int)(jujiVectorX * 29.0),(int)(sqrt(1-sqrt(jujiVectorX * jujiVectorX + jujiVectorY * jujiVectorY)) * 29),(jujiAngle + 180) / 180.0 * 3.1415 + 1.5708);
            }else{
                printf(" jujiVector : %.2f ",jujiAngle);
            }
        }
    }else{
        jujiAngle = 999.9;
        TjiAngle = 999.9;
    }
  }else{
    jujiAngle = 999.9;
    TjiAngle = 999.9;
  }

  if(serialWatch == "lin"){
    if(isUseDisplay){
        snprintf(displayBuffer,displayBufferSize,"%.2f",result);
        WriteTextOnDisplay(64,60,displayBuffer,10,false,false);
        snprintf(displayBuffer,displayBufferSize,"%.2f",VectorAbsoluteValue);
        WriteTextOnDisplay(90,38,displayBuffer,10,false,false);
        if(result > 999){
            DrawLineOnDisplay(7,32,50,0.0);
        }else if(result != -999.9){
            if(TjiAngle > 999) DrawLineOnDisplay(32+(int)(VectorY * 29.0),32+(int)(VectorX * 29.0),(int)(sqrt(1-VectorAbsoluteValue * VectorAbsoluteValue) * 29),(result) / 180.0 * 3.1415 + 1.5708);
            DrawLineOnDisplay(32+(int)(VectorY * 29.0),32+(int)(VectorX * 29.0),(int)(sqrt(1-VectorAbsoluteValue * VectorAbsoluteValue) * 29),(result + 180) / 180.0 * 3.1415 + 1.5708);
        }
    }else{
        printf(" vector : %.2f ",result);
    }
  }
  return result;
}