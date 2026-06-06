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

#define centerX 168 // カメラの中心のX座標

//共有変数
int task;
uint32_t allFirstTime;
bool isYosen = false;

//非共有変数
int preLineAngle;
int lineAngle;//0,90,180,270が入る本来、車体が直線上であるべき角度
int lastLineTime; //ミリ秒、最後に横線を読んだ時刻
bool isLineBuzzerOn;
float circleLineAngle;//実際の線の角度1 -999でlineがない
float catchObjectDistance;//進んだ距離,380000以下で進まな過ぎた時の処理、480000以上で進みすぎた時の処理がある
bool isCatchHorizonCan;//横缶を取ったか
int speed;//*10される。20～40程度
uint32_t firstTime;
bool isNotPlusTask = false;//taskを横線で加算するのを停止するかどうか
int unWatchTime = 0;//対象を見ていない時間の計測用,壁あての時間計測にも使用
uint32_t pPreTime = 0;
float pLeft = 0.0;//ライントレースP制御の左タイヤ
float pRight = 0.0;//ライントレースP制御の右タイヤ

//対象の数とか
int redBallNumber = 0;
int blueBallNumber = 0;
int canNumber = 0;
int allRedBallNumber;//6個
int allBlueBallNumber;//2個
int allCanNumber;//縦缶3個,横缶1個
int objectNumber;//赤6個,青2個,縦缶3個,横缶1個,合計12個
//1 : 赤が残り5,6個,青が残り0個
//2 : 赤が残り4個,青が残り1個,缶が残り2個
//3 : それ以外
int objectTask;

//GetCircleLineVectorの変数
float result;
int VectorNumber;
float VectorAbsoluteValue;
float jujiAngle;
float TjiAngle;

#define TurnSpeed 6

#define RightCircleLine circleLineSensor[11] + circleLineSensor[12] + circleLineSensor[13] + circleLineSensor[14] + circleLineSensor[15] + circleLineSensor[16] + circleLineSensor[17] + circleLineSensor[18] + circleLineSensor[19]
#define LeftCircleLine circleLineSensor[1] + circleLineSensor[2] + circleLineSensor[3] + circleLineSensor[4] + circleLineSensor[5] + circleLineSensor[6] + circleLineSensor[7] + circleLineSensor[8] + circleLineSensor[9]
#define RightFrontCircleLine circleLineSensor[16] + circleLineSensor[17] + circleLineSensor[18]
#define LeftFrontCircleLine circleLineSensor[2] + circleLineSensor[3] + circleLineSensor[4]

//ライントレースに関する変数の初期化
void LineTraceSetup(){
    task = 0;
    canNumber = 0;
    preLineAngle = 0;
    lineAngle = 0;
    lastLineTime = 0;
    jujiAngle = 0;
    TjiAngle = 0;
    speed = 40;
    objectNumber = 0;
    isCatchHorizonCan = false;
    allRedBallNumber = 0;
    allBlueBallNumber = 0;
    allCanNumber = 0;
    objectTask = 0;
}

//taskによって規定されるメインの動き
void MainMove(){
    if(task == 0 || task == 1){ //ライン上まで移動
        if(task == 0) firstTime = time_us_32();
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
            if((time_us_32() - firstTime) / 1000 > 4000){
                task = 1;
                MainMotorState(0,0);
                break;
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
            if((time_us_32() - firstTime) / 1000 > 4000){
                task = 1;
                MainMotorState(0,0);
                break;
            }
            SendBufferToDisplay();
        }while(RightFrontCircleLine > 0 || LeftFrontCircleLine > 0);
        gpio_put(buzzer_pin,1);
        sleep_ms(200);
        gpio_put(buzzer_pin,0);
        task++;
        if(task == 2){
            uint32_t firstTime2 = time_us_32();
            while((time_us_32() - firstTime2) / 1000 < 750){
                PrintDisplayMode();
                StraightLineTrace(0,speed);
                SendBufferToDisplay();
                if((time_us_32() - firstTime) / 1000 > 4000){
                    MainMotorState(0,0);
                    break;
                }
            }
            pLeft = speed * 10;pRight = speed * 10;
        }
    }else if(task <= 3){//ライントレース
        NewLineTrace();
        if(135 < angleX && angleX < 270){
            //読み落とし
            while(task <= 3){
                PrintDisplayMode();
                BackLineTrace();
                SendBufferToDisplay();
            }
        }
    }else if(task <= 4){//ライントレース～カメラ使用位置まで
        if(150 < angleX && angleX < 270){
            task = 5;
        }else{
            if(time_us_32() / 1000 < lastLineTime + 40000 / speed){
                NewLineTrace();
            }else{
                StraightLineTrace(90,speed);
                if(circleLineAngle < -999){
                    task = 5;
                }
            }
        }
    }else if(task <= 5){//Dエリアの缶とボールとってからペットボトルとる
        MainMotorState(0,0);
        gpio_put(buzzer_pin,1);
        sleep_ms(500);
        gpio_put(buzzer_pin,0);
        RotationToAngle(180);
        OnWall(180);
        MainMotorState(0,0);
        //ボールと缶を合わせて4つとって線上に復帰する
        int objID;
        bool isStraight = false;
        while(objectNumber < 5 && ((time_us_32() - allFirstTime) / 1000000 < 220 || !isYosen)){
            // uint8_t cmd = 0x23;
            // uart_write_blocking(camera_uart,&cmd,1);
            sleep_ms(1);
            objID = RotationToObject();
            if(objID == 888 || objID == 999){
                if(isStraight){
                    break;
                }else{
                    DaikeiKasokuLoop(1400,200,180);
                    isStraight = true;
                }
            }
            if(isStraight){
                catchObjectDistance = 1200.0 * 200.0;
            }else{
                catchObjectDistance = 0.0;
            }
            
            if(objID == 3 || objID == 4){
                CatchBall(false);
                isStraight = false;
            }else if(objID == 5 || objID == 6){
                CatchCan();
                isStraight = false;
            }
            
            UseColorLED(0,0,0);
            if(!(objID == 888 || objID == 999)){
                BackToLine();
                RotationToAngle(180);
                if(objectNumber == 5){
                    firstTime = time_us_32();
                    DaikeiKasokuLoop(250,-100,999);
                    sleep_ms(200);
                    // ResetGyro(180);
                }else{
                    OnWall(180);
                    sleep_ms(200);
                    ResetGyro(180);
                }
            }
        }
        UseColorLED(255,255,255);
        CatchPetBottle();
        UseColorLED(0,0,0);
        BackToLine();
        RotationToAngle(180);
        task++;
        preLineAngle = 180;
        lineAngle = 180;
        sleep_ms(300);
        UseColorLED(0,0,0);
        if(allBlueBallNumber == 2 && allCanNumber > 0){
            //残りの球は赤だけ
            objectTask = 1;
        }else if(allBlueBallNumber == 1 && allRedBallNumber == 2 && allCanNumber == 2){
            //前半すべて取り切れている
            objectTask = 2;
        }else{
            //ゴミ
            objectTask = 3;
        }
        pLeft = 0;pRight = 0;
    }else if(task <= 6){//後ろライントレース
        BackLineTrace();
    }else if(task <= 7){//壁あてしてジャイロのリセット
        gpio_put(buzzer_pin,0);
        MainMotorState(0,0);
        GetGyroAngleFromSub();
        RotationToAngle(180);
        firstTime = time_us_32();
        while(!gpio_get(touch_sensor_back_left_pin) || !gpio_get(touch_sensor_back_right_pin)){
            PrintDisplayMode();
            if(gpio_get(touch_sensor_back_left_pin) || gpio_get(touch_sensor_back_right_pin)){
                DaikeiKasoku(-50,999);
                if((time_us_32() - unWatchTime) / 1000 > 1000){
                    break;
                }  
            }else{
                DaikeiKasoku(-400,999);
                unWatchTime = time_us_32(); //関係ない変数
            }
            SendBufferToDisplay();
        }
        MainMotorState(0,0);
        sleep_ms(200);
        ResetGyro(180);
        GetDataFromLineToMain();
        circleLineAngle = GetCircleLineVector(20,true,true);
        uint32_t nowTime = time_us_32() / 1000;
        firstTime = time_us_32();
        while(nowTime + 100 > time_us_32() / 1000){
            PrintDisplayMode();
            GetDataFromLineToMain();
            circleLineAngle = GetCircleLineVector(20,true,true);
            DaikeiKasoku(200,180);
            if((circleLineAngle < -999 || VectorAbsoluteValue > 0.4 || (30 < ((int)circleLineAngle % 180) && ((int)circleLineAngle % 180) < 150 && VectorNumber < 3) || (circleLineSensor[5] == 0 && circleLineSensor[15] == 0 && VectorNumber > 2))){
                nowTime = time_us_32() / 1000;
            }
            if((time_us_32() - firstTime) / 1000 > 4300){
                isNotPlusTask = true;
                break;
            }
            SendBufferToDisplay();
        }
        RotationToAngle(90);
        DaikeiKasokuLoop(1000,200,90);
        catchObjectDistance = 400000;
        BackToLine();
        RotationToAngle(180);
        MainMotorState(0,0);
        gpio_put(buzzer_pin,1);
        sleep_ms(500);
        gpio_put(buzzer_pin,0);
        lastLineTime = time_us_32() / 1000;
        if(isNotPlusTask) lastLineTime = 0;
        task++;
        pLeft = 0;pRight = 0;
    }else if(task <= 8){//後ろライントレース
        BackLineTrace();
    }else if(task <= 9){//青ボールの排出
        MainMotorState(0,0);
        gpio_put(buzzer_pin,1);
        sleep_ms(500);
        gpio_put(buzzer_pin,0);
        TrashfromBasket(2);
        pLeft = 0;pRight = 0;
        task++;
        preLineAngle = 90;
        lineAngle = 90;
        sleep_ms(500);
        lastLineTime = time_us_32() / 1000;
    }else if(task <= 10){//後ろライントレース
        BackLineTrace();
    }else if(task <= 11){//缶の排出
        MainMotorState(0,0); 
        gpio_put(buzzer_pin,1);
        sleep_ms(500);
        gpio_put(buzzer_pin,0);
        if(allCanNumber > 0){
            TrashfromBasket(3);
        }
        task++;
        preLineAngle = 0;
        lineAngle = 0;
        sleep_ms(500);
        lastLineTime = time_us_32() / 1000;
        pLeft = 0;pRight = 0;
    }else if(task <= 12){//ライントレース
        BackLineTrace();
    }else if(task <= 13){//赤ボールの排出 →　壁あてしてジャイロのリセット
        MainMotorState(0,0);
        gpio_put(buzzer_pin,1);
        sleep_ms(500);
        gpio_put(buzzer_pin,0);
        if(allRedBallNumber > 0){
            TrashfromBasket(1);
        }
        OnWall(0);
        firstTime = time_us_32();
        MainMotorState(0,0);
        ResetGyro(0);
        if(isYosen){
            for(int i = 0;i < 3;i++){
                gpio_put(buzzer_pin,1);
                sleep_ms(300);
                gpio_put(buzzer_pin,0);
                sleep_ms(300);
            }
            sleep_ms(100000000);
        }
        gpio_put(buzzer_pin,1);
        sleep_ms(500);
        gpio_put(buzzer_pin,0);
        task++;
        preLineAngle = 0;
        lineAngle = 0;
        firstTime = time_us_32();
    }else if(task <= 15){//ライン上まで移動
        if(task == 14) firstTime = time_us_32();
        do{
            PrintDisplayMode();
            GetDataFromLineToMain();
            DaikeiKasoku(400,0);
            SendBufferToDisplay();
            if((time_us_32() - firstTime) / 1000 > 4500){
                task = 15;
                MainMotorState(0,0);
                break;
            }
        }while(RightFrontCircleLine == 0 || LeftFrontCircleLine == 0);
        do{
            PrintDisplayMode();
            GetDataFromLineToMain();
            DaikeiKasoku(400,0);
            SendBufferToDisplay();
            if((time_us_32() - firstTime) / 1000 > 4500){
                task = 15;
                MainMotorState(0,0);
                break;
            }
        }while(RightFrontCircleLine > 0 || LeftFrontCircleLine > 0);
        gpio_put(buzzer_pin,1);
        sleep_ms(200);
        gpio_put(buzzer_pin,0);
        task++;
        if(task == 16){
            firstTime = time_us_32();
            while((time_us_32() - firstTime) / 1000 < 750){
                PrintDisplayMode();
                StraightLineTrace(0,speed);
                SendBufferToDisplay();
                if((time_us_32() - firstTime) / 1000 > 4500){
                    break;
                }
            }
            pLeft = speed * 10;pRight = speed * 10;
        }
    }else if(task <= 17){//ライントレース
        NewLineTrace();
    }else if(task <= 18){
        if(150 < angleX && angleX < 270){
            task = 19;
        }else{
            if(time_us_32() / 1000 < lastLineTime + 40000 / speed){
                NewLineTrace();
            }else{
                StraightLineTrace(90,speed);
                if(circleLineAngle < -999){
                    task = 19;
                }
            }
        }
    }else if(objectNumber < 12){//残りをすべて回収する
        MainMotorState(0,0);
        sleep_ms(250);
        MainMotorState(0,0);
        sleep_ms(250);
        gpio_put(buzzer_pin,0);

        int preTask;

        //壁あてしてジャイロのリセット
        {
        GetGyroAngleFromSub();
        RotationToAngle(180);
        firstTime = time_us_32();
        while(!gpio_get(touch_sensor_back_left_pin) || !gpio_get(touch_sensor_back_right_pin)){
            PrintDisplayMode();
            if(gpio_get(touch_sensor_back_left_pin) || gpio_get(touch_sensor_back_right_pin)){
                DaikeiKasoku(-50,999);
                if((time_us_32() - unWatchTime) / 1000 > 1000){
                    break;
                }  
            }else{
                DaikeiKasoku(-200,999);
                unWatchTime = time_us_32(); //関係ない変数
            }
            SendBufferToDisplay();
        }
        MainMotorState(0,0);
        sleep_ms(200);
        ResetGyro(180);
        DaikeiKasokuLoop(2000,200,180);
        }
        //ボール,缶の回収
        {
        redBallNumber = 0;
        blueBallNumber = 0;
        canNumber = 0;
        int preObjectNumber = objectNumber;
        int objID = 555;
        isCatchHorizonCan = false;
        while((objectTask == 1 || objectTask == 2 || (objectTask == 3 && (redBallNumber < 3 || allRedBallNumber == 6) && (canNumber < 2 || allCanNumber == 4))) && objectNumber < 12 && (time_us_32() - allFirstTime) / 1000000 < 510 && !isCatchHorizonCan){
            // uint8_t cmd = 0x11;
            // uart_write_blocking(camera_uart,&cmd,1);
            sleep_ms(1);
            objID = RotationToObject();
            if(objID == 999 || objID == 888){
                UseColorLED(255,0,255);
                firstTime = time_us_32();
                while((time_us_32() - firstTime) / 1000 < 2000){
                    PrintDisplayMode();
                    RotationToAngle(180);
                    DaikeiKasoku(200,180);
                    if(gpio_get(touch_sensor_front_left_pin) || gpio_get(touch_sensor_front_right_pin)){
                        //壁に当たった時は大きく下がって位置を調整する
                        DaikeiKasokuLoop(2500,-200,180);
                        RotationToAngle(270);
                        DaikeiKasokuLoop(750,200,270);
                        catchObjectDistance = 400000;
                        BackToLine();
                        RotationToAngle(180);
                        firstTime = 0;
                    }
                    SendBufferToDisplay();
                }
                MainMotorState(0,0);
                BackToLine();
                RotationToAngle(180);
                UseColorLED(0,0,0);
                continue;
            }
            if(objID == 3 || objID == 4){
                CatchBall(true);
            }else if(objID == 5 || objID == 6){
                CatchCan();
            }
            catchObjectDistance = 375000;
            UseColorLED(0,0,0);
            BackToLine();
            RotationToAngle(180);
            DaikeiKasokuLoop(1000,-200,180);
        }
        lineAngle = 180;
        preLineAngle = 180;
        if(redBallNumber == 0 && blueBallNumber == 0 && canNumber == 0 && !isCatchHorizonCan){
            DaikeiKasokuLoop(1500,200,180);
        }
        }  
        //壁あてしてジャイロのリセット
        {
        lastLineTime = time_us_32() / 1000 - 1000;
        preTask = task;
        while(task == preTask){
            BackLineTrace();
            if(angleX < 120){
                //1個読み違えてる
                while(task == preTask){
                    NewLineTrace();
                }
            }
        }
        gpio_put(buzzer_pin,0);
        MainMotorState(0,0);
        GetGyroAngleFromSub();
        RotationToAngle(180);
        firstTime = time_us_32();
        while(!gpio_get(touch_sensor_back_left_pin) || !gpio_get(touch_sensor_back_right_pin)){
            PrintDisplayMode();
            DaikeiKasoku(-400,999);
            SendBufferToDisplay();
        }
        MainMotorState(0,0);
        sleep_ms(200);
        ResetGyro(180);
        GetDataFromLineToMain();
        circleLineAngle = GetCircleLineVector(20,true,true);
        uint32_t nowTime = time_us_32() / 1000;
        firstTime = time_us_32();
        while(nowTime + 100 > time_us_32() / 1000){
            PrintDisplayMode();
            GetDataFromLineToMain();
            circleLineAngle = GetCircleLineVector(20,true,true);
            DaikeiKasoku(200,180);
            if((circleLineAngle < -999 || VectorAbsoluteValue > 0.4 || (30 < ((int)circleLineAngle % 180) && ((int)circleLineAngle % 180) < 150 && VectorNumber < 3) || (circleLineSensor[5] == 0 && circleLineSensor[15] == 0 && VectorNumber > 2))){
                nowTime = time_us_32() / 1000;
            }
            SendBufferToDisplay();
            if((time_us_32() - firstTime) / 1000 > 4000){
                isNotPlusTask = true;
                break;
            }
        }
        RotationToAngle(90);
        DaikeiKasokuLoop(1000,200,90);
        catchObjectDistance = 400000;
        BackToLine();
        RotationToAngle(180);
        MainMotorState(0,0);
        gpio_put(buzzer_pin,1);
        sleep_ms(500);
        gpio_put(buzzer_pin,0);
        }
        //ライントレースしつつ、ボールと缶の排出～初期地点で壁あて
        //初期位置からボールをとるところまでライントレース
        {
        lastLineTime = time_us_32() / 1000;
        if(isNotPlusTask) lastLineTime = 0;
        preTask = task;
        pLeft = 0;pRight = 0;
        while(task == preTask){
            BackLineTrace();
        }
        gpio_put(buzzer_pin,0);
        if(blueBallNumber > 0){
            MainMotorState(0,0);
            gpio_put(buzzer_pin,1);
            sleep_ms(500);
            gpio_put(buzzer_pin,0);
            TrashfromBasket(2);
            preLineAngle = 270;
            lineAngle = 270;
            sleep_ms(500);
            lastLineTime = time_us_32() / 1000;
            pLeft = 0;pRight = 0;
        }
        lastLineTime = time_us_32() / 1000;
        preTask = task;
        while(task == preTask){
            BackLineTrace();
        }
        gpio_put(buzzer_pin,0);
        if(isCatchHorizonCan){
            MainMotorState(0,0);
            gpio_put(buzzer_pin,1);
            sleep_ms(500);
            gpio_put(buzzer_pin,0);
            TrashHorizonCan();
            preLineAngle = 180;
            lineAngle = 180;
            sleep_ms(500);
            pLeft = 0;pRight = 0;
        }
        if(canNumber > 0){
            MainMotorState(0,0);
            gpio_put(buzzer_pin,1);
            sleep_ms(500);
            gpio_put(buzzer_pin,0);
            TrashfromBasket(3);
            preLineAngle = 180;
            lineAngle = 180;
            sleep_ms(500);
            lastLineTime = time_us_32() / 1000;
            pLeft = 0;pRight = 0;
        }
        if(redBallNumber == 0 && (time_us_32() - allFirstTime) / 1000000 < 480){
            //すぐに戻る
            preLineAngle = 0;
            lineAngle = 0;
            lastLineTime = time_us_32() / 1000;
            preTask = task;
            while(task <= preTask){
                NewLineTrace();
            }
            while(task <= preTask + 1){
                if(150 < angleX && angleX < 270){
                    task++;
                }else{
                    if(time_us_32() / 1000 < lastLineTime + 40000 / speed){
                        NewLineTrace();
                    }else{
                        StraightLineTrace(90,speed);
                        if(circleLineAngle < -999){
                            task++;
                        }
                    }
                }
            }
            if((time_us_32() - allFirstTime) / 1000000 > 510){
                objectNumber = 12;
            }
        }else{
            lastLineTime = time_us_32() / 1000;
            preTask = task;
            while(task == preTask){
                BackLineTrace();
            }
            gpio_put(buzzer_pin,0);
            if(redBallNumber > 0){
                MainMotorState(0,0);
                gpio_put(buzzer_pin,1);
                sleep_ms(500);
                gpio_put(buzzer_pin,0);
                TrashfromBasket(1);
            }
            RotationToAngle(0);
            OnWall(0);
            MainMotorState(0,0);
            ResetGyro(0);
            gpio_put(buzzer_pin,1);
            sleep_ms(500);
            gpio_put(buzzer_pin,0);
            preLineAngle = 0;
            lineAngle = 0;
            firstTime = time_us_32();   
            //初期位置からボールをとるところまでライントレース
            if(objectNumber < 12 && (time_us_32() - allFirstTime) / 1000000 < 465){
                lastLineTime = time_us_32() / 1000;
                preTask = task;
                while(task < preTask + 2){
                    do{
                        PrintDisplayMode();
                        GetDataFromLineToMain();
                        DaikeiKasoku(400,0);
                        if((time_us_32() - firstTime) / 1000 > 4000){
                            task = preTask + 1;
                            MainMotorState(0,0);
                            break;
                        }
                        SendBufferToDisplay();
                    }while(RightFrontCircleLine == 0 || LeftFrontCircleLine == 0);
                    do{
                        PrintDisplayMode();
                        GetDataFromLineToMain();
                        DaikeiKasoku(400,0);
                        if((time_us_32() - firstTime) / 1000 > 4000){
                            task = preTask + 1;
                            MainMotorState(0,0);
                            break;
                        }
                        SendBufferToDisplay();
                    }while(RightFrontCircleLine > 0 || LeftFrontCircleLine > 0);
                    gpio_put(buzzer_pin,1);
                    sleep_ms(200);
                    gpio_put(buzzer_pin,0);
                    task++;
                    if(task == preTask + 2){
                        // firstTime = time_us_32();
                        uint32_t firstTime2 = time_us_32();
                        while((time_us_32() - firstTime2) / 1000 < 750){
                            PrintDisplayMode();
                            StraightLineTrace(0,speed);
                            SendBufferToDisplay();
                            if((time_us_32() - firstTime) / 1000 > 4000){
                                break;
                            }
                        }
                    }
                }
                pLeft = speed * 10;pRight = speed * 10;
                while(task < preTask + 4){
                    NewLineTrace();
                }
                while(task < preTask + 5){
                    if(150 < angleX && angleX < 270){
                        task++;
                    }else{
                        if(time_us_32() / 1000 < lastLineTime + 40000 / speed){
                            NewLineTrace();
                        }else{
                            StraightLineTrace(90,speed);
                            if(circleLineAngle < -999){
                                task++;
                            }
                        }
                    }
                }
            }else{
                objectNumber = 12;
            }
        }
        }
    }else{
        //機体を停止させるためにブザーを鳴らし続ける
        MainMotorState(0,0);
        while(true){
            gpio_put(buzzer_pin,1);
            sleep_ms(300);
            gpio_put(buzzer_pin,0);
            sleep_ms(300);
        }
    }
}

//後ろ向きに進むライントレース
void BackLineTrace(){
    GetDataFromLineToMain();
    GetGyroAngleFromSub();
    circleLineAngle = GetCircleLineVector(20,true,true);
    if(VectorNumber == 4 || TjiAngle < 999){
        //十字の感知
        if(time_us_32() / 1000 > lastLineTime + 70000 / speed){
            gpio_put(buzzer_pin,1);
            isLineBuzzerOn = true;
            lastLineTime = time_us_32() / 1000;
            if(isNotPlusTask){
                isNotPlusTask = false;
            }else{
                task++;
                if(task < 4 && lineAngle == 90){
                    task = 4;
                }else if(16 < task && task < 18 && lineAngle == 90){
                    task = 18;
                }
            }
        }
    }
    if(270 < circleLineAngle && circleLineAngle < 360){
        //右に曲がる
        if((circleLineAngle - 270.0) * 2.0 - 12.5 > 12.5){
            PMove((int)(12.5 * speed),(int)(-12.5 * speed));
        }else{
            PMove((int)(((circleLineAngle - 270.0) * 2.0 - 12.5) * speed),(int)(-12.5 * speed));
        }
        // MainMotorState((int)(TurnSpeed * speed),-(int)(12.5 * speed));
    }else if(0 < circleLineAngle && circleLineAngle < 90){
        //左に曲がる
        if((90.0 - circleLineAngle) * 2.0 - 12.5 > 12.5){
            PMove((int)(-12.5 * speed),(int)(12.5 * speed));
        }else{
            PMove((int)(-12.5 * speed),(int)(((90.0 - circleLineAngle) * 2.0 - 12.5) * speed));
        }
        // MainMotorState(-(int)(12.5 * speed),(int)(TurnSpeed * speed));
    }else if(VectorAbsoluteValue > 0.4){
        if((265 < circleLineAngle && circleLineAngle < 270) || (105 < circleLineAngle && circleLineAngle < 180)){
            //右に曲がる
            if(265 < circleLineAngle && circleLineAngle < 270){
                PMove((int)(-9.0 * speed),(int)(-12.5 * speed));
            }else if(((circleLineAngle - 105.0)) - 12.5 > 12.5){
                PMove((int)(12.5 * speed),(int)(-12.5 * speed));
            }else{
                PMove((int)(((circleLineAngle - 105.0) - 12.5) * speed),(int)(-12.5 * speed));
            }
            // MainMotorState((int)(6.25 * speed),-(int)(6.25 * speed));
        }else if((180 < circleLineAngle && circleLineAngle < 255) || (90 < circleLineAngle && circleLineAngle < 95)){
            //左に曲がる
            if(90 < circleLineAngle && circleLineAngle < 95){
                PMove((int)(-12.5 * speed),(int)(-9.0 * speed));
            }else if((255.0 - circleLineAngle) - 12.5 > 12.5){
                PMove((int)(-12.5 * speed),(int)(12.5 * speed));
            }else{
                PMove((int)(-12.5 * speed),(int)((255.0 - circleLineAngle) - 12.5) * speed);
            }
            // MainMotorState(-(int)(6.25 * speed),(int)(6.25 * speed));
        }else{
            PMove(-(int)(10 * speed),-(int)(10 * speed));
            // MainMotorState(-(int)(10 * speed),-(int)(10 * speed));
        }
    }else{
        if(lineAngle == 0 && angleX > 180){
            PMove((int)((-10 - (angleX - 360) * 0.25) * speed),(int)((-10 + (angleX - 360) * 0.25) * speed));
            // MainMotorState((int)((-10 - (angleX - 360) * 0.25) * speed),(int)((-10 + (angleX - 360) * 0.25) * speed));
        }else{
            PMove((int)((-10 - (angleX - lineAngle) * 0.25) * speed),(int)((-10 + (angleX - lineAngle) * 0.25) * speed));
            // MainMotorState((int)((-10 - (angleX - lineAngle) * 0.25) * speed),(int)((-10 + (angleX - lineAngle) * 0.25) * speed));
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

//ボールや缶を取った後、線に復帰するプログラム
//最後に元の向きに回転しないので注意
void BackToLine(){
    GetGyroAngleFromSub();
    int backAngle;
    if(catchObjectDistance > 480000){
        DaikeiKasokuLoop(500,-100,180);
    }
    if(catchObjectDistance < 380000){
        //進まな過ぎて線を検知できないとき
        if(0 <= angleX && angleX < 180){
            //左側にいる
            RotationToAngle(180);
            MainMotorState(100,100);
            sleep_ms(3800u - catchObjectDistance / 100);
            RotationToAngle(90);
            backAngle = 90;
        }else {
            //右側にいる
            RotationToAngle(180);
            MainMotorState(100,100);
            sleep_ms(3800u - catchObjectDistance / 100);
            RotationToAngle(270);
            backAngle = 270;
        }
        DaikeiKasokuLoop(1000,150,backAngle);
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
        if(catchObjectDistance > 480000){
            DaikeiKasokuLoop(500,150,backAngle);
        }
    }
    sleep_ms(250);
    GetDataFromLineToMain();
    circleLineAngle = GetCircleLineVector(20,true,true);
    firstTime = time_us_32();
    uint32_t nowTime = time_us_32() / 1000;
    while(nowTime + 60 > time_us_32() / 1000){
        PrintDisplayMode();
        GetDataFromLineToMain();
        circleLineAngle = GetCircleLineVector(20,true,true);
        DaikeiKasoku(-300,backAngle);
        if(circleLineAngle < -999 || VectorAbsoluteValue > 0.4 || (30 < ((int)circleLineAngle % 180) && ((int)circleLineAngle % 180) < 150 && VectorNumber < 3) || (circleLineSensor[4] == 0 && circleLineSensor[16] == 0 && circleLineSensor[5] == 0 && circleLineSensor[15] == 0 && VectorNumber > 2)){
            nowTime = time_us_32() / 1000;
        }
        SendBufferToDisplay();
        if((time_us_32() - firstTime) / 1000 > 1000 && (gpio_get(touch_sensor_back_left_pin) || gpio_get(touch_sensor_back_right_pin))){
            while(nowTime + 100 > time_us_32() / 1000){
                PrintDisplayMode();
                GetDataFromLineToMain();
                circleLineAngle = GetCircleLineVector(20,true,true);
                DaikeiKasoku(300,backAngle);
                if(circleLineAngle < -999 || VectorAbsoluteValue > 0.4 || (30 < ((int)circleLineAngle % 180) && ((int)circleLineAngle % 180) < 150 && task < 3) || (circleLineSensor[4] == 0 && circleLineSensor[16] == 0 && circleLineSensor[5] == 0 && circleLineSensor[15] == 0 && task > 2)){
                    nowTime = time_us_32() / 1000;
                }
                SendBufferToDisplay();
            }
        }
    }
    MainMotorState(0,0);
    sleep_ms(250);
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

//赤ボールか青ボールを取る
//isSuction : 吸引をするか
void CatchBall(bool isSuction){
    //tofで探す
    SetServoAngleFromMain(servo_arm_up_and_down_pin,60);
    SetServoAngleFromMain(servo_arm_left_and_right_pin,90);
    uint32_t tofTime = time_us_32();
    SetStepperON();
    uint32_t prenow,now;
    prenow = time_us_32();
    int num_objects,d,number = 0;
    int preX = 0;
    unWatchTime = prenow;
    do{
        PrintDisplayMode();
        num_objects = UseCamera();
        d = 999;
        for(int i = 0;i < num_objects;i++){
            if(d > abs(cameraInformation[i].x - centerX) && 3 <= cameraInformation[i].obj_id && cameraInformation[i].obj_id <= 4){
                number = i;
                d = abs(cameraInformation[i].x - centerX);
            }
        }
        GetGyroAngleFromSub();
        now = time_us_32();
        if(d == 999 && (preX > centerX + 5 || preX < centerX - 5)){
            MainMotorState(50,50);
            prenow = now;
            GetDistanceFromSub();
            if(distance > 260){
                tofTime = time_us_32();
            }
            SendBufferToDisplay();
            if((now - unWatchTime) / 1000 > 3000){
                break;
            }
            continue;
        }else{
            preX = cameraInformation[number].x;
            unWatchTime = now;
        }
        
        int s;
        if(distance > 600){
            s = 300;
        }else if(distance < 300){
            s = 100;
        }else{
            s = 100 + (int)((distance - 300) / 1.5);
        }
        if(preX > centerX + 5){
            MainMotorState(s-10,s+10);
        }else if(preX < centerX - 5){
            MainMotorState(s+10,s-10);
        }else{
            MainMotorState(s,s);
        }
        GetDistanceFromSub();
        if(distance == 0xFFFF){
            MainMotorState(0,0);
        }else{
            if(!(d == 999 && (preX > centerX + 5 || preX < centerX - 5)))catchObjectDistance += s * sin((angleX - 90) / 180.0 * 3.1416) * (now - prenow) / 1000.0;
            prenow = now;
        }
        if(distance > 260){
            tofTime = time_us_32();
        }
        SendBufferToDisplay();
        if(gpio_get(touch_sensor_front_left_pin) || gpio_get(touch_sensor_front_right_pin)){
            break;
        }
    }while(tofTime + 100000 > time_us_32());
    MainMotorState(0,0);
    PrintDisplayMode();
    GetCurrentFromSub();
    SendBufferToDisplay();
    int upDownArmStandardCurrent = current[2];
    SetServoAngleFromMain(servo_arm_up_and_down_pin,162);
    SetServoAngleFromMain(servo_left_claw_pin,160);
    SetServoAngleFromMain(servo_right_claw_pin,20);
    sleep_ms(1000);
    SetServoAngleFromMain(servo_arm_up_and_down_pin,168);
    sleep_ms(500);
    PrintDisplayMode();
    GetCurrentFromSub();
    SendBufferToDisplay();
    if(current[2] < upDownArmStandardCurrent * 2){
        //アームが引っ掛かってる
        SetServoAngleFromMain(servo_arm_left_and_right_pin,90);
        SetServoAngleFromMain(servo_arm_up_and_down_pin,120);
        sleep_ms(500);
        GetGyroAngleFromSub();
        if(angleX > 180){
            RotationToAngle((int)(angleX - 7));
            SetServoAngleFromMain(servo_arm_left_and_right_pin,80);
        }else{
            RotationToAngle((int)(angleX + 7));
            SetServoAngleFromMain(servo_arm_left_and_right_pin,100);
        }
        SetServoAngleFromMain(servo_arm_up_and_down_pin,166);
        sleep_ms(500);
    }
    if(isSuction){
        SetSuctionMotorSpeedFromMain(150);
        sleep_ms(2000);
        catchObjectDistance += 500 * 100 * sin((angleX - 90) / 180.0 * 3.1416);
        DaikeiKasokuLoop(600,100,999);
    }else{
        catchObjectDistance += 250 * 100 * sin((angleX - 90) / 180.0 * 3.1416);
        DaikeiKasokuLoop(350,100,999);
        GetCurrentFromSub();
        int standardCurrent[2] = {current[0],current[1]};
        SetServoAngleFromMain(servo_left_claw_pin,50);
        SetServoAngleFromMain(servo_right_claw_pin,130);
        sleep_ms(600);
        int servoAngle = 50;
        while(servoAngle <= 160 && (current[0] < standardCurrent[0] * 2 || current[1] < standardCurrent[1] * 2)){
            PrintDisplayMode();
            servoAngle += 10;
            SetServoAngleFromMain(servo_left_claw_pin,servoAngle);
            SetServoAngleFromMain(servo_right_claw_pin,180 - servoAngle);
            sleep_ms(50);
            GetCurrentFromSub();
            SendBufferToDisplay();
        }
        if(servoAngle == 20) servoAngle += 10;
        SetServoAngleFromMain(servo_left_claw_pin,servoAngle - 20);
        SetServoAngleFromMain(servo_right_claw_pin,180 - (servoAngle - 20));
        if(servoAngle > 70) {
            //2つ同時に拾ってる可能性が高い
            SetSuctionMotorSpeedFromMain(150);
            sleep_ms(1000);
            SetServoAngleFromMain(servo_left_claw_pin,160);
            SetServoAngleFromMain(servo_right_claw_pin,20);
            sleep_ms(2000);
            DaikeiKasokuLoop(600,-200,999);
        }
    }
    TurnOnColorLEDFromMain();
    DaikeiKasokuLoop(250,-100,999);
    GetGyroAngleFromSub();
    catchObjectDistance += 150 * 100 * sin((angleX - 90) / 180.0 * 3.1416);
    sleep_ms(250);
    if(isSuction && allRedBallNumber <= 2 && redBallNumber == 0){
        //ピラミッドを崩すとき
        DaikeiKasokuLoop(5000,-25,999);
    }
    SetServoAngleFromMain(servo_arm_up_and_down_pin,90);
    if(!isSuction){
        sleep_ms(1000);
        SetSuctionMotorSpeedFromMain(150);
        sleep_ms(500);
        SetServoAngleFromMain(servo_left_claw_pin,160);
        SetServoAngleFromMain(servo_right_claw_pin,20);
        sleep_ms(1250);
    }
    SetServoAngleFromMain(servo_arm_up_and_down_pin,55);
    uint32_t colorTime = time_us_32();
    color = 0;
    while(true){
        GetColorFromSub();
        if(color == 1 || color == 3){
            if(objectTask == 1 && allRedBallNumber >= 3){
                SetServoAngleFromMain(servo_arm_left_and_right_pin,60);
            }else{
                SetServoAngleFromMain(servo_arm_left_and_right_pin,120);
            }
            allRedBallNumber++;
            redBallNumber++;
            break;
        }else if(color == 2){
            SetServoAngleFromMain(servo_arm_left_and_right_pin,60);
            allBlueBallNumber++;
            blueBallNumber++;
            break;
        }
        if(colorTime + 1000000 < time_us_32()){
            //タイムアウト
            SetSuctionMotorSpeedFromMain(0);
            sleep_ms(500);
            SetServoAngleFromMain(servo_arm_up_and_down_pin,55);
            sleep_ms(1000);
            return;
        }
        sleep_ms(10);
    }
    objectNumber++;
    sleep_ms(800);
    SetSuctionMotorSpeedFromMain(0);
    sleep_ms(2200);
    SetServoAngleFromMain(servo_arm_up_and_down_pin,40);
    sleep_ms(1000);
    SetServoAngleFromMain(servo_arm_left_and_right_pin,90);
    SetServoAngleFromMain(servo_arm_up_and_down_pin,50);
    sleep_ms(500);
}

//缶をとる
void CatchCan(){
    //tofで探す
    SetServoAngleFromMain(servo_arm_up_and_down_pin,60);
    SetServoAngleFromMain(servo_arm_left_and_right_pin,90);
    SetServoAngleFromMain(servo_left_claw_pin,160);
    SetServoAngleFromMain(servo_right_claw_pin,20);
    uint32_t tofTime = time_us_32();
    SetStepperON();
    int num_objects,d,number;
    uint32_t prenow,now;
    prenow = time_us_32();
    int preX = 0;
    bool isBreak = false;
    int canDistance = 115;//135
    unWatchTime = prenow;
    do{
        PrintDisplayMode();
        num_objects = UseCamera();
        d = 999;
        for(int i = 0;i < num_objects;i++){
            if(d > abs(cameraInformation[i].x - centerX) && 5 <= cameraInformation[i].obj_id && cameraInformation[i].obj_id <= 6){
                number = i;
                d = abs(cameraInformation[i].x - centerX);
            }
        }
        GetGyroAngleFromSub();
        now = time_us_32();
        if(d == 999 && (preX > centerX + 5 || preX < centerX - 5)){
            MainMotorState(50,50);
            prenow = now;
            GetDistanceFromSub();
            if(cameraInformation[number].y > canDistance){
                tofTime = time_us_32();
            }
            SendBufferToDisplay();
            if((now - unWatchTime) / 1000 > 3000){
                break;
            }
            continue;
        }else{
            preX = cameraInformation[number].x;
            unWatchTime = now;
        }
        int s;
        if((cameraInformation[number].y - canDistance) / 1.5 > 200){
            s = 300;
        }else if(cameraInformation[number].y < canDistance + 10){
            s = 100;
        }else{
            s = 100 + (int)((cameraInformation[number].y - canDistance) / 1.5);
        }
        if(preX > centerX + 5){
            MainMotorState(s-10,s+10);
        }else if(preX < centerX - 5){
            MainMotorState(s+10,s-10);
        }else{
            MainMotorState(s,s);
        }
        GetDistanceFromSub();
        if(distance == 0xFFFF){
            MainMotorState(0,0);
        }else{
            if(!(d == 999 && (preX > centerX + 5 || preX < centerX - 5)))catchObjectDistance += s * sin((angleX - 90) / 180.0 * 3.1416) * (now - prenow) / 1000.0;
            prenow = now;
        }
        if(cameraInformation[number].y > canDistance && distance > 310){
            tofTime = time_us_32();
        }
        SendBufferToDisplay();
        if(isBreak) break;
        if(gpio_get(touch_sensor_front_left_pin) || gpio_get(touch_sensor_front_right_pin)){
            break;
        }
    }while(tofTime + 100000 > time_us_32() || cameraInformation[number].y > canDistance);
    if(isBreak) return;
    if(cameraInformation[number].obj_id == 5){
        SetServoAngleFromMain(servo_left_claw_pin,110);
        SetServoAngleFromMain(servo_right_claw_pin,70);
    }else{
        SetServoAngleFromMain(servo_left_claw_pin,140);
        SetServoAngleFromMain(servo_right_claw_pin,40);
    }
    MainMotorState(0,0);
    SetServoAngleFromMain(servo_arm_up_and_down_pin,155);
    sleep_ms(1000);
    SetServoAngleFromMain(servo_arm_up_and_down_pin,168);
    sleep_ms(250);
    //少し前進する
    if(task < 7){
        catchObjectDistance += 1150 * 100 * sin((angleX - 90) / 180.0 * 3.1416);
        DaikeiKasokuLoop(1250,100,999); //1250 * 100
    }else{
        catchObjectDistance += 1650 * 100 * sin((angleX - 90) / 180.0 * 3.1416);
        DaikeiKasokuLoop(1750,100,999);
    }
    bool isGoto = false;
    CanCatch:
    SetServoAngleFromMain(servo_left_claw_pin,30);
    SetServoAngleFromMain(servo_right_claw_pin,150);
    sleep_ms(750);
    int servoAngle = 30;
    if(serialWatch == "oth") {
        snprintf(displayBuffer,displayBufferSize,"%d",servoAngle);
        WriteTextOnDisplay(10,10,displayBuffer,14,false,false);
    }
    GetCurrentFromSub();
    int standardCurrent[2] = {current[0],current[1]};
    while(servoAngle <= 160 && (current[0] > standardCurrent[0] * 0.8 && current[1] > standardCurrent[1] * 0.8)){
        PrintDisplayMode();
        servoAngle += 10;
        SetServoAngleFromMain(servo_left_claw_pin,servoAngle);
        SetServoAngleFromMain(servo_right_claw_pin,180 - servoAngle);
        sleep_ms(200);
        GetCurrentFromSub();
        if(serialWatch == "oth") {
            snprintf(displayBuffer,displayBufferSize,"%d",servoAngle);
            WriteTextOnDisplay(10,30,displayBuffer,10,false,false);
            snprintf(displayBuffer,displayBufferSize,"%u %d",current[0] ,standardCurrent[0]);
            WriteTextOnDisplay(10,45,displayBuffer,10,false,false);
            snprintf(displayBuffer,displayBufferSize,"%u %d",current[1] ,standardCurrent[1]);
            WriteTextOnDisplay(10,60,displayBuffer,10,false,false);
        }
        SendBufferToDisplay();
    }
    SetServoAngleFromMain(servo_left_claw_pin,20);
    SetServoAngleFromMain(servo_right_claw_pin,160);
    sleep_ms(250);
    MainMotorState(0,0);
    if(60 < servoAngle && servoAngle < 140 && task > 7){
        //缶(横缶)を拾えた
        if(!isGoto){
            DaikeiKasokuLoop(500,-200,999);
            isGoto = true;
            goto CanCatch;
        }
        isCatchHorizonCan = true;
        allCanNumber++;
        objectNumber++;
        sleep_ms(1000);
        SetServoAngleFromMain(servo_arm_up_and_down_pin,90);
        SetServoAngleFromMain(servo_arm_left_and_right_pin,90);
        sleep_ms(500);
    }else if(servoAngle < 70){
        //縦缶を拾えた
        canNumber++;
        allCanNumber++;
        objectNumber++;
        sleep_ms(500);
        SetServoAngleFromMain(servo_arm_up_and_down_pin,50);
        sleep_ms(1500);
        SetServoAngleFromMain(servo_left_claw_pin,160);
        SetServoAngleFromMain(servo_right_claw_pin,20);
        sleep_ms(1000);
        //初めて缶をとった時は振る
        if(canNumber == 1){
            SetServoAngleFromMain(servo_arm_up_and_down_pin,90);
            sleep_ms(500);
            SetServoAngleFromMain(servo_center_basket_pin,150);
            sleep_ms(200);
            SetServoAngleFromMain(servo_center_basket_pin,140);
            sleep_ms(200);
            SetServoAngleFromMain(servo_center_basket_pin,130);
            sleep_ms(200);
            SetServoAngleFromMain(servo_center_basket_pin,120);
            sleep_ms(200);
            SetServoAngleFromMain(servo_center_basket_pin,110);
            sleep_ms(500);
            SetServoAngleFromMain(servo_center_basket_pin,160);
            sleep_ms(500);
            SetServoOffFromMain(servo_center_basket_pin);
            sleep_ms(10);
            SetServoAngleFromMain(servo_arm_up_and_down_pin,50);
        }
    }else{
        if(!isGoto && task > 7){
            DaikeiKasokuLoop(500,-200,999);
            isGoto = true;
            goto CanCatch;
        }
        SetServoAngleFromMain(servo_left_claw_pin,160);
        SetServoAngleFromMain(servo_right_claw_pin,20);
        sleep_ms(200);
        DaikeiKasokuLoop(750,-200,999);
        catchObjectDistance -= 50000;
        sleep_ms(500);
        SetServoAngleFromMain(servo_arm_up_and_down_pin,50);
        SetServoAngleFromMain(servo_arm_left_and_right_pin,90);
        sleep_ms(500);
    }
}

//ペットボトルを取る
void CatchPetBottle(){
    sleep_ms(100);
    GetDataFromLineToMain();
    circleLineAngle = GetCircleLineVector(20,true,true);
    UseColorLED(255,255,255);
    if(objectNumber < 5){
        firstTime = time_us_32();
        while(VectorNumber < 4){
            PrintDisplayMode();
            GetDataFromLineToMain();
            circleLineAngle = GetCircleLineVector(20,true,true);
            DaikeiKasoku(100,180);
            SendBufferToDisplay();
        }
        firstTime = time_us_32();
        UseColorLED(255,0,255);
        while((time_us_32() - firstTime) / 1000 < 1000){
            PrintDisplayMode();
            GetGyroAngleFromSub();
            GetDataFromLineToMain();
            circleLineAngle = GetCircleLineVector(20,true,true);
            MainMotorState(100 - (int)((angleX - 180) * 10),100 + (int)((angleX - 180) * 10));
            SendBufferToDisplay();
        }
        UseColorLED(0,255,255);
    }
    UseColorLED(255,0,255);
    GetDistanceFromSub();
    uint32_t deltaTime = 0;
    uint32_t preTime = time_us_32() / 1000;
    firstTime = time_us_32();
    if(distance < 350){
        while(deltaTime < 50){
            PrintDisplayMode();
            DaikeiKasoku(-100,999);
            GetDistanceFromSub();
            if(distance > 360){
                deltaTime += time_us_32() / 1000 - preTime;
            }else{
                deltaTime = 0;
            }
            preTime = time_us_32() / 1000;
            SendBufferToDisplay();
        }
    }
    deltaTime = 0;
    preTime = time_us_32() / 1000;
    firstTime = time_us_32();
    pLeft = 100;pRight = 100;
    while(deltaTime < 50){
        PrintDisplayMode();
        StraightLineTrace(180,10);
        GetDistanceFromSub();
        if(distance < 350){
            deltaTime += time_us_32() / 1000 - preTime;
        }else{
            deltaTime = 0;
        }
        preTime = time_us_32() / 1000;
        SendBufferToDisplay();
        if((time_us_32() - firstTime) / 1000 > 5000){
            DaikeiKasokuLoop(1250,-150,180);
            break;
        } 
    }
    MainMotorState(0,0);
    SetServoAngleFromMain(servo_arm_left_and_right_pin,90);
    SetServoAngleFromMain(servo_left_claw_pin,90);
    SetServoAngleFromMain(servo_right_claw_pin,90);
    SetServoAngleFromMain(servo_arm_up_and_down_pin,90);
    sleep_ms(500);
    for(int i = 100;i <= 160;i+=10){
        SetServoAngleFromMain(servo_arm_up_and_down_pin,i);
        sleep_ms(250);
    }
    SetServoAngleFromMain(servo_arm_up_and_down_pin,163);
    sleep_ms(1000);
    DaikeiKasokuLoop(1300,100,180);
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
    SetServoAngleFromMain(servo_arm_up_and_down_pin,60);
    //指定の場所まで移動
    RotationToAngle(270);
    sleep_ms(500);
    firstTime = time_us_32();
    while(!gpio_get(touch_sensor_front_left_pin) && !gpio_get(touch_sensor_front_right_pin)){
        PrintDisplayMode();
        DaikeiKasoku(200,270);
        SendBufferToDisplay();
    }
    for(int i = 90; i <= 160;i += 10){
        MainMotorState(-10,-10);
        sleep_ms(200);
        SetServoAngleFromMain(servo_arm_up_and_down_pin,i);
    }
    MainMotorState(0,0);
    sleep_ms(500);
    SetServoAngleFromMain(servo_left_claw_pin,70);
    SetServoAngleFromMain(servo_right_claw_pin,110);
    sleep_ms(1000);
    MainMotorState(-200,-200);
    sleep_ms(100);
    MainMotorState(0,0);
    SetServoAngleFromMain(servo_left_claw_pin,90);
    SetServoAngleFromMain(servo_right_claw_pin,90);
    sleep_ms(1000);
    SetServoAngleFromMain(servo_arm_up_and_down_pin,50);
    sleep_ms(500);
    SetServoAngleFromMain(servo_arm_left_and_right_pin,90);
    sleep_ms(1000);
    catchObjectDistance = 400000;
}

//台形加速のwhile文の中身
//関数内でGetGyroAngleFromSub();を使用。
//ディスプレイの表示は外部で必要
//speed : 最高速度(後ろに下がる場合は負の値)
//angle : 常に向きたい角度,999で角度補正をなくす
void DaikeiKasoku(int speed,int angle){
    if(angle == 999){
        if((time_us_32() - firstTime) / 2000 > abs(speed)){
            MainMotorState(speed,speed);
        }else{
            if(speed > 0){
                MainMotorState((int)(time_us_32() - firstTime) / 2000,(int)(time_us_32() - firstTime) / 2000);
            }else{
                MainMotorState((int)(time_us_32() - firstTime) / -2000,(int)(time_us_32() - firstTime) / -2000);
            }  
        }
    }else{
        GetGyroAngleFromSub();
        if(angle < 10 && angleX > 180) angleX -= 360;
        if((time_us_32() - firstTime) / 2000 > abs(speed)){
            MainMotorState(speed - (int)((angleX-angle) * 10),speed + (int)((angleX-angle) * 10));
        }else{
            if(speed > 0){
                MainMotorState((int)(time_us_32() - firstTime) / 2000 - (int)((angleX-angle) * 10),(int)(time_us_32() - firstTime) / 2000 + (int)((angleX-angle) * 10));
            }else{
                MainMotorState((int)(time_us_32() - firstTime) / -2000 - (int)((angleX-angle) * 10),(int)(time_us_32() - firstTime) / -2000 + (int)((angleX-angle) * 10));
            }  
        }
    }
}

//while文で台形加速で進む
//time : ミリ秒での進む時間
//speed : 最高速度(後ろに下がる場合は負の値)
//angle : 常に向きたい角度,999で角度補正をなくす
//最後に速度を0にする
void DaikeiKasokuLoop(int time,int speed,int angle){
    firstTime = time_us_32();
    while((time_us_32() - firstTime) / 1000 < time){
        PrintDisplayMode();
        DaikeiKasoku(speed,angle);
        SendBufferToDisplay();
    }
    MainMotorState(0,0);
}

//円形ラインセンサを使ったライントレース
void NewLineTrace(){
    GetDataFromLineToMain();
    GetGyroAngleFromSub();
    circleLineAngle = GetCircleLineVector(20,true,true);
    if(VectorNumber == 4 || (TjiAngle < 999 && task > 3)){
        //十字の感知
        if(time_us_32() / 1000 > lastLineTime + 70000 / speed){
            gpio_put(buzzer_pin,1);
            isLineBuzzerOn = true;
            lastLineTime = time_us_32() / 1000;
            if(isNotPlusTask){
                isNotPlusTask = false;
            }else{
                task++;
                if(task < 4 && lineAngle == 90){
                    task = 4;
                }else if(16 < task && task < 18 && lineAngle == 90){
                    task = 18;
                }
            }
            
        }
    }
    if(90 < circleLineAngle && circleLineAngle < 180){
        //右に曲がる
        if(12.5 - (circleLineAngle - 90.0) * 2.0 < -12.5){
            PMove((int)(12.5 * speed),(int)(-12.5 * speed));
        }else{
            PMove((int)(12.5 * speed),(int)((12.5 - (circleLineAngle - 90.0) * 2.0) * speed));
        }
        // PMove((int)(12.5 * speed),-(int)(TurnSpeed * speed));
        // MainMotorState((int)(12.5 * speed),-(int)(TurnSpeed * speed));
    }else if(180 < circleLineAngle && circleLineAngle < 270){
        //左に曲がる
        if(12.5 - (270.0 - circleLineAngle) * 2.0 < -12.5){
            PMove((int)(-12.5 * speed),(int)(12.5 * speed));
        }else{
            PMove((int)((12.5 - (270.0 - circleLineAngle) * 2.0) * speed),(int)(12.5 * speed));
        }
        // PMove(-(int)(TurnSpeed * speed),(int)(12.5 * speed));
        // MainMotorState(-(int)(TurnSpeed * speed),(int)(12.5 * speed)); //理論値は-2.21
    }else if(VectorAbsoluteValue > 0.4){
        //個々の角度設定が90 - x の関係で間違っている可能性
        if((85 < circleLineAngle && circleLineAngle < 90) || 285 < circleLineAngle ){
            //右に曲がる
            if(85 < circleLineAngle && circleLineAngle < 90){
                PMove((int)(12.5 * speed),(int)(9.0 * speed));
            }else if(12.5 - (circleLineAngle - 285.0) < -12.5){
                PMove((int)(12.5 * speed),(int)(-12.5 * speed));
            }else{
                PMove((int)(12.5 * speed),(int)((12.5 - (circleLineAngle - 285.0)) * speed));
            }
            // MainMotorState((int)(6.25 * speed),-(int)(6.25 * speed));
        }else if(circleLineAngle < 75 || (270 < circleLineAngle && circleLineAngle < 275)){
            //左に曲がる
            if(270 < circleLineAngle && circleLineAngle < 275){
                PMove((int)(9.0 * speed),(int)(12.5 * speed));
            }else if(12.5 - (75 - circleLineAngle) < -12.5){
                PMove((int)(-12.5 * speed),(int)(12.5 * speed));
            }else{
                PMove((int)((12.5 - (75.0 - circleLineAngle)) * speed),(int)(12.5 * speed));
            }
            // PMove(-(int)(TurnSpeed * speed),(int)(12.5 * speed));
            // MainMotorState(-(int)(6.25 * speed),(int)(6.25 * speed));
        }else{
            PMove((int)(10 * speed),(int)(10 * speed));
            // MainMotorState((int)(10 * speed),(int)(10 * speed));
        }
    }else{
        if(lineAngle == 0 && angleX > 180){
            PMove((int)((10 - (angleX - 360) * 0.25) * speed),(int)((10 + (angleX - 360) * 0.25) * speed));
            // MainMotorState((int)((10 - (angleX - 360) * 0.25) * speed),(int)((10 + (angleX - 360) * 0.25) * speed));
        }else{
            PMove((int)((10 - (angleX - lineAngle) * 0.25) * speed),(int)((10 + (angleX - lineAngle) * 0.25) * speed));
            // MainMotorState((int)((10 - (angleX - lineAngle) * 0.25) * speed),(int)((10 + (angleX - lineAngle) * 0.25) * speed));
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

//カメラを使う位置に戻る。線上にいる前提
void OnWall(int angle){
    firstTime = time_us_32();
    while(!gpio_get(touch_sensor_back_left_pin) || !gpio_get(touch_sensor_back_right_pin)){
        PrintDisplayMode();
        if(gpio_get(touch_sensor_back_left_pin) || gpio_get(touch_sensor_back_right_pin)){
            DaikeiKasoku(-50,angle);
            if((time_us_32() - unWatchTime) / 1000 > 1000){
                break;
            }
        }else{
            DaikeiKasoku(-400,angle);
            unWatchTime = time_us_32(); //関係ない変数
        }
        SendBufferToDisplay();
    }
    MainMotorState(0,0);
}

//島と壁の間を抜ける処理
void PassTheSpace(){
    DaikeiKasokuLoop(600,-200,0);
    MainMotorState(0,0);
    sleep_ms(100);
    RotationToAngle(90);
    sleep_ms(250);
    DaikeiKasokuLoop(1300,400,90);
    firstTime = time_us_32();
    while((time_us_32() - firstTime) / 2000 < 400){
        PrintDisplayMode();
        GetDataFromLineToMain();
        GetGyroAngleFromSub();
        
        MainMotorState(400 - (int)(time_us_32() - firstTime) / 2000 - (int)((angleX - 90) * 10),400 - (int)(time_us_32() - firstTime) / 2000 + (int)((angleX - 90) * 10));

        SendBufferToDisplay();
    }
    MainMotorState(0,0);
    // RotationToAngle(0);
    // OnWall(0);
    // RotationToAngle(90);
}

#define gain 0.09
//P制御で動かす(主にライントレース用)
//left : 左モーターの目標速度
//right : 右モーターの目標速度
void PMove(int left,int right){
    if(time_us_32() - pPreTime > 500000){
        //0.5s以上離れているときは初めて使った時と判定してpPreTimeをリセット
        pPreTime = time_us_32();
    }
    uint32_t now = time_us_32();
    float k;
    if(now == pPreTime){
        k = 0;
    }else{
        k = 1.0f - powf(0.5f, ((now - pPreTime) / 1000000.0f) / gain);
    }
    pLeft += ((float)left - pLeft) * k;
    pRight += ((float)right - pRight) * k;
    MainMotorState((int)pLeft,(int)pRight);
    pPreTime = now;
}

//指定の角度に向く
void RotationToAngle(int target_angle){
    SetStepperON();
    //目標角度を0度から360度の間に補正
    target_angle = target_angle % 360;
    if (target_angle < 0) {
        target_angle += 360;
    }
    firstTime = time_us_32();
    uint32_t lastCheckTime = time_us_32();
    float lastRotation_angle = 999;
    while(true){
        PrintDisplayMode();
        GetGyroAngleFromSub();
        //rotation_angleは回転角
        float rotation_angle = target_angle - angleX;
        if(rotation_angle < 0){
            rotation_angle += 360; 
        }
        if(lastCheckTime == 0 || (time_us_32() - lastCheckTime) / 1000 > 1500){
            if(fabs(lastRotation_angle - rotation_angle) < 1.5){
                DaikeiKasokuLoop(750,-200,999);
            }
            lastRotation_angle = rotation_angle;
            lastCheckTime = time_us_32();
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

//もっとも手前の物体のほうを向く
//0°と360°の境目で使用禁止
//objectIDを返す。888→タイムアウト,999→対象がない
//isHorizonCan : 横缶のみを探すかどうか
int RotationToObject(){
    firstTime = time_us_32();
    int isGreaterThancenterX; // 0が初期状態。1が大きい。2が小さい。
    int decidedObj_id;
    int Y,number;
    int rotationTime;
    int firstRotationTime;
    int retireNumber;
    Find:
    isGreaterThancenterX = 0;
    decidedObj_id = 0;
    rotationTime = time_us_32();
    firstRotationTime = rotationTime;
    retireNumber = 0;
    while((time_us_32() - rotationTime) / 1000 < 500 || retireNumber != 0){
        PrintDisplayMode();
        int num_objects = 0;
        num_objects = UseCamera();
        while(num_objects > 100){
            num_objects = UseCamera();
            UseColorLED(255,0,255);   
        }//100より大きい数はエラー信号
        // UseColorLED(0,0,0);
        Y = 999,number = 0;
        for(int i = 0;i < num_objects;i++){
            if(Y > cameraInformation[i].y){
                if((rotationTime - firstRotationTime) / 1000 < 3000 || (rotationTime - firstRotationTime) / 1000 > 6000 || (3 <= cameraInformation[i].obj_id && cameraInformation[i].obj_id <= 4)){
                    if((cameraInformation[i].obj_id == 3 && allRedBallNumber < 6 && !(objectTask == 2 && allRedBallNumber >= 5 && allCanNumber <= 2)) || (cameraInformation[i].obj_id == 4 && allBlueBallNumber < 2) || ((cameraInformation[i].obj_id == 5 || cameraInformation[i].obj_id == 6) && allCanNumber < 4 && !(objectTask == 2 && allRedBallNumber <= 3))){
                        number = i;
                        Y = cameraInformation[i].y;
                    }
                } 
            }
        }
        if(num_objects == 0 || (Y == 999 && isCatchHorizonCan)){
            retireNumber++;
            if(retireNumber > 50){
                return 999;
            }
        }else{
            retireNumber = 0;
        }
        if(num_objects == 0 || Y == 999){
            rotationTime = time_us_32();
            SendBufferToDisplay();
            continue;
        }
        if(decidedObj_id != cameraInformation[number].obj_id && !(decidedObj_id == 5 && cameraInformation[number].obj_id == 6) && !(decidedObj_id == 6 && cameraInformation[number].obj_id == 5)){
            decidedObj_id = cameraInformation[number].obj_id;
            if(decidedObj_id == 3){
                UseColorLED(255,0,0);
            }else if(decidedObj_id == 4){
                UseColorLED(0,0,255);
            }else{
                UseColorLED(255,255,0);
            }
            rotationTime = time_us_32();
        }
        if((rotationTime - firstRotationTime) / 1000 > 9000 || ((rotationTime - firstRotationTime) / 1000 > 4500 && num_objects == 1)){
            // for(int i = 0;i < num_objects;i++){
                // if((cameraInformation[i].obj_id == 3 && allRedBallNumber <= 6) || (cameraInformation[i].obj_id == 4 && allBlueBallNumber <= 2) || ((cameraInformation[i].obj_id == 5 || cameraInformation[i].obj_id == 6)) && allCanNumber <= 4){
                    // number = i;
                    // decidedObj_id = cameraInformation[number].obj_id;
                    // if(decidedObj_id == 3){
                        // UseColorLED(255,0,0);
                    // }else if(decidedObj_id == 4){
                        // UseColorLED(0,0,255);
                    // }else{
                        // UseColorLED(255,255,0);
                    // }
                    // rotationTime = time_us_32() - 10000000;//強制的にループから抜けさせる
                // }

            // }
            return 888;
        } 
        SendBufferToDisplay();
    }
    if(cameraInformation[number].x > centerX){
        isGreaterThancenterX = 1;
    }else{
        isGreaterThancenterX = 2;
    }
    int gotoNumber = 0;
    GetGyroAngleFromSub();
    float targetAngle = angleX - atan((float)(cameraInformation[number].x - centerX) / (cameraInformation[number].y + 40.0)) / 3.142 * 180;
    
    firstTime = time_us_32();
    while(true){
        PrintDisplayMode();
        int num_objects;
        num_objects = UseCamera();
        while(num_objects > 100){//100より大きい数はエラー信号
            MainMotorState(0,0);
            num_objects = UseCamera();
        }
        GetGyroAngleFromSub();
        Y = 999,number = 0;
        // カメラは反転しているのでYが小さいほど手前にある
        for(int i = 0;i < num_objects;i++){
            if(Y > cameraInformation[i].y && 3 <= cameraInformation[i].obj_id && cameraInformation[i].obj_id <= 6){
                if(((isGreaterThancenterX == 1 && centerX - 5 < cameraInformation[i].x  && (cameraInformation[i].x < 320.0 - (180 - angleX) * 3.0 || decidedObj_id == 4)) || (isGreaterThancenterX == 2 && ( (angleX - 180.0) * 3.0 < cameraInformation[i].x || decidedObj_id == 4) && cameraInformation[i].x < centerX + 5)) && (cameraInformation[i].obj_id == decidedObj_id || (cameraInformation[i].obj_id == 5 && decidedObj_id == 6) || (cameraInformation[i].obj_id == 6 && decidedObj_id == 5))){
                    if(fabs(targetAngle - (angleX - atan((float)(cameraInformation[i].x - centerX) / (cameraInformation[i].y + 40.0)) / 3.142 * 180)) < 10.0){
                        number = i;
                        Y = cameraInformation[i].y;
                    }
                }
            }
        }
        if(Y == 999 || num_objects == 0){
            MainMotorState(0,0);
            SendBufferToDisplay();
            gotoNumber++;
            if(gotoNumber > 50) goto Find;
            continue;
        }
        //カメラの画像は上下左右反転しているので回転方向が本来と逆なことに注意
        if(cameraInformation[number].x > centerX){
            //左側にある　→　反時計回りに回転
            if(angleX < 135 && (cameraInformation[number].obj_id == 5 || cameraInformation[number].obj_id == 6)){
                MainMotorState(0,0);
            }else if(gpio_get(touch_sensor_back_left_pin)){
                //左壁に当たっている
                if((cameraInformation[number].x - centerX) * 2 > 400){
                    MainMotorState(0,400);
                }else if((cameraInformation[number].x - centerX) * 2 < 25){
                    MainMotorState(0,25);
                }else{
                    MainMotorState(0,(cameraInformation[number].x - centerX) * 2);
                }
            }else{
                //左壁に当たってない
                if((cameraInformation[number].x - centerX) * 2 > 400){
                    MainMotorState(-400,400);
                }else if((cameraInformation[number].x - centerX) * 2 < 25){
                    MainMotorState(-25,25);
                }else{
                    MainMotorState((cameraInformation[number].x - centerX) * -2,(cameraInformation[number].x - centerX) * 2);
                }
            }
        }else{
            //右側にある　→　時計回りに回転
            if(angleX > 225 && (cameraInformation[number].obj_id == 5 || cameraInformation[number].obj_id == 6)){
                MainMotorState(0,0);
            }else if(gpio_get(touch_sensor_back_right_pin)){
                //右壁に当たっている
                if((centerX - cameraInformation[number].x) * 2 > 400){
                    MainMotorState(400,0);
                }else if((centerX - cameraInformation[number].x) * 2 < 25){
                    MainMotorState(25,0);
                }else{
                    MainMotorState((centerX - cameraInformation[number].x) * 2,0);
                }
            }else{
                //右壁に当たってない
                if((centerX - cameraInformation[number].x) * 2 > 400){
                    MainMotorState(400,-400);
                }else if((centerX - cameraInformation[number].x) * 2 < 25){
                    MainMotorState(25,-25);
                }else{
                    MainMotorState((centerX - cameraInformation[number].x) * 2,(centerX - cameraInformation[number].x) * -2);
                }
            }
        }
        if(centerX - 3 < cameraInformation[number].x && cameraInformation[number].x < centerX + 3){
            MainMotorState(0,0);
            return cameraInformation[number].obj_id;
        }
        SendBufferToDisplay();
        if((time_us_32() - firstTime) / 1000 > 5000){
            //タイムアウト
            goto Find;
        }
    }
}

//線に沿って前進する
//angle : ジャイロで常に向いているべき角度
//speed2 : スピード(×10される)
void StraightLineTrace(int angle,int speed2){
    GetDataFromLineToMain();
    GetGyroAngleFromSub();
    circleLineAngle = GetCircleLineVector(20,true,true);
    if(90 < circleLineAngle && circleLineAngle < 180 && (angleX - angle < -5 && angle != 0 || (180 < angleX && angleX < 355 && angle == 0))){
        //右に曲がる
        if(12.5 - (circleLineAngle - 90.0) * 2.0 < -12.5){
            PMove((int)(12.5 * speed),(int)(-12.5 * speed));
        }else{
            PMove((int)(12.5 * speed),(int)((12.5 - (circleLineAngle - 90.0) * 2.0) * speed));
        }
    }else if(180 < circleLineAngle && circleLineAngle < 270 && angleX - angle > 5){
        //左に曲がる
        if(12.5 - (270.0 - circleLineAngle) * 2.0 < -12.5){
            PMove((int)(-12.5 * speed),(int)(12.5 * speed));
        }else{
            PMove((int)((12.5 - (270.0 - circleLineAngle) * 2.0) * speed),(int)(12.5 * speed));
        }
    }else if(VectorAbsoluteValue > 0.4){
        if((85 < circleLineAngle && circleLineAngle < 90) || 285 < circleLineAngle ){
            //右に曲がる
            if(85 < circleLineAngle && circleLineAngle < 90){
                PMove((int)(12.5 * speed),(int)(9.0 * speed));
            }else if(12.5 - (circleLineAngle - 285.0) < -12.5){
                PMove((int)(12.5 * speed),(int)(-12.5 * speed));
            }else{
                PMove((int)(12.5 * speed),(int)((12.5 - (circleLineAngle - 285.0)) * speed));
            }
        }else if(circleLineAngle < 75 || (270 < circleLineAngle && circleLineAngle < 275)){
            //左に曲がる
            if(270 < circleLineAngle && circleLineAngle < 275){
                PMove((int)(9.0 * speed),(int)(12.5 * speed));
            }else if(12.5 - (75 - circleLineAngle) < -12.5){
                PMove((int)(-12.5 * speed),(int)(12.5 * speed));
            }else{
                PMove((int)((12.5 - (75.0 - circleLineAngle)) * speed),(int)(12.5 * speed));
            }
        }else{
            PMove((int)(10 * speed2),(int)(10 * speed2));
            // MainMotorState((int)(10 * speed2),(int)(10 * speed2));
        }
    }else{
        if(angle == 0 && angleX > 180){
            PMove((int)((10 - (angleX - 360) * 0.25) * speed2),(int)((10 + (angleX - 360) * 0.25) * speed2));
            // MainMotorState((int)((10 - (angleX - 360) * 0.25) * speed2),(int)((10 + (angleX - 360) * 0.25) * speed2));
        }else{
            PMove((int)((10 - (angleX - angle) * 0.25) * speed2),(int)((10 + (angleX - angle) * 0.25) * speed2));
            // MainMotorState((int)((10 - (angleX - angle) * 0.25) * speed2),(int)((10 + (angleX - angle) * 0.25) * speed2));
        }
    }
}

//ボールや缶を排出する
//objectは赤ボールが1,青ボールが2,缶が3
//赤ボールの時だけ終了時が十字条件を満たさない
void TrashfromBasket(int object){
    RotationToAngle(90);
    GetDataFromLineToMain();
    circleLineAngle = GetCircleLineVector(20,true,true);
    firstTime = time_us_32();
    //ゴールまで下がる
    while((((!circleLineSensor[9] && !circleLineSensor[10] && !circleLineSensor[11]) && object != 2) || ((!circleLineSensor[7] || !circleLineSensor[12]) && object == 2)) || (time_us_32() - firstTime) / 1000 < 1250){
        PrintDisplayMode();
        GetDataFromLineToMain();
        circleLineAngle = GetCircleLineVector(20,true,true);
        DaikeiKasoku(-300,90);
        SendBufferToDisplay();
    }
    MainMotorState(0,0);
    if(object == 1){
        if(objectTask == 1 && allRedBallNumber >= 3){
            TrashfromBasketFromMain(11);
        }else{
            TrashfromBasketFromMain(10);
        }
    }else{
        TrashfromBasketFromMain(object);
    }
    
    GetDataFromLineToMain();
    circleLineAngle = GetCircleLineVector(20,true,true);
    firstTime = time_us_32();
    if(object == 1){
        while(circleLineSensor[5] + circleLineSensor[6] == 0  || (time_us_32() - firstTime) / 1000 < 750){
            PrintDisplayMode();
            GetDataFromLineToMain();
            circleLineAngle = GetCircleLineVector(20,true,true);
            DaikeiKasoku(300,90);
            SendBufferToDisplay();
        }
        MainMotorState(0,0);
        sleep_ms(250);
        RotationToAngle(0);
    }else{
        while((VectorNumber != 4 && !circleLineSensor[5]) || (time_us_32() - firstTime) / 1000 < 750){
            PrintDisplayMode();
            GetDataFromLineToMain();
            circleLineAngle = GetCircleLineVector(20,true,true);
            DaikeiKasoku(300,90);
            SendBufferToDisplay();
        }
        MainMotorState(0,0);
        sleep_ms(250);
        if(object == 2){
            RotationToAngle(90);
        }else{
            RotationToAngle(0);
        }
        sleep_ms(250);
    }
}

//横缶を排出する
void TrashHorizonCan(){
    RotationToAngle(270);
    GetDataFromLineToMain();
    MainMotorState(0,0);

    //捨てる
    SetServoAngleFromMain(servo_arm_left_and_right_pin,90);
    SetServoAngleFromMain(servo_arm_up_and_down_pin,160);
    sleep_ms(500);
    SetServoAngleFromMain(servo_left_claw_pin,160);
    SetServoAngleFromMain(servo_right_claw_pin,20);
    sleep_ms(1000);
    SetServoAngleFromMain(servo_arm_up_and_down_pin,50);
    sleep_ms(1000);
    RotationToAngle(0);
}

//カメラ、ラインセンサ、ジャイロ、tof、カラーセンサ、電流センサを使う
void UseAllSensor(){
    UseCamera();
    GetDataFromLineToMain();
    GetGyroAngleFromSub();
    GetDistanceFromSub();
    GetColorFromSub();
    GetCurrentFromSub();
}

//円形ラインセンサのベクトルの和の向きを計算する
//circleLineAngleに代入して使う
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