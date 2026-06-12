#include "display.hpp"
#include "gyro.hpp"
#include "main_to_sub.hpp"
#include "motor.hpp"
#include "other_sensor.hpp"
#include "others.hpp"
#include "servo.hpp"
#include "tof.hpp"
#include "../config.hpp"
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include <stdio.h>

#define BAUDRATE 19200

bool isGyroReset;

void MainToSubSetup(){
    gpio_init(main_to_sub_RX_pin);
    gpio_init(main_to_sub_TX_pin);
    gpio_set_function(main_to_sub_RX_pin,GPIO_FUNC_UART);
    gpio_set_function(main_to_sub_TX_pin,GPIO_FUNC_UART);
    uart_init(main_to_sub_uart,BAUDRATE);
    correctionAngle = 0;
}

void SubToMainSetup(){
    isGyroReset = false;
    gpio_init(sub_to_main_RX_pin);
    gpio_init(sub_to_main_TX_pin);
    gpio_set_function(sub_to_main_RX_pin,GPIO_FUNC_UART);
    gpio_set_function(sub_to_main_TX_pin,GPIO_FUNC_UART);
    uart_init(sub_to_main_uart,BAUDRATE);
    uart_set_irq_enables(sub_to_main_uart,true,false);
    irq_set_exclusive_handler(UART0_IRQ,SubCallBack);
    irq_set_enabled(UART0_IRQ,true);   
}

void SubCallBack(){
    printf("割り込みされた\n");
    while(uart_is_readable(sub_to_main_uart)){
        uint8_t data = (uint8_t)uart_getc(sub_to_main_uart);
        switch (data)
        {
        case 0x01:
            //AngleXの取得
            UseGyroSensor();
            uart_write_blocking(sub_to_main_uart,gyroBuffer,2);
            break;
        case 0x02:
            //distanceの取得
            UseTof();
            uart_write_blocking(sub_to_main_uart,(uint8_t[]){(uint8_t)(distance >> 8),(uint8_t)distance},2);
            break;
        case 0x03:
            //colorの取得
            color = UseColorSensor();
            uart_write_blocking(sub_to_main_uart,(uint8_t[]){color},1);
            break;
        case 0x04:
            //currentの取得
            UseCurrentSensor(0);
            UseCurrentSensor(1);
            UseCurrentSensor(2);
            uart_write_blocking(sub_to_main_uart,currentBuffer,6);
            break;
        case 0x11:{
            uint8_t gpio,angle;
            //サーボを動かす
            uart_read_blocking(sub_to_main_uart,&gpio,1);
            uart_read_blocking(sub_to_main_uart,&angle,1);
            SetServoAngle(gpio,angle);
            break;
        }
        case 0x12:{
            //吸引DCを制御する
            uint8_t duty;
            uart_read_blocking(sub_to_main_uart,&duty,1);
            SetSuctionMotorSpeed(duty * 4);
            break;
        }
        case 0x13:{
            //サーボをオフにする
            uint8_t gpio;
            uart_read_blocking(sub_to_main_uart,&gpio,1);
            SetServoOff(gpio);
            break;
        }
        case 0x14:{
            //ジャイロをリセットする
            uint8_t halfAngle;
            uart_read_blocking(sub_to_main_uart,&halfAngle,1);
            correctionAngle = (int)(halfAngle) * 2;
            isGyroReset = true;
            break;
        }
        case 0x21:{
            //赤玉の排出(通常)
            SetServoAngle(servo_left_basket_pin,140);
            sleep_ms(500);
            SetServoAngle(servo_left_basket_pin,120);
            sleep_ms(500);
            SetServoAngle(servo_left_basket_pin,100);
            sleep_ms(500);
            SetServoAngle(servo_left_basket_pin,80);
            sleep_ms(500);
            SetServoAngle(servo_left_basket_pin,60);
            sleep_ms(2000);
            SetServoAngle(servo_left_basket_pin,160);
            sleep_ms(2000);
            SetServoOff(servo_left_basket_pin);
            break;
        }
        case 0x22:{
            //赤玉の排出(同時に２つ)
            SetServoAngle(servo_left_basket_pin,140);
            SetServoAngle(servo_right_basket_pin,140);
            sleep_ms(500);
            SetServoAngle(servo_left_basket_pin,120);
            SetServoAngle(servo_right_basket_pin,120);
            sleep_ms(500);
            SetServoAngle(servo_left_basket_pin,100);
            SetServoAngle(servo_right_basket_pin,100);
            sleep_ms(500);
            SetServoAngle(servo_left_basket_pin,80);
            SetServoAngle(servo_right_basket_pin,80);
            sleep_ms(500);
            SetServoAngle(servo_left_basket_pin,60);
            SetServoAngle(servo_right_basket_pin,60);
            sleep_ms(2000);
            SetServoAngle(servo_left_basket_pin,160);
            SetServoAngle(servo_right_basket_pin,160);
            sleep_ms(2000);
            SetServoOff(servo_left_basket_pin);
            SetServoOff(servo_right_basket_pin);
            break;
        }
        case 0x23:{
            //青玉の排出(通常)
            SetServoAngle(servo_right_basket_pin,140);
            sleep_ms(500);
            SetServoAngle(servo_right_basket_pin,120);
            sleep_ms(500);
            SetServoAngle(servo_right_basket_pin,100);
            sleep_ms(500);
            SetServoAngle(servo_right_basket_pin,80);
            sleep_ms(500);
            SetServoAngle(servo_right_basket_pin,60);
            sleep_ms(2000);
            SetServoAngle(servo_right_basket_pin,160);
            sleep_ms(2000);
            SetServoOff(servo_right_basket_pin);
            break;
        }
        case 0x24:{
            //黄缶の排出(通常)
            SetServoAngle(servo_center_basket_pin,60);
            sleep_ms(3000);
            SetServoAngle(servo_center_basket_pin,160);
            sleep_ms(2000);
            SetServoOff(servo_center_basket_pin);
            break;
        }
        case 0x31:{
            //から線のLEDつける
            gpio_put(color_sensor_LED_pin,1);
            break;
        }
        default:
            break;
        }
    }
}

//サーボの角度を設定する関数
//pin : ピン番号
//angle : 整数の角度(10°～170°(GPIO8,10は160°まで?))
void SetServoAngleFromMain(unsigned int gpio,int angle){
    uart_write_blocking(main_to_sub_uart,(uint8_t[]){0x11,(uint8_t)gpio,(uint8_t)angle},3);
    sleep_ms(1);
}

void SetServoOffFromMain(unsigned int gpio){
    uart_write_blocking(main_to_sub_uart,(uint8_t[]){0x13,(uint8_t)gpio},2);
    sleep_ms(1);
}

// 対象を捨てるプログラム
//object = 10 : 赤玉
//object = 11 : 赤玉(青玉のほうにもためてる)
//object = 2  : 青玉
//object = 3  : 黄缶
void TrashfromBasketFromMain(int object){
    if(object == 10){
        uart_write_blocking(main_to_sub_uart,(uint8_t[]){0x21},1);
        sleep_ms(3150);
    }else if(object == 11){
        uart_write_blocking(main_to_sub_uart,(uint8_t[]){0x22},1);
        sleep_ms(3150);
    }else if(object == 2){
        uart_write_blocking(main_to_sub_uart,(uint8_t[]){0x23},1);
        sleep_ms(3150);
    }else if(object == 3){
        SetServoAngleFromMain(servo_arm_up_and_down_pin,90);
        uart_write_blocking(main_to_sub_uart,(uint8_t[]){0x24},1);
        sleep_ms(2900);
        SetServoAngleFromMain(servo_arm_up_and_down_pin,50);
    }
}

//吸引のモーターのスピードを制御する関数
//4で割った数値を代入する(duty = 300の場合は75)
void SetSuctionMotorSpeedFromMain(uint8_t duty){
   uart_write_blocking(main_to_sub_uart,(uint8_t[]){0x12,(uint8_t)duty},2);
}

//ジャイロをリセットする関数
//correctionAngle : gyroの補正用の角度
//※使用後1.2秒はジャイロを使わないこと
void ResetGyroFromMain(int correctionAngle){
   uart_write_blocking(main_to_sub_uart,(uint8_t[]){0x14,(uint8_t)(correctionAngle / 2)},2);
}

//メインマイコンがサブマイコンからAngleXを取得する関数
void GetGyroAngleFromSub(){
    // printf("送信\n");
    uart_write_blocking(main_to_sub_uart,(uint8_t[]){0x01},1);
    // printf("受信待ち\n");
    uart_read_blocking(main_to_sub_uart,gyroBuffer,2);
    // printf("受信\n");
    angleX = ((gyroBuffer[1] << 8) | gyroBuffer[0]) / 16.0 + correctionAngle;
    if(angleX >= 360) angleX -= 360;
    if(angleX < 0) angleX += 360;
    if(serialWatch == "gyr"){
        if(isUseDisplay){
            DrawCircleOnDisplay(5,20,20);
            DrawLineOnDisplay(25,40,20,radian(angleX)-1.57);
            WriteTextOnDisplay(60,30,"angleX",8,false,false);
            snprintf(displayBuffer,displayBufferSize,"%f",angleX);
            WriteTextOnDisplay(60,40,displayBuffer,8,false,false);
        }else{
            printf("angleX : %f\n",angleX);
        }
    }
}

//メインマイコンがサブマイコンからdistanceを取得する関数
void GetDistanceFromSub(){
    uint8_t data[2];
    // printf("tof待機");
    uart_write_blocking(main_to_sub_uart,(uint8_t[]){0x02},1);
    // printf(" tof書き込み完了\n");
    UARTReadTimeout(main_to_sub_uart,data,2,10);
    // printf(" tof読み取り完了\n");
    distance = (data[0] << 8) | data[1];
    if(serialWatch == "tof"){
        if(isUseDisplay){
            if (distance == 0xFFFF){
                WriteTextOnDisplay(5,30,"Measurement",8,false,false);
                WriteTextOnDisplay(5,40,"timed out",8,false,false);
            }else{
                snprintf(displayBuffer,displayBufferSize,"%u mm",distance);
                WriteTextOnDisplay(5,30,displayBuffer,8,false,false);
            }
        }else{
            if (distance == 0xFFFF){
                printf("Measurement timed out\n");
            }else{
                printf("Distance: %u mm\n", distance);
            }
        }
    }
}

void TurnOnColorLEDFromMain(){
    uart_write_blocking(main_to_sub_uart,(uint8_t[]){0x31},1);
}

//メインマイコンがサブマイコンからcolorを取得する関数
void GetColorFromSub(){
    uart_write_blocking(main_to_sub_uart,(uint8_t[]){0x03},1);
    UARTReadTimeout(main_to_sub_uart,&color,1,10);
    if(serialWatch == "col"){
        if(isUseDisplay){
            if(color == 1){
                WriteTextOnDisplay(5,30,"red",12,false,false);
            }else if(color == 2){
                WriteTextOnDisplay(5,30,"blue",12,false,false);
            }else if(color == 3){
                WriteTextOnDisplay(5,30,"can",12,false,false);
            }else if(color == 255){
                WriteTextOnDisplay(5,30,"i2c error",12,false,false);
            }else if(color == 127){
                WriteTextOnDisplay(5,30,"No Object",12,false,false);
            }else{
                snprintf(displayBuffer,displayBufferSize,"%u error",color);
                WriteTextOnDisplay(5,30,displayBuffer,12,false,false);
            }
        }else{
            if(color == 1){
                printf("red\n");
            }else if(color == 2){
                printf("blue\n");
            }else if(color == 3){
                printf("can\n");
            }
        }
    }
}

//メインマイコンがサブマイコンからcurrentを取得する関数
//        left  : 0
//        right : 1
//        DC    : 2 
void GetCurrentFromSub(){
    uart_write_blocking(main_to_sub_uart,(uint8_t[]){0x04},1);
    UARTReadTimeout(main_to_sub_uart,currentBuffer,6,10);
    for(int i = 0;i < 3;i++){
        current[i] = ((uint16_t)currentBuffer[i * 2]) << 8 | (uint16_t)currentBuffer[i * 2 + 1]; 
    }
    if(serialWatch == "cur"){
        if(isUseDisplay){
            snprintf(displayBuffer,displayBufferSize,"left hand : %u",current[0]);
            WriteTextOnDisplay(5,30,displayBuffer,8,false,false);
            snprintf(displayBuffer,displayBufferSize,"right hand : %u",current[1]);
            WriteTextOnDisplay(5,40,displayBuffer,8,false,false);
            snprintf(displayBuffer,displayBufferSize,"Arm : %u",current[2]);
            WriteTextOnDisplay(5,50,displayBuffer,8,false,false);
        }else{
            printf("left hand : %u right hand : %u Arm : %u\n",current[0],current[1],current[2]);
        }
    }
}

bool UARTReadTimeout(uart_inst_t *uart, uint8_t *data, int length, uint32_t timeout_ms){
    uint32_t start = time_us_32();

    for(int i = 0; i < length; i++){
        while(!uart_is_readable(uart)){
            if(time_us_32() - start > timeout_ms * 1000){
                return false;   // タイムアウト
            }
        }
        data[i] = uart_getc(uart);
    }

    return true;    // 正常受信
}