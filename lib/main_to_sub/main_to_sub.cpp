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

#define BAUDRATE 115200

void MainToSubSetup(){
    gpio_init(main_to_sub_RX_pin);
    gpio_init(main_to_sub_TX_pin);
    gpio_set_function(main_to_sub_RX_pin,GPIO_FUNC_UART);
    gpio_set_function(main_to_sub_TX_pin,GPIO_FUNC_UART);
    uart_init(main_to_sub_uart,BAUDRATE);
}

void SubToMainSetup(){
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
            uint8_t gpio;
            uart_read_blocking(sub_to_main_uart,&gpio,1);
            SetServoOff(gpio);
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

//吸引のモーターのスピードを制御する関数
//4で割った数値を代入する(duty = 300の場合は75)
void SetSuctionMotorSpeedFromMain(uint8_t duty){
   uart_write_blocking(main_to_sub_uart,(uint8_t[]){0x12,(uint8_t)duty},2);
}

//メインマイコンがサブマイコンからAngleXを取得する関数
void GetGyroAngleFromSub(){
    // printf("送信\n");
    uart_write_blocking(main_to_sub_uart,(uint8_t[]){0x01},1);
    // printf("受信待ち\n");
    uart_read_blocking(main_to_sub_uart,gyroBuffer,2);
    // printf("受信\n");
    angleX = ((gyroBuffer[1] << 8) | gyroBuffer[0]) / 16.0;
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
    uart_read_blocking(main_to_sub_uart,data,2);
    // printf(" tof読み取り完了\n");
    distance = (data[0] << 8) | data[1];
    if(serialWatch == "tof"){
        if(isUseDisplay){
            if (distance == 0xFF){
                WriteTextOnDisplay(5,30,"Measurement",8,false,false);
                WriteTextOnDisplay(5,40,"timed out",8,false,false);
            }else{
                snprintf(displayBuffer,displayBufferSize,"%u mm",distance);
                WriteTextOnDisplay(5,30,displayBuffer,8,false,false);
            }
        }else{
            if (distance == 0xFF){
                printf("Measurement timed out\n");
            }else{
                printf("Distance: %u mm\n", distance);
            }
        }
    }
}

//メインマイコンがサブマイコンからcolorを取得する関数
void GetColorFromSub(){
    uart_write_blocking(main_to_sub_uart,(uint8_t[]){0x03},1);
    uart_read_blocking(main_to_sub_uart,&color,1);
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
void GetCurrentFromSub(){
    uart_write_blocking(main_to_sub_uart,(uint8_t[]){0x04},1);
    uart_read_blocking(main_to_sub_uart,currentBuffer,6);
    for(int i = 0;i < 3;i++){
        current[i] = ((uint16_t)currentBuffer[i * 2]) << 8 | (uint16_t)currentBuffer[i * 2 + 1]; 
    }
    if(serialWatch == "cur"){
        if(isUseDisplay){
            snprintf(displayBuffer,displayBufferSize,"left hand : %u",current[0]);
            WriteTextOnDisplay(5,30,displayBuffer,8,false,false);
            snprintf(displayBuffer,displayBufferSize,"right hand : %u",current[1]);
            WriteTextOnDisplay(5,40,displayBuffer,8,false,false);
            snprintf(displayBuffer,displayBufferSize,"DC : %u",current[2]);
            WriteTextOnDisplay(5,50,displayBuffer,8,false,false);
        }else{
            printf("left hand : %u right hand : %u DC : %u\n",current[0],current[1],current[2]);
        }
    }
}