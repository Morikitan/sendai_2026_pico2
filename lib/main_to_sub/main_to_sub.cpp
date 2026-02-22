#include "display.hpp"
#include "gyro.hpp"
#include "main_to_sub.hpp"
#include "motor.hpp"
#include "other_sensor.hpp"
#include "servo.hpp"
#include "tof.hpp"
#include "../config.hpp"
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include <stdio.h>

#define BAUDRATE 115200

void MainToSubSetup(){
    gpio_init(main_to_line_RX_pin);
    gpio_init(main_to_line_TX_pin);
    gpio_set_function(main_to_line_RX_pin,GPIO_FUNC_UART);
    gpio_set_function(main_to_line_TX_pin,GPIO_FUNC_UART);
    uart_init(main_to_sub_uart,BAUDRATE);
}

void SubToMainSetup(){
    gpio_init(line_to_main_RX_pin);
    gpio_init(line_to_main_TX_pin);
    gpio_set_function(line_to_main_RX_pin,GPIO_FUNC_UART);
    gpio_set_function(line_to_main_TX_pin,GPIO_FUNC_UART);
    uart_set_irq_enables(sub_to_main_uart,true,false);
    irq_set_exclusive_handler(UART0_IRQ,SubCallBack);
    irq_set_enabled(UART0_IRQ,true);
    uart_init(sub_to_main_uart,BAUDRATE);
}

void SubCallBack(){
    while(uart_is_readable(sub_to_main_uart)){
        uint8_t data = (uint8_t)uart_getc(sub_to_main_uart);
        switch (data)
        {
        case 0x01:
            //AngleXの取得
            uart_write_blocking(sub_to_main_uart,gyroBuffer,2);
            break;
        case 0x02:
            //distanceの取得
            UseTof();
            uart_write_blocking(sub_to_main_uart,(uint8_t[]){(uint8_t)(distance >> 8)},1);
            uart_write_blocking(sub_to_main_uart,(uint8_t[]){(uint8_t)distance},1);
            break;
        case 0x03:
            //colorの取得
            UseColorSensor();
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
            //サーボを動かす
            while(!uart_is_readable(sub_to_main_uart));
            uint gpio = (uint)uart_getc(sub_to_main_uart);
            while(!uart_is_readable(sub_to_main_uart));
            int angle = (int)uart_getc(sub_to_main_uart);
            SetServoAngle(gpio,angle);
            break;
        }
        case 0x12:{
            //吸引DCを制御する
            while(!uart_is_readable(sub_to_main_uart));
            uint duty = (uint)uart_getc(sub_to_main_uart);
            SetSuctionMotorSpeed(duty);
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
}

//吸引のモーターのスピードを制御する関数
void SetSuctionMotorFromMain(int duty){
   uart_write_blocking(main_to_sub_uart,(uint8_t[]){0x12,(uint8_t)duty},2);
}

//メインマイコンがサブマイコンからAngleXを取得する関数
void GetGyroAngleFromSub(){
    uart_write_blocking(main_to_sub_uart,(uint8_t[]){0x01},1);
    uart_read_blocking(main_to_sub_uart,gyroBuffer,2);
    angleX = ((gyroBuffer[1] << 8) | gyroBuffer[0]) / 16.0;
}

//メインマイコンがサブマイコンからdistanceを取得する関数
void GetDistanceFromSub(){
    uint8_t data[2];
    uart_write_blocking(main_to_sub_uart,(uint8_t[]){0x02},1);
    uart_read_blocking(main_to_sub_uart,data,2);
    distance = (data[0] << 8) | data[1];
}

//メインマイコンがサブマイコンからcolorを取得する関数
void GetColorFromSub(){
    uart_write_blocking(main_to_sub_uart,(uint8_t[]){0x03},1);
    uart_read_blocking(main_to_sub_uart,&color,1);
}

//メインマイコンがサブマイコンからcurrentを取得する関数
void GetCurrentFromSub(){
    uart_write_blocking(main_to_sub_uart,(uint8_t[]){0x04},1);
    uart_read_blocking(main_to_sub_uart,currentBuffer,6);
    for(int i = 0;i < 3;i++){
        current[i] = ((uint16_t)currentBuffer[i * 2]) << 8 | (uint16_t)currentBuffer[i * 2 + 1]; 
    }
}