#include "display.hpp"
#include "gyro.hpp"
#include "main_to_sub.hpp"
#include "other_sensor.hpp"
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
            uart_putc(sub_to_main_uart,(uint8_t)(distance >> 8));
            uart_putc(sub_to_main_uart,(uint8_t)distance);
            break;
        case 0x03:
            //colorの取得
            break;
        default:
            break;
        }
    }
}

//サーボの角度を設定するプログラム
//pin : ピン番号
//angle : 整数の角度(0°～180°(GPIO8,10は160°まで?))
void SetServoAngleFromMain(unsigned int gpio,int angle){
    
}

//メインマイコンがサブマイコンからAngleXを取得する関数
void GetGyroAngleFromSub(){
    uart_putc(main_to_sub_uart,0x01);
    uart_read_blocking(main_to_sub_uart,gyroBuffer,2);
    AngleX = ((gyroBuffer[1] << 8) | gyroBuffer[0]) / 16.0;
}

//メインマイコンがサブマイコンからdistanceを取得する関数
void GetDistanceFromSub(){
    uint8_t data[2];
    uart_putc(main_to_sub_uart,0x02);
    uart_read_blocking(main_to_sub_uart,data,2);
    distance = (data[0] << 8) | data[1];
}

//メインマイコンがサブマイコンからcolorを取得する関数
void GetColorFromSub(){
    uart_putc(main_to_sub_uart,0x03);
    uart_read_blocking(main_to_sub_uart,&color,1);
}