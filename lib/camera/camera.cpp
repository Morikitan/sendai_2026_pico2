#include "camera.hpp"
#include "display.hpp"
#include "../config.hpp"
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include <math.h>
#include <stdio.h>

// int CameraDataNumber = 1;
// int HowManyData = 0;
// uint8_t CameraData1[] = {0,0,0,0,0,0,0};
// uint8_t CameraData[] = {0,0,0,0,0,0,0};

//カメラの初期化
void CameraSetup(){
    uart_init(camera_uart,9600);
    gpio_set_function(camera_TX_pin, GPIO_FUNC_UART);  // TXピン
    gpio_set_function(camera_RX_pin, GPIO_FUNC_UART);  // RXピン
}

void UseCamera(){
    
}