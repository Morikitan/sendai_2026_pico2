#include "camera.hpp"
#include "display.hpp"
#include "../config.hpp"
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include <math.h>
#include <stdio.h>

//カメラの初期化
void CameraSetup(){
    uart_init(camera_uart,9600);
    gpio_set_function(camera_TX_pin, GPIO_FUNC_UART);  // TXピン
    gpio_set_function(camera_RX_pin, GPIO_FUNC_UART);  // RXピン
}

void UseCamera(){
    // カメラからの情報を受け取る(UART)
    // その情報を使える形に加工する
    // ディスプレイで表示するorシリアルモニタで表示するための処理を忘れないように
}