#include <stdio.h>
#include "camera.hpp"
#include "display.hpp"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "../config.hpp"

#define BAUDRATE 115200

CameraInformation cameraInformation[16]; 

void CameraSetup() {
    stdio_init_all();
    sleep_ms(500); 
    gpio_init(camera_RX_pin);
    gpio_init(camera_TX_pin);
    gpio_set_function(camera_TX_pin, GPIO_FUNC_UART);
    gpio_set_function(camera_RX_pin, GPIO_FUNC_UART);
    uart_init(camera_uart, BAUDRATE);
    printf("Camera Setup Complete!\n");
}

int UseCamera() {
    uint8_t cmd = 0x01;
    uint8_t data_length = 0;

    uart_write_blocking(camera_uart, &cmd, 1);

    uart_read_blocking(camera_uart, &data_length, 1);

    if (data_length > 0) {
        if(data_length > 64 || data_length % 3 != 0){
            return 0;
        }
        uint8_t buffer[data_length];
        if(!uart_read_with_timeout(camera_uart, buffer, data_length,250)){
            return 0;
        }
        int num_objects = data_length / 3;
        // printf("Found %d objects!\n", num_objects);
        if(num_objects > 16){
            return 999;
        }
        for (int i = 0; i < num_objects; i++) {
            uint8_t obj_id = buffer[i * 3];
            uint8_t x_half = buffer[i * 3 + 1];   // 2で割られたX座標
            uint8_t y_half = buffer[i * 3 + 2];   // 2で割られたY座標
            cameraInformation[i].x = x_half * 2;              // 0~160を0~320に変換
            cameraInformation[i].y = y_half * 2;
            if(serialWatch == "cam"){
                if(isUseDisplay){
                    if(i < 4){
                        if (obj_id == 3) {
                            snprintf(displayBuffer,displayBufferSize,"Red X%d Y%d",realxy[0],real_y);
                        } else if (obj_id == 4) {
                            snprintf(displayBuffer,displayBufferSize,"Blu X%d Y%d",real_x,real_y);
                        } else if (obj_id == 5) {
                            snprintf(displayBuffer,displayBufferSize,"CaV X%d Y%d",real_x,real_y);
                        } else if (obj_id == 6) {
                            snprintf(displayBuffer,displayBufferSize,"CaH X%d Y%d",real_x,real_y);
                        } else {
                            snprintf(displayBuffer,displayBufferSize,"??? X%d Y%d",real_x,real_y);
                        }
                        WriteTextOnDisplay(5,30+i*10,displayBuffer,8,false,false);  
                    }else if(i < 8){
                        if (obj_id == 3) {
                            snprintf(displayBuffer,displayBufferSize,"Red");
                        } else if (obj_id == 4) {
                            snprintf(displayBuffer,displayBufferSize,"Blu");
                        } else if (obj_id == 5) {
                            snprintf(displayBuffer,displayBufferSize,"CaV");
                        } else if (obj_id == 6) {
                            snprintf(displayBuffer,displayBufferSize,"CaH");
                        } else {
                            snprintf(displayBuffer,displayBufferSize,"???");
                        }
                        WriteTextOnDisplay(100,-10+i*10,displayBuffer,8,false,false);
                    }
                }else{
                    if (obj_id == 3) {
                        printf("  [赤] ID:%d X:%d Y:%d\n", obj_id, real_x,real_y);
                    } else if (obj_id == 4) {
                        printf("  [青] ID:%d, X:%d Y:%d\n", obj_id, real_x,real_y);
                    } else if (obj_id == 5) {
                        printf("  [縦缶] ID:%d, X:%d Y:%d\n", obj_id, real_x,real_y);
                    } else if (obj_id == 6) {
                        printf("  [横缶] ID:%d, X:%d Y:%d\n", obj_id, real_x,real_y);
                    } else {
                        printf("  [Unknown] ID:%d, X:%d Y:%d\n", obj_id, real_x,real_y);
                    }
                }
            }
            return num_objects;
        }
    } else {
        printf("No objects found.\n");
    }
    return 0;
}

bool uart_read_with_timeout(uart_inst_t *uart, uint8_t *buffer, size_t length, uint32_t timeout_ms) {
    absolute_time_t start = get_absolute_time();
    size_t count = 0;

    while (count < length) {
        // データが来ていれば読む
        if (uart_is_readable(uart)) {
            buffer[count++] = uart_getc(uart);
        }

        // タイムアウト判定
        if (absolute_time_diff_us(start, get_absolute_time()) > timeout_ms * 1000) {
            return false; // タイムアウト
        }
    }

    return true; // 成功
}