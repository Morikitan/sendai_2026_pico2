#include "display.hpp"
#include "gyro.hpp"
#include "others.hpp"
#include "../config.hpp"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <stdio.h>

float angleX;
unsigned char gyroBuffer[2];

#define BNO_ADDRESS 0x28

void GyroSetup(){
    i2c_init(gyro_i2c,115200);
    gpio_set_function(gyro_SDA_pin, GPIO_FUNC_I2C);
    gpio_set_function(gyro_SCL_pin, GPIO_FUNC_I2C);
    i2c_write_blocking(gyro_i2c,BNO_ADDRESS,(uint8_t[]){0x00},1,true);
    uint8_t chip_id;
    i2c_read_blocking(gyro_i2c, BNO_ADDRESS, &chip_id, 1, false); 
    if (chip_id != 0xA0) {
        while (true) {
            printf("bno055が見つかりません。\n");
            sleep_ms(1000);
        }
    }
    //[][2]はかつてすべて80なのでおかしい時はそれにもどすことを検討
    uint8_t config[][3] = {
        {0x3D, 0x00, 50},
        {0x07, 0x00, 10},
        {0x3E, 0x00, 10},
        {0x3D, 0x08, 50}
    };
    for (int i = 0; i < 4; i++) {
        uint8_t reg = config[i][0];
        uint8_t value = config[i][1];
        uint8_t delay = config[i][2];

        uint8_t data[2] = {reg, value};
        i2c_write_blocking(gyro_i2c, BNO_ADDRESS, data, 2, false); 
        sleep_ms(delay); 
    }
    printf("bno055はIMUモードで正常に起動しました。\n");
}

void UseGyroSensor(){
    bool isBreak = false;
    float i2cTime = time_us_32() / 1000000.0;
    while(i2c_get_write_available(gyro_i2c) == false){
        if(time_us_32() / 1000000.0 - i2cTime > 0.1){
            isBreak = true;
            break;
        }
    }
    if(isBreak == true){
        
    }else{   
        i2c_write_blocking(gyro_i2c, BNO_ADDRESS, (uint8_t[]){0x1A}, 1, true);
        isBreak = false; 
        i2cTime = time_us_32() / 1000000.0;
        /*while(i2c_get_read_available(i2c1) == 0){
            if(time_us_32() / 1000000.0 - i2cTime > 0.1){
                isBreak = true;
                break;
            }
        }*/
        if(isBreak == true){
            if(isUseDisplay){
                WriteTextOnDisplay(5,15,"ジャイロ死亡",24,true,true);
            }else{
                printf("ジャイロ死亡\n");
            }
        }else{
            i2c_read_blocking(gyro_i2c, BNO_ADDRESS, gyroBuffer, 2, false); 
            angleX = ((gyroBuffer[1] << 8) | gyroBuffer[0]) / 16.0;
            if(serialWatch == "gyr"){
                if(isUseDisplay){
                    DrawCircleOnDisplay(5,20,20);
                    DrawLineOnDisplay(25,40,20,radian(angleX) -1.57);
                    WriteTextOnDisplay(60,30,"angleX",8,false,false);
                    snprintf(displayBuffer,displayBufferSize,"%f",angleX);
                    WriteTextOnDisplay(60,40,displayBuffer,8,false,false);
                }else{
                    printf("angleX : %f\n",angleX);
                }
            }
            // if(mode == 2 || mode == 4 || mode == 8 || mode == 10){
                // if (angleX > 180){
                    // angleX -= 180;
                // }else{
                    // angleX += 180;
                // }
            // }
        }
    }
}
