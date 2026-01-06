#include "display.hpp"
#include "../config.hpp"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "u8g2.h"
#include <math.h>
#include <string>

u8g2_t u8g2;
float DisplayPreTime = 0;

void DisplaySetup(){
    // ディスプレイ初期化（I2C + ノーブランドSSD1309用）
    i2c_init(display_i2c, 400 * 1000);  // 400kHz
    gpio_set_function(display_SDA_pin, GPIO_FUNC_I2C);  // SDA
    gpio_set_function(display_SCL_pin, GPIO_FUNC_I2C);  // SCL
    gpio_pull_up(display_SDA_pin);//ここいる？
    gpio_pull_up(display_SCL_pin);
    printf("1");
    //0_fを2_fにするとピン番号の向きが反転する
    //うまくいかなかったら1306でやってみる
    u8g2_Setup_ssd1309_i2c_128x64_noname0_f(
        &u8g2, U8G2_R0, u8x8_byte_pico_i2c, u8x8_gpio_and_delay_cb);
    printf("2");
    u8g2_SetI2CAddress(&u8g2, 0x78); // I2Cアドレス (8bit形式) ←これあってる？
    printf("3");
    u8g2_InitDisplay(&u8g2);
    printf("4");
    u8g2_SetPowerSave(&u8g2, 0); // 電源ON

    u8g2_SetContrast(&u8g2, 128);  // 最大
}

uint8_t u8x8_byte_pico_i2c(u8x8_t *u8x8, uint8_t msg,uint8_t arg_int, void *arg_ptr) {
    static uint8_t buffer[32];		/* u8g2/u8x8 will never send more than 32 bytes between START_TRANSFER and END_TRANSFER */
    static uint8_t buf_idx;
    uint8_t *data;
 
    switch(msg)
    {
        case U8X8_MSG_BYTE_SEND:
            data = (uint8_t *)arg_ptr;      
            while( arg_int > 0 )
            {
	            buffer[buf_idx++] = *data;
	            data++;
	            arg_int--;
            }      
            break;
        case U8X8_MSG_BYTE_INIT:
            /* add your custom code to init i2c subsystem */
            break;
        case U8X8_MSG_BYTE_SET_DC:
            /* ignored for i2c */
            break;
        case U8X8_MSG_BYTE_START_TRANSFER:
            buf_idx = 0;
            break;
        case U8X8_MSG_BYTE_END_TRANSFER:
            i2c_write_blocking(display_i2c,u8x8_GetI2CAddress(u8x8) >> 1,buffer,buf_idx,false);
            break;
        default:
            return 0;
  }
  return 1;
}

// GPIOコールバック（未使用でも定義が必要）
uint8_t u8x8_gpio_and_delay_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    switch (msg) {
        case U8X8_MSG_DELAY_MILLI:
            sleep_ms(arg_int);
            break;
        case U8X8_MSG_GPIO_AND_DELAY_INIT:
        case U8X8_MSG_GPIO_RESET:
        case U8X8_MSG_GPIO_CS:
        case U8X8_MSG_GPIO_DC:
        case U8X8_MSG_GPIO_I2C_CLOCK:
        case U8X8_MSG_GPIO_I2C_DATA:
            // I2CはGPIO操作不要
            break;
    }
    return 1;
}