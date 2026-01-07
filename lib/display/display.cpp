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
char DisplayBuffer[DisplayBufferSize];
int DisplayMode;

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

// x : 左端からのx座標(0～127)
// y : 上端からのy座標(0～63)
// word : 書き込む文字列(char型の配列)
// size : 文字の大きさ。8 or 10 or 12 or 14 or 18 or 24
// isFirstLine : 最初の行かどうか
// isLastLine : 最後の行かどうか
void WriteTextOnDisplay(char x,char y,const char* text,char size,bool isFirstLine,bool isLastLine){
    // バッファをクリア
    if(isFirstLine == true)u8g2_ClearBuffer(&u8g2);                  
    // フォント選択
    if(size == 8)u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr); 
    else if(size == 10)u8g2_SetFont(&u8g2, u8g2_font_ncenB10_tr); 
    else if(size == 12)u8g2_SetFont(&u8g2, u8g2_font_ncenB12_tr); 
    else if(size == 14)u8g2_SetFont(&u8g2, u8g2_font_ncenB14_tr); 
    else if(size == 18)u8g2_SetFont(&u8g2, u8g2_font_ncenB18_tr); 
    else if(size == 24)u8g2_SetFont(&u8g2, u8g2_font_ncenB24_tr); 
    else return;
    // 文字列描画
    u8g2_DrawStr(&u8g2, x, y, text); 
    // 表示に反映
    if(isLastLine == true)u8g2_SendBuffer(&u8g2);                   
}

//円を描く関数
// x : 円の左端のx座標
// y : 円の上端のy座標
// r : 円の半径
void DrawCircleOnDisplay(int x,int y,int r){
    u8g2_DrawCircle(&u8g2,x+r,y+r,r,U8G2_DRAW_ALL);
}

//直線を描く関数
// x0 : 直線の開始点のx座標
// y0 : 直線の開始点のy座標
// length : 直線の長さ
// angle : 直線の開始点からの角度(座標平面での弧度法)
void DrawLineOnDisplay(int x0,int y0,int length,float angle){
    u8g2_DrawLine(&u8g2,x0,y0,x0 + int(length * cos(angle)),y0 + int(length * sin(angle)));
}

//点を打つ変数
// x : 点のx座標
// y : 点のy座標
void DrawPixelOnDisplay(int x,int y){
    u8g2_DrawPixel(&u8g2,x,y);
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