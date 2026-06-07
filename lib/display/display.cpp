#include "action.hpp"
#include "display.hpp"
#include "../config.hpp"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "u8g2.h"
#include <math.h>
#include <string>

std::string serialWatch;
u8g2_t u8g2;
float displayPreTime = 0;
char displayBuffer[displayBufferSize];
int displayMode;
int circle20[20][2] = {
    {60,30},{59,21},{54,12},{48,6},{39,1},
    {30,0},{21,1},{12,6},{6,12},{1,21},
    {0,30},{1,39},{6,48},{12,54},{21,59},
    {30,60},{39,59},{48,54},{54,48},{59,39}
};

void DisplaySetup(){
    /******************
    1  : hom 通常モード(modeを表示)〇
    2  : cam1 カメラの値の前半(ボールの位置と色とか？)
    3  : cam2 カメラの値の後半
    4  : col カラーセンサの値
    5  : cur 電流センサの値〇
    6  : gyr 機体の角度(AngleX)〇
    7  : lin ラインセンサーの値(0か1で受け取る)〇
    8  : tim 1回の経過時間(ミリ秒)〇
    9  : tof tofの値〇
    10 : oth その他(時によって変わる)〇
    *******************/
    serialWatch = "hom";
    SetDisplayMode();

    // ディスプレイ初期化（I2C + ノーブランドSSD1309用）
    i2c_init(display_i2c, 400 * 1000);  // 400kHz
    gpio_init(display_SCL_pin);
    gpio_init(display_SDA_pin);
    gpio_set_function(display_SDA_pin, GPIO_FUNC_I2C);  // SDA
    gpio_set_function(display_SCL_pin, GPIO_FUNC_I2C);  // SCL
    gpio_pull_up(display_SDA_pin);//ここいる？ → いる
    gpio_pull_up(display_SCL_pin);
    gpio_pull_up(display_reset_pin);

    //0_fを2_fにするとピン番号の向きが反転する
    //うまくいかなかったら1306でやってみる
    //u8g2_Setup_ssd1309_i2c_128x64_noname0_f
    printf("1");
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

//ディスプレイ上の説明欄(一番上の文字列)を生成する
void PrintDisplayMode(){
    if(displayMode == 1){
        serialWatch = "hom";
        WriteTextOnDisplay(5,15,"<Home>",12,true,false);
        snprintf(displayBuffer,displayBufferSize,"Red : %d",allRedBallNumber);
        WriteTextOnDisplay(5,25,displayBuffer,8,false,false);
        snprintf(displayBuffer,displayBufferSize,"Blue : %d",allBlueBallNumber);
        WriteTextOnDisplay(5,35,displayBuffer,8,false,false);
        snprintf(displayBuffer,displayBufferSize,"Can : %d",allCanNumber);
        WriteTextOnDisplay(5,45,displayBuffer,8,false,false);
        if(isYosen){
            WriteTextOnDisplay(5,55,"yosen",8,false,false);
        }else{
            WriteTextOnDisplay(5,55,"honsen",8,false,false);
        }
    }else if(displayMode == 2){
        serialWatch = "cam1";
        WriteTextOnDisplay(5,15,"<Camera1>",12,true,false);
    }else if(displayMode == 3){
        serialWatch = "cam2";
        WriteTextOnDisplay(5,15,"<Camera2>",12,true,false);
    }else if(displayMode == 4){
        serialWatch = "col";
        WriteTextOnDisplay(5,15,"<ColorSen>",12,true,false);
    }else if(displayMode == 5){
        serialWatch = "cur";
        WriteTextOnDisplay(5,15,"<CurSen>",12,true,false);
    }else if(displayMode == 6){
        serialWatch = "gyr";
        WriteTextOnDisplay(5,15,"<GyroSensor>",12,true,false);
    }else if(displayMode == 7){
        serialWatch = "lin";
        WriteTextOnDisplay(64,15,"<Line>",12,true,false);
        DrawLineMapOnDisplay();
    }else if(displayMode == 8){
        serialWatch = "tim";
        WriteTextOnDisplay(5,15,"<DeltaTime>",12,true,false);
        snprintf(displayBuffer,displayBufferSize,"%fms",timer_hw->timerawl / 1000.0-displayPreTime);
        WriteTextOnDisplay(5,30,displayBuffer,12,false,false);
        displayPreTime = timer_hw->timerawl / 1000.0;
    }else if(displayMode == 9){
        serialWatch = "tof";
        WriteTextOnDisplay(5,15,"<Tof>",12,true,false);
    }else if(displayMode == 10){
        serialWatch = "oth";
        WriteTextOnDisplay(5,15,"<Others>",12,true,false);
    }else{
        serialWatch = "???";
        WriteTextOnDisplay(5,15,"<error>",12,true,false);
    }
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


//SerialWatchの値に対応したdisplayModeの値を設定する
void SetDisplayMode(){
    /******************
    1  : hom 通常モード(modeを表示)
    2  : cam カメラの値(ボールの位置と色とか？)
    3  : col カラーセンサの値
    4  : cur 電流センサの値
    5  : gyr 機体の角度(AngleX)
    6  : lin ラインセンサーの値(0か1で受け取る)
    7  : tim 1回の経過時間(ミリ秒)
    8  : tof tofの値
    9  : oth その他(時によって変わる)
    *******************/
    if(serialWatch == "hom"){
        displayMode = 1;
    }else if(serialWatch == "cam1"){
        displayMode = 2;
    }else if(serialWatch == "cam2"){
        displayMode = 3;
    }else if(serialWatch == "col"){
        displayMode = 4;
    }else if(serialWatch == "cur"){
        displayMode = 5;
    }else if(serialWatch == "gyr"){
        displayMode = 6;
    }else if(serialWatch == "lin"){
        displayMode = 7;
    }else if(serialWatch == "tim"){
        displayMode = 8;
    }else if(serialWatch == "tof"){
        displayMode = 9;
    }else if(serialWatch == "oth"){
        displayMode = 10;
    }
}

//ラインセンサの表示用の枠を描画する。地獄。
void DrawLineMapOnDisplay(){
    for(int i = 0;i < 20;i++){
        Draw4x4CircleOnDisplay(circle20[i][0],circle20[i][1]);
    }
    Draw4x4CircleOnDisplay(80,22);
    Draw4x4CircleOnDisplay(80,30);
    Draw4x4CircleOnDisplay(80,38);
}

//□■■□
//■□□■
//■□□■
//□■■□
//上の形の円を描く関数
// x : 円の左端のx座標
// y : 円の上端のy座標
void Draw4x4CircleOnDisplay(char x,char y){
    DrawPixelOnDisplay(x+1,y);
    DrawPixelOnDisplay(x+2,y);
    DrawPixelOnDisplay(x,y+1);
    DrawPixelOnDisplay(x+3,y+1);
    DrawPixelOnDisplay(x,y+2);
    DrawPixelOnDisplay(x+3,y+2);
    DrawPixelOnDisplay(x+1,y+3);
    DrawPixelOnDisplay(x+2,y+3);
}

//2x2の四角を描く関数
// x : 円の左端のx座標
// y : 円の上端のy座標
void Draw2x2BoxOnDisplay(char x,char y){
    DrawPixelOnDisplay(x,y);
    DrawPixelOnDisplay(x,y+1);
    DrawPixelOnDisplay(x+1,y);
    DrawPixelOnDisplay(x+1,y+1);
}

void SendBufferToDisplay(){
    u8g2_SendBuffer(&u8g2);
}