
#pragma once

#include "u8g2.h"
#include <string>

#ifdef __cplusplus
extern "C" {
#endif

void DisplaySetup();
void WriteTextOnDisplay(char x,char y,const char* word,char size,bool isFirstLine,bool isLastLine);
void PrintDisplayMode();
void DrawCircleOnDisplay(int x,int y,int r);
void DrawLineOnDisplay(int x0,int y0,int length,float angle);
void DrawPixelOnDisplay(int x,int y);
uint8_t u8x8_byte_pico_i2c(u8x8_t *u8x8, uint8_t msg,uint8_t arg_int, void *arg_ptr);
uint8_t u8x8_gpio_and_delay_cb(u8x8_t *u8x8, uint8_t msg,uint8_t arg_int, void *arg_ptr);
void SetDisplayMode();
void DrawLineMapOnDisplay();
void Draw4x4CircleOnDisplay(char x,char y);
void Draw2x2BoxOnDisplay(char x,char y);

#ifdef __cplusplus
}
#endif