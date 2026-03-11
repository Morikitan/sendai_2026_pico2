#include "display.hpp"
#include "line.hpp"
#include "main_to_line.hpp"
#include "picoPioUart.pio.h"
#include "../config.hpp"
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include <stdio.h>

PIO pio;
uint sm_rx;
uint sm_tx;
uint offset;
uint offset2;
bool parity_check;

const uint SERIAL_BAUD = 125000;

void MainToLineSetup(){
    pio = pio0;

    sm_rx = 0;
    offset = pio_add_program(pio, &picoPioUartRx_program);
    picoPioUartRx_program_init(pio, sm_rx, offset, main_to_line_RX_pin, SERIAL_BAUD);

    sm_tx = 1;
    offset2 = pio_add_program(pio, &picoPioUartTx_program);
    picoPioUartTx_program_init(pio, sm_tx, offset2, main_to_line_TX_pin, SERIAL_BAUD);
}

void LineToMainSetup(){
    pio = pio0;

    sm_rx = 0;
    offset = pio_add_program(pio, &picoPioUartRx_program);
    picoPioUartRx_program_init(pio, sm_rx, offset, line_to_main_RX_pin, SERIAL_BAUD);

    sm_tx = 1;
    offset2 = pio_add_program(pio, &picoPioUartTx_program);
    picoPioUartTx_program_init(pio, sm_tx, offset2, line_to_main_TX_pin, SERIAL_BAUD);
    //割り込みを使うことを検討したい
}

void PutDataFromLineToMain(){
    uint8_t predata = 0;
    while(predata != 0x36){
        predata = picoPioUartRx_program_getc(true,&parity_check);
    }
    UseLineSensor();
    uint8_t data1 = 0x00;
    for(int i = 0;i < 8;i++){
        if(circleLineSensor[i] == true){
            //ラインが反応していたらそのビットを1に
            data1 |= 0x01 << i;
        }
    }
    printf("data1送信");
    picoPioUartTx_program_putc(data1,true);
    uint8_t data2 = 0x00;
    for(int i = 0;i < 8;i++){
        if(circleLineSensor[i + 8] == true){
            //ラインが反応していたらそのビットを1に
            data2 |= 0x01 << i;
        }
    }
    printf("data2送信");
    picoPioUartTx_program_putc(data2,true);
    uint8_t data3 = 0x00;
    for(int i = 0;i < 4;i++){
        if(circleLineSensor[i + 16] == true){
            //ラインが反応していたらそのビットを1に
            data3 |= 0x01 << i;
        }
    }
    for(int i = 0;i < 3;i++){
        if(frontLineSensor[i] == true){
            //ラインが反応していたらそのビットを1に
            data3 |= 0x01 << (i + 4);
        }
    }
    printf("data3送信");
    picoPioUartTx_program_putc(data3,true);
    
    uint8_t callBack = picoPioUartRx_program_getc(true,&parity_check);
    if(callBack == 0x12 && parity_check == true){
        //成功
        printf(" %X %X %X\n",data1,data2,data3);
        return;
    }else if(callBack == 0x24 || parity_check == false){
        //失敗
        printf(" 送信失敗\n");
        PutDataFromLineToMain();
    }
}

void GetDataFromLineToMain(){
    printf("データ要求");
    picoPioUartTx_program_putc(0x36,true);
    printf(" 受信1");
    uint8_t data1 = picoPioUartRx_program_getc(true,&parity_check);
    if(parity_check == false){
        printf(" 受信失敗");
        picoPioUartTx_program_putc(0x24,true);
        picoPioUartRx_program_clear_buffer();
        GetDataFromLineToMain();
        return;
    }
    for(int i = 0;i < 8;i++){
        if((data1 >> i & 0x01) == 0x01){
            //ビットを1なら反応している
            circleLineSensor[i] = true;
        }else{
            circleLineSensor[i] = false;
        }
    }
    printf(" 受信2");
    uint8_t data2 = picoPioUartRx_program_getc(true,&parity_check);
    if(parity_check == false){
        printf(" 受信失敗");
        picoPioUartTx_program_putc(0x24,true);
        picoPioUartRx_program_clear_buffer();
        GetDataFromLineToMain();
        return;
    }
    for(int i = 0;i < 8;i++){
        if((data2 >> i & 0x01) == 0x01){
            //ビットを1なら反応している
            circleLineSensor[i+8] = true;
        }else{
            circleLineSensor[i+8] = false;
        }
    }
    printf(" 受信3");
    uint8_t data3 = picoPioUartRx_program_getc(true,&parity_check);
    if(parity_check == false){
        printf(" 受信失敗");
        picoPioUartTx_program_putc(0x24,true);
        picoPioUartRx_program_clear_buffer();
        GetDataFromLineToMain();
        return;
    }
    for(int i = 0;i < 4;i++){
        if((data3 >> i & 0x01) == 0x01){
            //ビットを1なら反応している
            circleLineSensor[i+16] = true;
        }else{
            circleLineSensor[i+16] = false;
        }
    }
    for(int i = 0;i < 3;i++){
        if((data3 >> (i+4) & 0x01) == 0x01){
            //ビットを1なら反応している
            frontLineSensor[i] = true;
        }else{
            frontLineSensor[i] = false;
        }
    }

    if(serialWatch == "lin"){
        if(isUseDisplay){
            for(int i = 0;i < 20;i++){
                if(circleLineSensor[i])Draw2x2BoxOnDisplay(circle20[i][0]+1,circle20[i][1]+1);
            }
            if(frontLineSensor[0])Draw2x2BoxOnDisplay(81,23);
            if(frontLineSensor[1])Draw2x2BoxOnDisplay(81,31);
            if(frontLineSensor[2])Draw2x2BoxOnDisplay(81,39);
            SendBufferToDisplay();
        }else{
            printf("Circle : ");
            for(int i = 0;i < 20;i++){
                if(circleLineSensor[i]){
                    printf("1");
                }else{
                    printf("0");
                }
            }
            printf(" Front : ");
            for(int i = 0;i < 3;i++){
                if(frontLineSensor[i]){
                    printf("1");
                }else{
                    printf("0");
                }
            }
            printf("\n");
        }
    }
    printf(" %X %X %X\n",data1,data2,data3);
    //データを受け取り終わったら返信する
    picoPioUartTx_program_putc(0x12,true);
}

//UART(シリアル通信)で送信する関数
//
//data : 送るデータ(uint8_t型)
//even_parity : 偶数か奇数のどちらになるようにパリティを付加するか。trueで偶数。falseで奇数。
void picoPioUartTx_program_putc(unsigned char data, bool even_parity) {
    uint32_t byte = (uint32_t)data;
    uint8_t parity = 0;
    for (int i = 0; i < 8; i++) {
        parity ^= byte & 0x1;
        byte >>= 1;
    }
    byte = (uint32_t)data;
    if (parity) {
        if (even_parity) {
            byte |= 0x100;  // 偶数になるようにパリティを付加します
        }
    } else {
        if (!even_parity) {
            byte |= 0x100;  // 奇数になるようにパリティを付加します
        }
    }
    pio_sm_put_blocking(pio, sm_tx, (uint32_t)byte);  // TX FIFOへputします
}

//UART(シリアル通信)で受信する関数
//
//
//even_parity : 偶数か奇数のどちらになるようにパリティを付加されているか。trueで偶数。falseで奇数。
//parity_check : パリティビットの結果。正しいならtrue。違ったらfalseで、例外処理を用意する。データがなくてもfalseになる。
unsigned char picoPioUartRx_program_getc(bool even_parity,bool* parity_check) {
    // if(pio_sm_is_rx_fifo_empty(pio, sm_rx)){
        // *parity_check = false;
        // return 0;
    // }else{
     while (pio_sm_is_rx_fifo_empty(pio, sm_rx)) {
        tight_loop_contents();
        //printf("待機中");
     }
     
    uint32_t c32 = pio_sm_get(pio, sm_rx);

    c32 >>= 23;
    //パリティビットの検証をする
    bool real_parity = (c32 & 0x100) != 0;
    uint8_t byte = c32 & 0xff;

    uint8_t pcheck = 0;
    for (int i = 0; i < 8; i++) {
        pcheck ^= byte & 0x1;
        byte >>= 1;
    }

    *parity_check = (pcheck == real_parity);

    return (unsigned char)(c32 & 0xff);
    // }
}

void picoPioUartRx_program_clear_buffer(){
    while (!pio_sm_is_rx_fifo_empty(pio, sm_rx)){
        uint32_t c32 = pio_sm_get(pio, sm_rx);
    }
}