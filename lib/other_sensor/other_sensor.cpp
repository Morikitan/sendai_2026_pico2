#include "display.hpp"
#include "other_sensor.hpp"
#include "../config.hpp"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <stdio.h>

uint8_t color;
uint16_t current[3];
uint8_t currentBuffer[6];

void ColorSensorSetup(){
    i2c_init(color_sensor_i2c, 100000);
    gpio_set_function(color_sensor_SCL_pin, GPIO_FUNC_I2C);
    gpio_set_function(color_sensor_SDA_pin, GPIO_FUNC_I2C);
    gpio_init(color_sensor_LED_pin);
    gpio_set_dir(color_sensor_LED_pin, GPIO_OUT);
    gpio_put(color_sensor_LED_pin,1);
    // 固定時間モード / Lowゲイン / 22.4ms
    // bit7=0 bit6=0 bit3=0 bit2=0 bit1,0=10
    write_register(REG_CONTROL, 0x02);
    sleep_ms(50);
}

void EncoderSetup(){
    gpio_init(encoderA_pin);
    gpio_init(encoderB_pin);
    gpio_set_dir(encoderA_pin,GPIO_IN);
    gpio_set_dir(encoderB_pin,GPIO_IN);
    gpio_set_irq_enabled_with_callback(encoderA_pin,GPIO_IRQ_EDGE_FALL,true,&MainInterrupt);
}

bool wait_data_ready(uint32_t timeout_ms){
    absolute_time_t timeout = make_timeout_time_ms(timeout_ms);

    while (!time_reached(timeout)) {

        uint8_t ctrl;
        if (!read_registers(REG_CONTROL, &ctrl, 1))
            return false;

        // bit5 を確認
        if (ctrl & (1 << 5)) {
            return true;  // 測定完了
        }
    }

    return false; // タイムアウト
}

// レジスタへの書き込み
bool write_register(uint8_t reg, uint8_t value){
    uint8_t buf[2] = { reg, value };
    int ret = i2c_write_blocking(color_sensor_i2c, color_sensor_adr, buf, 2, false);
    return (ret == 2);
}

// レジスタからのデータの読み出し
bool read_registers(uint8_t start_reg, uint8_t *dest, size_t len){
    // Write the register address
    if (i2c_write_blocking(color_sensor_i2c, color_sensor_adr, &start_reg, 1, true) != 1){
        return false;
    }
    // Read data
    int ret = i2c_read_blocking(color_sensor_i2c, color_sensor_adr, dest, len, false);
    return (ret == int(len));
}

//カラセンを利用した物体の判定
//戻り値は　255:I2Cエラー,0:その他,1:赤ボール,2:青ボール,3:缶
int UseColorSensor(){
    //カラーセンサー用LEDの点灯
    //データの読み出し
    uint8_t data[8];
    if (!read_registers(REG_DATA_RED_H, data, 8)) {
        printf("I2C error\n");
        //エラー時はLEDを消灯
        // gpio_put(color_sensor_LED_pin,0);
        return 255;
    }
    uint16_t red    = (data[0] << 8) | data[1];
    uint16_t green  = (data[2] << 8) | data[3];
    uint16_t blue   = (data[4] << 8) | data[5];
    uint16_t corr   = (data[6] << 8) | data[7];
    //補正値の計算
    int r = red   > corr ? red   - corr : 0;
    int g = green > corr ? green - corr : 0;
    int b = blue  > corr ? blue  - corr : 0;
    //RGB値の確認用
    printf("R:%d G:%d B:%d\n", r, g, b);
    //objectは持っているものを表す変数　127:その他,1:赤ボール,2:青ボール,3:缶
    int object = 127;
    if(r > 2000){
        object = 1;
    }else if(b > 1750){
        object = 2;
    }else if(r > 1100 && g > 950){
        object = 3;
    }
    printf("Object:%d\n",object);
    //LEDの消灯
    // gpio_put(color_sensor_LED_pin,0);
    return object;
}

void CurrentSensorSetup(){
    adc_init();
    adc_gpio_init(current_sensor_left_pin);
    adc_gpio_init(current_sensor_right_pin);
    adc_gpio_init(current_sensor_DC_pin);
}

//input : 使うピンの設定
//        left  : 0
//        right : 1
//        DC    : 2 
void UseCurrentSensor(unsigned int input){
    adc_select_input(input);
    current[input] = adc_read();
    currentBuffer[input * 2] = (uint8_t)(current[input] >> 8);
    currentBuffer[input * 2 + 1] = (uint8_t)current[input];
    if(serialWatch == "cur"){
        printf("current%u : %u",input,current[input]);
    }
}

void MainInterrupt(uint gpio, uint32_t events){
    if(gpio == encoderA_pin){
        if (gpio_get(encoderB_pin) == true) {
            // 時計回り
            displayMode++;
        } else {
            // 反時計回り
            displayMode--;
        }

        // ループ処理（0〜MAX_displayMode）
        if (displayMode > 10) {
            displayMode = 1;
        } else if (displayMode <= 0) {
            displayMode = 10;
        }
    }
}