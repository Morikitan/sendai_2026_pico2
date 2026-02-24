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
    i2c_init(color_sensor_i2c, 400000);
    gpio_init(color_sensor_LED_pin);
    gpio_init(color_sensor_SCL_pin);
    gpio_init(color_sensor_SDA_pin);
    gpio_set_dir(color_sensor_LED_pin,GPIO_OUT);
    gpio_set_function(color_sensor_SCL_pin,GPIO_FUNC_I2C);
    gpio_set_function(color_sensor_SDA_pin,GPIO_FUNC_I2C);
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


void UseColorSensor(){
    uint8_t data[8] = {0};
    //8bitのデータの読み出し
    if (!read_registers(REG_DATA_RED_H, data, sizeof(data))) {
        printf("I2C read error\n");
        sleep_ms(500);
    }
    //RGB値と補正値の計算
    uint16_t red    = (uint16_t(data[0]) << 8) | data[1];
    uint16_t green  = (uint16_t(data[2]) << 8) | data[3];
    uint16_t blue   = (uint16_t(data[4]) << 8) | data[5];
    uint16_t corr   = (uint16_t(data[6]) << 8) | data[7];
    //補正値を差し引いたRGB値の計算
    int r = int(red)   - int(corr);
    int g = int(green) - int(corr);
    int b = int(blue)  - int(corr);
    printf("Red: %d\n", r);
    printf("Green: %d \n", g);
    printf("Blue: %d \n", b);
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