#include "display.hpp"
#include "tof.hpp"
#include "../config.hpp"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "VL53L0X.h"
#include <stdio.h>

VL53L0X sensor(tof_i2c,VL53L0X_DEFAULT_ADDRESS);
unsigned short int distance;

void TofSetup(){
    gpio_init(tof_reset_pin);
    gpio_init(tof_SCL_pin);
    gpio_init(tof_SDA_pin);
    gpio_set_dir(tof_reset_pin,GPIO_OUT);
    gpio_set_function(tof_SCL_pin,GPIO_FUNC_I2C);
    gpio_set_function(tof_SDA_pin,GPIO_FUNC_I2C);

    i2c_init(tof_i2c,400000);
    sensor.init();
    sensor.setTimeout(500);
    sensor.startContinuous();
}

void UseTof(){
    distance = sensor.readRangeContinuousMillimeters();
    if (sensor.timeoutOccurred()){
        printf("Measurement timed out\n");
    }else{
        printf("Distance: %u mm\n", distance);
    }
}