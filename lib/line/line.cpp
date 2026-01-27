#include "display.hpp"
#include "line.hpp"
#include "../config.hpp"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <stdio.h>

bool circleLineSensor[20];
bool frontLineSensor[3];

uint circle_line_sensor_pin_name[20] = 
{circle_line_sensor_pin1,circle_line_sensor_pin2,circle_line_sensor_pin3,circle_line_sensor_pin4,
circle_line_sensor_pin5,circle_line_sensor_pin6,circle_line_sensor_pin7,circle_line_sensor_pin8,
circle_line_sensor_pin9,circle_line_sensor_pin10,circle_line_sensor_pin11,circle_line_sensor_pin12,
circle_line_sensor_pin13,circle_line_sensor_pin14,circle_line_sensor_pin15,circle_line_sensor_pin16,
circle_line_sensor_pin17,circle_line_sensor_pin18,circle_line_sensor_pin19,circle_line_sensor_pin20};

void LineSetup(){
    for(int i = 0;i < 20;i++){
        gpio_init(circle_line_sensor_pin_name[i]);
        gpio_set_dir(circle_line_sensor_pin_name[i],GPIO_IN);
    }
    gpio_init(front_line_sensor_pinA);
    gpio_init(front_line_sensor_pinB);
    gpio_init(front_line_sensor_pinC);
    gpio_set_dir(front_line_sensor_pinA,GPIO_IN); 
    gpio_set_dir(front_line_sensor_pinB,GPIO_IN); 
    gpio_set_dir(front_line_sensor_pinC,GPIO_IN);
}

void UseLineSensor(){
    for(int i = 0;i < 20;i++){
        circleLineSensor[i] = gpio_get(circle_line_sensor_pin_name[i]);
    }
    frontLineSensor[0] = gpio_get(front_line_sensor_pinA);
    frontLineSensor[1] = gpio_get(front_line_sensor_pinB);
    frontLineSensor[2] = gpio_get(front_line_sensor_pinC);
}