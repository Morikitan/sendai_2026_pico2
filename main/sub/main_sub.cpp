#include "gyro/gyro.hpp"
#include "servo/servo.hpp"
#include "config.hpp"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include <stdio.h>

int main(){
    stdio_init_all();
    GyroSetup();
    ServoSetup();
    while(true){
        UseGyroSensor();
        CatchBall();
    }
}