#include "display/display.hpp"
#include "gyro/gyro.hpp"
#include "motor/motor.hpp"
#include "main_to_sub/main_to_sub.hpp"
#include "other_sensor/other_sensor.hpp"
#include "others/others.hpp"
#include "servo/servo.hpp"
#include "tof/tof.hpp"
#include "config.hpp"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "pico/stdlib.h"
#include <stdio.h>

int main(){
    stdio_init_all();
    CurrentSensorSetup();
    ColorSensorSetup();
    SuctionSetup();
    // GyroSetup();
    //ServoSetup();
    //sleep_ms(1000);
    //SetSuctionMotorSpeed(250);
    while(true){
        //adc_select_input(2);
        //int adc = adc_read();
        //printf("%d\n",adc);
        //sleep_ms(10);
        //UseGyroSensor();
        //CatchBall();
        //SetServoAngle(servo_sentor_basket_pin,60);
        //CatchCan(true);
        //sleep_ms(1000);
        //CatchCan(false);
        //sleep_ms(1000);
        //ThrowCan();
        //sleep_ms(5000);
        UseColorSensor();
    }
}