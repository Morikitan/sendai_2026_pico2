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
    sleep_ms(500);
    SubToMainSetup();
    CurrentSensorSetup();
    ColorSensorSetup();
    SuctionSetup();
    TofSetup();
    GyroSetup();
    
    ServoSetup();
    
    //sleep_ms(1000);
    // SetSuctionMotorSpeed(250);
    printf("tof使用開始\n");
    while(true){
        // UseTof();
        // SetServoAngle(servo_arm_up_and_down_pin,60);
        // SetServoAngle(servo_left_claw_pin,90);
        // SetServoAngle(servo_right_claw_pin,90);
        sleep_ms(101);
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
        //printf("object: %d\n",UseColorSensor());
        // sleep_ms(100);
    }
}