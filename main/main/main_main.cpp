#include "display/display.hpp"
#include "action/action.hpp"
#include "camera/camera.hpp"
#include "gyro/gyro.hpp"
#include "line/line.hpp"
#include "main_to_line/main_to_line.hpp"
#include "main_to_sub/main_to_sub.hpp"
#include "motor/motor.hpp"
#include "other_sensor/other_sensor.hpp"
#include "others/others.hpp"
#include "tof/tof.hpp"
#include "config.hpp"
#include "pico/stdlib.h"
#include "u8g2.h"
#include "hardware/i2c.h"
#include <stdio.h>

int main(){
    stdio_init_all();
    sleep_ms(1000);
    StepperSetup();
    CameraSetup();
    DisplaySetup();
    MainToLineSetup();
    MainToSubSetup();
    PinSetup();
    EncoderSetup();
    SetStepperSleep();
    LineTraceSetup();
    ColorLEDSetup();
    sleep_ms(2000);
    SetServoAngleFromMain(servo_arm_up_and_down_pin,50);
    SetServoAngleFromMain(servo_arm_left_and_right_pin,90);
    SetServoAngleFromMain(servo_left_claw_pin,90);
    SetServoAngleFromMain(servo_right_claw_pin,90);
    // SetSuctionMotorSpeedFromMain(75);//30%
    UseColorLED(0,255,0);
    gpio_put(buzzer_pin,true);
    sleep_ms(500);
    gpio_put(buzzer_pin,false);
    while(true){
        if(gpio_get(tactile_switch_pin1) == true){
            task = 8;
            CatchBall(true);
        }else if(gpio_get(tactile_switch_pin2) == true){
            //最初に自由ボールを入れる場合の処理
        }else if(gpio_get(tactile_switch_pin3) == true){
            allFirstTime = time_us_32();
            SetStepperON();
            sleep_ms(500);
            while(true){
                PrintDisplayMode();
                MainMove();
                SendBufferToDisplay();
            }
        }
        PrintDisplayMode();
        UseAllSensor();
        float value = GetCircleLineVector(20,true,true);
        SendBufferToDisplay();
    }
}