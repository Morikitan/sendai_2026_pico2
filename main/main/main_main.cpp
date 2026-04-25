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
    SetServoAngleFromMain(servo_arm_up_and_down_pin,60);
    SetServoAngleFromMain(servo_left_claw_pin,90);
    SetServoAngleFromMain(servo_right_claw_pin,90);
    // SetSuctionMotorSpeedFromMain(75);//30%
    UseColorLED(0,255,0);
    while(true){
        
        if(gpio_get(tactile_switch_pin1) == true){
            CatchBall();
            sleep_ms(3000);
            CatchVerticalCan();
        }else if(gpio_get(tactile_switch_pin2) == true){
            //CatchPetBottle();
            SetServoAngleFromMain(servo_arm_up_and_down_pin,155);
            sleep_ms(1000);
            SetServoAngleFromMain(servo_arm_up_and_down_pin,168);
            sleep_ms(250);
            SetServoAngleFromMain(servo_left_claw_pin,40);
            SetServoAngleFromMain(servo_right_claw_pin,140);
        }else if(gpio_get(tactile_switch_pin3) == true){
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