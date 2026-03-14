#include "display/display.hpp"
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
    DisplaySetup();
    MainToLineSetup();
    MainToSubSetup();
    PinSetup();
    EncoderSetup();
    SetStepperSleep();
    sleep_ms(2000);
    
    // SetServoAngleFromMain(servo_arm_up_and_down_pin,60);
    SetSuctionMotorSpeedFromMain(75);//30%
    while(true){
        sleep_ms(100);
        PrintDisplayMode();
        GetGyroAngleFromSub();
        GetDistanceFromSub();
        GetColorFromSub();
        GetCurrentFromSub();
        
        /*if(frontLineSensor[0] == true && frontLineSensor[2] == false){
            //左に曲がる
            MainMotorState(-125,250);
        }else if(frontLineSensor[0] == false && frontLineSensor[2] == true){
            //右に曲がる
            MainMotorState(250,-125);
        }else{
            MainMotorState(250,250);
        }*/
        SendBufferToDisplay();
    }
}