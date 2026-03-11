#include "display/display.hpp"
#include "line/line.hpp"
#include "motor/motor.hpp"
#include "others/others.hpp"
#include "main_to_line/main_to_line.hpp"
#include "config.hpp"
#include "pico/stdlib.h"
#include "u8g2.h"
#include "hardware/i2c.h"
#include <stdio.h>

int main(){
    stdio_init_all();
    StepperSetup();
    DisplaySetup();
    sleep_ms(2000);
    MainToLineSetup();
    PinSetup();

    while(true){
        PrintDisplayMode();
        GetDataFromLineToMain();
        if(frontLineSensor[0] == true && frontLineSensor[2] == false){
            //左に曲がる
            MainMotorState(-125,250);
        }else if(frontLineSensor[0] == false && frontLineSensor[2] == true){
            //右に曲がる
            MainMotorState(250,-125);
        }else{
            MainMotorState(250,250);
        }
    }
}