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
    sleep_ms(200);
    MainToLineSetup();
    PinSetup();

    while(true){
        PrintDisplayMode();
        // WriteTextOnDisplay(5,15,"<DeltaTime>",12,true,true);
        sleep_ms(1000);
        GetDataFromLineToMain();
        // sleep_ms(1000);
        // MainMotorState(100,100);
        sleep_ms(100);
    }
}