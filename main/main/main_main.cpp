#include "motor/motor.hpp"
#include "others/others.hpp"
#include "config.hpp"
#include "pico/stdlib.h"
#include "u8g2.h"
#include <stdio.h>

//あ

int main(){
    stdio_init_all();
    StepperSetup();
    PinSetup();
    while(true){
        //if(gpio_get(tactile_switch_pin1) == true){
            MainMotorState(300,300);
        //}
    }
}