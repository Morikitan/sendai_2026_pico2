#include "line/line.hpp"
#include "main_to_line/main_to_line.hpp"
#include "config.hpp"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "u8g2.h"
#include <stdio.h>

int main(){
    stdio_init_all();
    sleep_ms(2000);
    LineSetup();
    LineToMainSetup();
    printf("start\n");
    while(true){
        // UseLineSensor();
        // circleLineSensor[1] = true;
        // circleLineSensor[4] = true;
        // circleLineSensor[5] = true;
        // circleLineSensor[7] = true;
        // circleLineSensor[12] = true;
        // circleLineSensor[15] = true;
        // frontLineSensor[1] = true;
        PutDataFromLineToMain();
        // sleep_ms(1);
    }
}