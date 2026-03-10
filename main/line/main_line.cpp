#include "line/line.hpp"
#include "main_to_line/main_to_line.hpp"
#include "config.hpp"
#include "pico/stdlib.h"
#include "u8g2.h"
#include <stdio.h>

int main(){
    stdio_init_all();
    LineSetup();
    LineToMainSetup();
    while(true){
        UseLineSensor();
        // PutDataFromLineToMain();
        sleep_ms(10);
    }
}