#include "display/display.hpp"
#include "motor/motor.hpp"
#include "others/others.hpp"
#include "config.hpp"
#include "pico/stdlib.h"
#include "u8g2.h"
#include "hardware/i2c.h"
#include <stdio.h>

//あ

int main(){
    stdio_init_all();
    // StepperSetup();
    sleep_ms(3000);
    DisplaySetup();
    sleep_ms(200);
    
    // PinSetup();

    while(true){
        /*WriteTextOnDisplay(10,20,"Hello World!",12,true,true);
        // MainMotorState(300,300);
        printf("Hello World!\n");
        sleep_ms(1000);*/
        printf("Scanning...\n");

        for (int addr = 1; addr < 127; addr++) {
            uint8_t buf;
            int result = i2c_read_blocking(display_i2c, addr, &buf, 1, false);

            if (result >= 0) {
                printf("Found device at 0x%02X\n", addr);
            }
        }

        printf("Done\n\n");
        sleep_ms(3000);
    }
}