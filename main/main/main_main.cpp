#include "display/display.hpp"
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
    // PinSetup();

    while(true){
        // PrintDisplayMode();
        //WriteTextOnDisplay(5,15,"<DeltaTime>",12,true,true);
        //GetDataFromLineToMain();
        sleep_ms(1000);
        MainMotorState(100,100);
        /*printf("Scanning...\n");

        for (int addr = 1; addr < 127; addr++) {
            uint8_t buf;
            int result = i2c_read_blocking(display_i2c, addr, &buf, 1, false);

            if (result >= 0) {
                printf("Found device at 0x%02X\n", addr);
            }
        }

        printf("Done\n\n");
        sleep_ms(3000);*/
    }
}