#include "display/display.hpp"
#include "gyro/gyro.hpp"
#include "motor/motor.hpp"
#include "main_to_sub/main_to_sub.hpp"
#include "other_sensor/other_sensor.hpp"
#include "others/others.hpp"
#include "servo/servo.hpp"
#include "tof/tof.hpp"
#include "config.hpp"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"
#include <stdio.h>

int main(){
    stdio_init_all();
    sleep_ms(500);
    SubToMainSetup();
    CurrentSensorSetup();
    ColorSensorSetup();
    SuctionSetup();
    TofSetup();
    GyroSetup();
    
    ServoSetup();
    
    //sleep_ms(1000);
    // SetSuctionMotorSpeed(250);
    printf("tof使用開始\n");
    while(true){
        sleep_ms(1);
        if(isGyroReset){
            uint32_t status = save_and_disable_interrupts();
            GyroSetup();
            restore_interrupts(status);
            isGyroReset = false;
        }
    }
}