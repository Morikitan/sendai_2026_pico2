#include "display/display.hpp"
#include "gyro/gyro.hpp"
#include "main_to_sub/main_to_sub.hpp"
#include "other_sensor/other_sensor.hpp"
#include "others/others.hpp"
#include "servo/servo.hpp"
#include "tof/tof.hpp"
#include "config.hpp"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include <stdio.h>

int main(){
    stdio_init_all();
    // GyroSetup();
    ServoSetup();
    sleep_ms(1000);
    while(true){
        // UseGyroSensor();
        //CatchBall();
        // SetServoAngle(servo_sentor_basket_pin,60);
        CatchCan(true);
        sleep_ms(1000);
        CatchCan(false);
        sleep_ms(1000);
        ThrowCan();
        sleep_ms(5000);
    }
}