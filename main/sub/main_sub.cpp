#include "display/display.hpp"
#include "servo/servo.hpp"
#include "config.hpp"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "u8g2.h"
#include <stdio.h>

int main(){
    stdio_init_all();
    ServoSetup();
    while(true){
        SetServoAngle(servo_left_claw_pin,0);
        SetServoAngle(servo_right_claw_pin,0);
        sleep_ms(2000);
        SetServoAngle(servo_left_claw_pin,90);
        SetServoAngle(servo_right_claw_pin,90);
        sleep_ms(2000);
        SetServoAngle(servo_left_claw_pin,180);
        SetServoAngle(servo_right_claw_pin,180);
        sleep_ms(2000);
    }
}