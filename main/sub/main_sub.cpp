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
#include "pico/stdlib.h"
#include <stdio.h>

int main(){
    stdio_init_all();
    CurrentSensorSetup();
    // GyroSetup();
    //ServoSetup();
    //sleep_ms(1000);
    gpio_init(motor_suction_pin);
    gpio_set_dir(motor_suction_pin,GPIO_OUT);
    gpio_put(motor_suction_pin,1);
    while(true){
        gpio_put(motor_suction_pin,1);
        sleep_ms(5);
        gpio_put(motor_suction_pin,0);
        sleep_ms(15);
        adc_select_input(2);
        printf("%u\n",adc_read());
        // UseGyroSensor();
        //CatchBall();
        // SetServoAngle(servo_sentor_basket_pin,60);
        //CatchCan(true);
        //sleep_ms(1000);
        // CatchCan(false);
        //sleep_ms(1000);
        // ThrowCan();
        //sleep_ms(5000);
    }
}