#pragma once

#include <string>

//重要な変数・配列
extern std::string SerialWatch;

//camera
#define camera_uart uart1
#define camera_TX_pin 4
#define camera_RX_pin 5

//display
#define display_i2c i2c0
#define display_SDA_pin 16
#define display_SCL_pin 17
#define display_reset_pin 18

#define isUseDisplay true
#define DisplayBufferSize 200
extern char DisplayBuffer[DisplayBufferSize];
extern int DisplayMode;

//gyro
#define gyro_i2c i2c1
#define gyro_SDA_pin 18
#define gyro_SCL_pin 19
extern float AngleX;

//line
extern bool circleLineSensor[20];
#define main_to_line_TX_pin 14 //メインマイコン側
#define main_to_line_RX_pin 15 //メインマイコン側
#define line_to_main_TX_pin 12 //ラインマイコン側
#define line_to_main_RX_pin 13 //ラインマイコン側
#define circle_line_sensor_pin1 0
#define circle_line_sensor_pin2 1
#define circle_line_sensor_pin3 2
#define circle_line_sensor_pin4 3
#define circle_line_sensor_pin5 4
#define circle_line_sensor_pin6 5
#define circle_line_sensor_pin7 6
#define circle_line_sensor_pin8 7
#define circle_line_sensor_pin9 8
#define circle_line_sensor_pin10 9
#define circle_line_sensor_pin11 10
#define circle_line_sensor_pin12 11
#define circle_line_sensor_pin13 14
#define circle_line_sensor_pin14 15
#define circle_line_sensor_pin15 16
#define circle_line_sensor_pin16 17
#define circle_line_sensor_pin17 18
#define circle_line_sensor_pin18 19
#define circle_line_sensor_pin19 20
#define circle_line_sensor_pin20 21
#define front_line_sensor_pinA 22
#define front_line_sensor_pinB 26
#define front_line_sensor_pinC 27

//motor (ステッパー)
#define stepper_reset_pin 6
#define stepper_sleep_pin 7
#define stepper_right_clock_pin 2
#define stepper_right_direction_pin 3
#define stepper_left_clock_pin 0
#define stepper_left_direction_pin 1

//tactile switch (タクトスイッチ)
#define tactile_switch_pin1 19
#define tactile_switch_pin2 20
#define tactile_switch_pin3 21

//touch sensor
#define touch_sensor_front_left_pin 0
#define touch_sensor_front_right_pin 1
#define touch_sensor_back_right_pin 8
#define touch_sensor_back_left_pin 9

//servo
#define servo_left_basket_pin 2
#define servo_sentor_basket_pin 4
#define servo_right_basket_pin 6
#define servo_left_claw_pin 8
#define servo_right_claw_pin 10
#define servo_arm_left_and_right_pin 12
#define servo_arm_up_and_down_pin 14

//sub
#define main_to_sub_uart uart0
#define main_to_sub_TX_pin 12
#define main_to_sub_RX_pin 13
#define sub_to_main_TX_pin 1
#define sub_to_main_RX_pin 0

//tof
#define tof_SDA_pin 16
#define tof_SCL_pin 17

//other_sensor


//others
#define buzzer_pin 22
#define color_LED_pin 28