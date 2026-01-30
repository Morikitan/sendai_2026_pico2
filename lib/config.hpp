#pragma once

#include <string>
#include "pico/stdlib.h"

//重要な変数・配列
extern std::string SerialWatch;

//camera
#define camera_uart uart1
#define camera_TX_pin 4 //メインマイコン
#define camera_RX_pin 5 //メインマイコン

//display
#define display_i2c i2c0
#define display_SDA_pin 16 //メインマイコン
#define display_SCL_pin 17 //メインマイコン
#define display_reset_pin 18 //メインマイコン

#define isUseDisplay true
#define DisplayBufferSize 200
extern char DisplayBuffer[DisplayBufferSize];
extern int DisplayMode;

//encorer
#define encoderA_pin 11 //メインマイコン
#define encoderB_pin 10 //メインマイコン

//gyro
#define gyro_i2c i2c1
#define gyro_SDA_pin 18 //サブマイコン
#define gyro_SCL_pin 19 //サブマイコン
#define gyro_reset_pin 22 //サブマイコン
extern float AngleX;
extern unsigned char gyroBuffer[2];

//line
extern bool circleLineSensor[20];
extern bool frontLineSensor[3];
#define main_to_line_TX_pin 14 //メインマイコン
#define main_to_line_RX_pin 15 //メインマイコン
#define line_to_main_TX_pin 12 //ラインマイコン
#define line_to_main_RX_pin 13 //ラインマイコン
#define circle_line_sensor_pin1 16 //ラインマイコン
#define circle_line_sensor_pin2 17 //ラインマイコン
#define circle_line_sensor_pin3 18 //ラインマイコン
#define circle_line_sensor_pin4 19 //ラインマイコン
#define circle_line_sensor_pin5 20 //ラインマイコン
#define circle_line_sensor_pin6 21 //ラインマイコン
#define circle_line_sensor_pin7 22 //ラインマイコン
#define circle_line_sensor_pin8 26 //ラインマイコン
#define circle_line_sensor_pin9 27 //ラインマイコン
#define circle_line_sensor_pin10 28 //ラインマイコン
#define circle_line_sensor_pin11 0 //ラインマイコン
#define circle_line_sensor_pin12 1 //ラインマイコン
#define circle_line_sensor_pin13 5 //ラインマイコン
#define circle_line_sensor_pin14 6 //ラインマイコン
#define circle_line_sensor_pin15 7 //ラインマイコン
#define circle_line_sensor_pin16 8 //ラインマイコン
#define circle_line_sensor_pin17 9 //ラインマイコン
#define circle_line_sensor_pin18 10 //ラインマイコン
#define circle_line_sensor_pin19 11 //ラインマイコン
#define circle_line_sensor_pin20 14 //ラインマイコン
#define front_line_sensor_pinA 2 //ラインマイコン
#define front_line_sensor_pinB 3 //ラインマイコン
#define front_line_sensor_pinC 4 //ラインマイコン

//motor (ステッパー)
#define stepper_reset_pin 6 //メインマイコン
#define stepper_sleep_pin 7 //メインマイコン
#define stepper_right_clock_pin 2 //メインマイコン
#define stepper_right_direction_pin 3 //メインマイコン
#define stepper_left_clock_pin 0 //メインマイコン
#define stepper_left_direction_pin 1 //メインマイコン

//motor (吸引)
#define motor_suction_pin 21 //サブマイコン

//tactile switch (タクトスイッチ)
#define tactile_switch_pin1 19 //メインマイコン
#define tactile_switch_pin2 20 //メインマイコン
#define tactile_switch_pin3 21 //メインマイコン

//tof
#define tof_i2c i2c0 //
#define tof_SDA_pin 16 //サブマイコン
#define tof_SCL_pin 17 //サブマイコン
#define tof_reset_pin 20 //サブマイコン
extern unsigned short int distance; //mm単位

//touch sensor
#define touch_sensor_front_left_pin 27 //メインマイコン
#define touch_sensor_front_right_pin 26 //メインマイコン
#define touch_sensor_back_right_pin 8 //メインマイコン
#define touch_sensor_back_left_pin 9 //メインマイコン

//servo
#define servo_left_basket_pin 2 //サブマイコン
#define servo_sentor_basket_pin 4 //サブマイコン
#define servo_right_basket_pin 6 //サブマイコン
#define servo_left_claw_pin 8 //サブマイコン
#define servo_right_claw_pin 10 //サブマイコン
#define servo_arm_left_and_right_pin 12 //サブマイコン
#define servo_arm_up_and_down_pin 14 //サブマイコン

//sub
#define main_to_sub_uart uart0
#define main_to_sub_TX_pin 12 //メインマイコン
#define main_to_sub_RX_pin 13 //メインマイコン
#define sub_to_main_uart uart0
#define sub_to_main_TX_pin 1 //サブマイコン
#define sub_to_main_RX_pin 0 //サブマイコン

//other sensors
#define current_sensor_left_pin 26  //サブマイコン
#define current_sensor_right_pin 27  //サブマイコン
#define current_sensor_DC_pin 28 //サブマイコン
#define color_sensor_i2c i2c0
#define color_sensor_SDA_pin 16 //サブマイコン
#define color_sensor_SCL_pin 17 //サブマイコン
#define color_sensor_LED_pin 13 //サブマイコン
extern uint8_t color;

//others
#define buzzer_pin 22 //メインマイコン
#define color_LED_pin 28 //メインマイコン