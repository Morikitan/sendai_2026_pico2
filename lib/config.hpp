#pragma once

#include <string>

extern std::string SerialWatch;

//camera
#define camera_uart uart1
#define camera_TX_pin 4
#define camera_RX_pin 5

//display
#define display_i2c i2c0
#define display_SDA_pin 16
#define display_SCL_pin 17

//タッチセンサー
#define touch_sensor_front_left_pin 0
#define touch_sensor_front_right_pin 1
#define touch_sensor_back_left_pin 14
#define touch_sensor_back_right_pin 15

//タクトスイッチ
#define tactile_switch_pin1 19
#define tactile_switch_pin2 20
#define tactile_switch_pin3 21