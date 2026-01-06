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