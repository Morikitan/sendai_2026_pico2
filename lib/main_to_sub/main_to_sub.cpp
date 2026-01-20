#include "display.hpp"
#include "main_to_sub.hpp"
#include "../config.hpp"
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include <stdio.h>

void MainToSubSetup(){
    gpio_init(main_to_line_RX_pin);
    gpio_init(main_to_line_TX_pin);
    gpio_set_function(main_to_line_RX_pin,GPIO_FUNC_UART);
    gpio_set_function(main_to_line_TX_pin,GPIO_FUNC_UART);
}

void SubToMainSetup(){
    gpio_init(line_to_main_RX_pin);
    gpio_init(line_to_main_TX_pin);
    gpio_set_function(line_to_main_RX_pin,GPIO_FUNC_UART);
    gpio_set_function(line_to_main_TX_pin,GPIO_FUNC_UART);
}