#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "pico/stdlib.h"

void CameraSetup();
void UseCamera();
bool uart_read_with_timeout(uart_inst_t *uart, uint8_t *buffer, size_t length, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif