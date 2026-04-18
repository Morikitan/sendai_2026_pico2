#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void PinSetup();
void ColorLEDSetup();
float radian(float angle);
void UseColorLED(uint8_t red,uint8_t green,uint8_t blue);

#ifdef __cplusplus
}
#endif