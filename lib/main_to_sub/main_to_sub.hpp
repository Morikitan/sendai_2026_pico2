#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "pico/stdlib.h"

void MainToSubSetup();
void SubToMainSetup();
void SubCallBack();
void SetServoAngleFromMain(unsigned int gpio,int angle);
void SetSuctionMotorSpeedFromMain(uint8_t duty);
void GetGyroAngleFromSub();
void GetDistanceFromSub();
void GetColorFromSub();
void GetCurrentFromSub();

#ifdef __cplusplus
}
#endif