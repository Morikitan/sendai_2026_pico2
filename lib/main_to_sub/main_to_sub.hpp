#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "pico/stdlib.h"

void MainToSubSetup();
void SubToMainSetup();
void SubCallBack();
void SetServoAngleFromMain(unsigned int gpio,int angle);
void TrashfromBasketFromMain(int object);
void SetServoOffFromMain(unsigned int gpio);
void SetSuctionMotorSpeedFromMain(uint8_t duty);
void ResetGyroFromMain(int correctionAngle);
void GetGyroAngleFromSub();
void GetDistanceFromSub();
void GetColorFromSub();
void GetCurrentFromSub();
void TurnOnColorLEDFromMain();

#ifdef __cplusplus
}
#endif