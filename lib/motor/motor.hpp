#pragma once

#include "pico/stdlib.h"

#ifdef __cplusplus
extern "C" {
#endif

void StepperSetup();
void SetStepperSpeed(unsigned int slice_num,unsigned int gpio, float freq_hz);
void MainMotorState(int speed1,int speed2);
void SuctionSetup();
void SetSuctionMotorSpeed(uint duty);

#ifdef __cplusplus
}
#endif