#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void StepperSetup();
void SetStepperSpeed(unsigned int slice_num,unsigned int gpio, float freq_hz);
void MainMotorState(int speed1,int speed2);

#ifdef __cplusplus
}
#endif