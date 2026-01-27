#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void StepperSetup();
void SetStepperSpeed(unsigned int slice_num,unsigned int gpio, float freq_hz);

#ifdef __cplusplus
}
#endif