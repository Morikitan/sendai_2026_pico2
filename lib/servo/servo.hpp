#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void ServoSetup();
void SetServoAngle(unsigned int gpio,int angle);

#ifdef __cplusplus
}
#endif