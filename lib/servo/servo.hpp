#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void ServoSetup();
void SetServoAngle(unsigned int gpio,int angle);
void SetServoOff(unsigned int gpio);
void ThrowCan();
void CatchCanFromSub();

#ifdef __cplusplus
}
#endif