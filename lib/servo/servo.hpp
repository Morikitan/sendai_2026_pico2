#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void ServoSetup();
void SetServoAngle(unsigned int gpio,int angle);
void DownArm();
void UpArm();
void SetServoOff(unsigned int gpio);
void CatchBall();
void CatchCan(bool isShake);
void ThrowCan();
void CatchPetBottle();

#ifdef __cplusplus
}
#endif