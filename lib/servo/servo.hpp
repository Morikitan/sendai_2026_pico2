#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void ServoSetup();
void SetServoAngle(unsigned int gpio,int angle);
void DownArm();
void UpArm();
void CatchBall();

#ifdef __cplusplus
}
#endif