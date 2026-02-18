#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void MainToSubSetup();
void SubToMainSetup();
void SubCallBack();
void SetServoAngleFromMain(unsigned int gpio,int angle);
void SetSuctionMotorFromMain(int duty);
void GetGyroAngleFromSub();
void GetDistanceFromSub();
void GetColorFromSub();
void GetCurrentFromSub();

#ifdef __cplusplus
}
#endif