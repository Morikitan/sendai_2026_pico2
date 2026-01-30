#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void MainToSubSetup();
void SubToMainSetup();
void SubCallBack();
void SetServoAngleFromMain(unsigned int gpio,int angle);
void GetGyroAngleFromSub();
void GetDistanceFromSub();
void GetColorFromSub();

#ifdef __cplusplus
}
#endif