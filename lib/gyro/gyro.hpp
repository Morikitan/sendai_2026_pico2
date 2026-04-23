#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void GyroSetup();
void UseGyroSensor();
void ResetGyro(int correctionAngle2);

#ifdef __cplusplus
}
#endif