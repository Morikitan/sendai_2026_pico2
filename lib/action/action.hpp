#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void LineTraceSetup();
void BuzzerRing(int times, int length);
void RotationToAngle(int target_angle);
void MainMove();
void LineTrace();
void CatchPetBottle();
void CatchBall();
void CatchCan();
void ThrowCan();
void UseAllSensor();

#ifdef __cplusplus
}
#endif