#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void LineTraceSetup();
void BuzzerRing(int times, int length);
void RotationToAngle(int target_angle);
void MainMove();
void LineTrace();
void NewLineTrace();
void CatchPetBottle();
void CatchBall();
void CatchCan();
void ThrowCan();
void UseAllSensor();
float GetCircleLineVector(int number,bool isFrontLine);

#ifdef __cplusplus
}
#endif