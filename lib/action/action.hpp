#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void LineTraceSetup();
void BuzzerRing(int times, int length);
void RotationToAngle(int target_angle);
void MainMove();
// void LineTrace();
void NewLineTrace();
void BackLineTrace();
void BackToLine();
void StraightLineTrace(int angle);
int RotationToObject();
void CatchPetBottle();
void CatchBall();
void CatchCan();
void TrashfromBasket(int object);
void UseAllSensor();
void PassTheSpace();
float GetCircleLineVector(int number,bool isFrontLine,bool isGetJuji);
void OnWall();

#ifdef __cplusplus
}
#endif