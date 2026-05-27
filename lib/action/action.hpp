#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void LineTraceSetup();
void MainMove();
void BackLineTrace();
void BackToLine();
void BuzzerRing(int times, int length);
void CatchBall(bool isSuction);
void CatchCan();
void CatchPetBottle();
void DaikeiKasoku(int speed,int angle);
void DaikeiKasokuLoop(int time,int speed,int angle);
void NewLineTrace();
void OnWall(int angle);
void PassTheSpace();
void PMove(int left,int right);
void RotationToAngle(int target_angle);
int RotationToObject();
void StraightLineTrace(int angle,int speed2);
void TrashfromBasket(int object);
void TrashHorizonCan();
void UseAllSensor();
float GetCircleLineVector(int number,bool isFrontLine,bool isGetJuji);
#ifdef __cplusplus
}
#endif