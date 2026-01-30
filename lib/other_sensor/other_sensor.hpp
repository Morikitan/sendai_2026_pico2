#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void ColorLEDSetup();
void UseColorSensor();
void CurrentSensorSetup();
void UseCurrentSensor(unsigned int input);

#ifdef __cplusplus
}
#endif