#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void ColorSensorSetup();
void UseColorSensor();
void CurrentSensorSetup();
void UseCurrentSensor(unsigned int input);

#ifdef __cplusplus
}
#endif