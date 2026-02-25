#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define REG_CONTROL 0x00
#define REG_DATA_RED_H 0x03

void ColorSensorSetup();
bool wait_data_ready(uint32_t timeout_ms); 
bool write_register(uint8_t reg, uint8_t value);
bool read_registers(uint8_t start_reg, uint8_t *dest, size_t len);
int UseColorSensor();
void CurrentSensorSetup();
void UseCurrentSensor(unsigned int input);

#ifdef __cplusplus
}
#endif