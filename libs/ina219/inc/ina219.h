#ifndef INA219_H
#define INA219_H

#include "stdbool.h"

#include "stm32g4xx_hal.h"

void ina219_init(I2C_HandleTypeDef*);
bool ina219_is_initialized(void);
float ina219_ShuntVoltage_mV(void);
float ina219_ShuntVoltage_V(void);
float ina219_BusVoltage_mV(void);
float ina219_BusVoltage_V(void);
float ina219_Current_mA(void);
float ina219_Power_mW(void);

#endif