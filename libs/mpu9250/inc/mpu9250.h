#ifndef MPU9250_H
#define MPU9250_H

#include <stdbool.h>

#include "stm32g4xx_hal.h"

#include "mpu9250_reg.h"

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t addr;
    bool initialized;
} mpu9250_sensor_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} mpu9250_imu_raw_data_t;

typedef struct {
    mpu9250_imu_raw_data_t accel;
    mpu9250_imu_raw_data_t gyro;
    int16_t temp;
    mpu9250_imu_raw_data_t mag;
} mpu9250_raw_data_t;

typedef struct {
    float x;
    float y;
    float z;
} mpu9250_imu_data_t;

typedef struct {
    mpu9250_imu_data_t accel;
    mpu9250_imu_data_t gyro;
    float temp;
    mpu9250_imu_data_t mag;
} mpu9250_data_t;

extern mpu9250_sensor_t hmpu9250;
extern mpu9250_raw_data_t volatile hmpu9250_raw_data;
extern mpu9250_data_t volatile hmpu9250_data;

//void mpu9250_init(void);
bool mpu9250_is_initialized(void);
void mpu9250_data_scaled(mpu9250_data_t*);
void mpu9250_data_raw(mpu9250_raw_data_t*);
void mpu9250_accel_raw(mpu9250_imu_raw_data_t*);
void mpu9250_accel_scaled(mpu9250_imu_data_t*);
void mpu9250_gyro_raw(mpu9250_imu_raw_data_t*);
void mpu9250_gyro_scaled(mpu9250_imu_data_t*);
int16_t mpu9250_temp_raw(void);
float mpu9250_temp_celsius(void);
void mpu9250_mag_raw(mpu9250_imu_raw_data_t*);
void mag9250_mag_scaled(mpu9250_imu_data_t*);
void mpu9250_test_acc(void);
void mpu9250_test_gyro(void);
void mpu9250_test_mag(void);
void EXTI4_IRQHandler(void);

#endif