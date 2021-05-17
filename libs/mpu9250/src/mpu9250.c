#include <math.h>
#include "i2c.h"

#include "mpu9250.h"
#include "cli.h"

#define DATA_TRANSFER_TIMEOUT 100
#define GRAVITATIONAL_ACCEL 9.81F

typedef enum mpu9250_clock_t {
    MPU9250_CLK_INTERNAL,
    MPU9250_CLK_PLL = 1,
    MPU9250_CLK_RESET = 7,
} mpu9250_clock_t;

typedef enum mpu9250_accel_config_t {
    MPU9250_ACCEL_2G,
    MPU9250_ACCEL_4G,
    MPU9250_ACCEL_8G,
    MPU9250_ACCEL_16G,
} mpu9250_accel_config_t;

typedef enum mpu9250_gyro_config_t {
    MPU9250_GYRO_250,
    MPU9250_GYRO_500,
    MPU9250_GYRO_1000,
    MPU9250_GYRO_2000,
} mpu9250_gyro_config_t;

typedef enum mpu9250_dlpf_config_t{
    MPU9250_DLPF_250_HZ,
    MPU9250_DLPF_184_HZ,
    MPU9250_DLPF_92_HZ,
    MPU9250_DLPF_41_HZ,
    MPU9250_DLPF_20_HZ,
    MPU9250_DLPF_10_HZ,
    MPU9250_DLPF_5_HZ,
} mpu9250_dlpf_config_t;

static inline HAL_StatusTypeDef sensor_i2c_write_bytes(uint16_t, uint8_t*, uint16_t);
static inline HAL_StatusTypeDef sensor_i2c_read_bytes(uint16_t, uint8_t*, uint16_t);

#define sensor_i2c_write_byte(addr, ptr) sensor_i2c_write_bytes(addr, ptr, 1)
#define sensor_i2c_read_byte(addr, ptr) sensor_i2c_read_bytes(addr, ptr, 1)

static inline void mpu9250_reset(void);
static inline void mpu9250_sleep(bool);
static inline void mpu9250_clock_source(mpu9250_clock_t);
static inline void mpu9250_accel_scale(mpu9250_accel_config_t);
static inline void mpu9250_gyro_scale(mpu9250_gyro_config_t);
static inline void mpu9250_dlpf(mpu9250_dlpf_config_t);
static inline uint8_t mpu9250_get_sensor_address(void);
static inline uint8_t mpu9250_get_int_status(void);

mpu9250_sensor_t hmpu9250;
mpu9250_raw_data_t volatile hmpu9250_raw_data;
mpu9250_data_t volatile hmpu9250_data;

float gyro_scale, accel_scale;

void mpu9250_init() {
    hmpu9250.hi2c = &hi2c1;
    hmpu9250.addr = MPU9250_I2C_ADDRESS<<1;

    MX_I2C1_Init();
    mpu9250_reset();
    mpu9250_sleep(false);

    if (mpu9250_get_sensor_address() != 0x71) {
        Error_Handler(__FILE__, __LINE__);
    } else {
        hmpu9250.initialized = true;
    }
    mpu9250_clock_source(MPU9250_CLK_PLL);
    mpu9250_dlpf(MPU9250_DLPF_20_HZ);
    mpu9250_accel_scale(MPU9250_ACCEL_4G);
    mpu9250_gyro_scale(MPU9250_GYRO_250
    );
}

void mpu9250_data_raw(mpu9250_raw_data_t *sensor) {
    mpu9250_accel_raw(&(sensor->accel));
    mpu9250_gyro_raw(&(sensor->gyro));
    sensor->temp = mpu9250_temp_raw();
}

void mpu9250_data_scaled(mpu9250_data_t *sensor) {
    mpu9250_accel_scaled(&(sensor->accel));
    mpu9250_gyro_scaled(&(sensor->gyro));
    sensor->temp = mpu9250_temp_celsius();
}

void mpu9250_accel_raw(mpu9250_imu_raw_data_t *sensor){
    uint8_t tmp[6];
    sensor_i2c_read_bytes(MPU9250_ACCEL_XOUT_H, tmp, 6);

    sensor->x = (int16_t)((((int16_t)tmp[0]) << 8) | tmp[1]);
    sensor->y = (int16_t)((((int16_t)tmp[2]) << 8) | tmp[3]);
    sensor->z = (int16_t)((((int16_t)tmp[4]) << 8) | tmp[5]);
}

void mpu9250_accel_scaled(mpu9250_imu_data_t *sensor) {
    mpu9250_imu_raw_data_t tmp;
    mpu9250_accel_raw(&tmp);

    sensor->x = (float)tmp.x * accel_scale;
    sensor->y = (float)tmp.y * accel_scale;
    sensor->z = (float)tmp.z * accel_scale;
}

void mpu9250_gyro_raw(mpu9250_imu_raw_data_t *sensor) {
    uint8_t tmp[6];
    sensor_i2c_read_bytes(MPU9250_GYRO_XOUT_H, tmp, 6);

    sensor->x = (int16_t)((((int16_t)tmp[0]) << 8) | tmp[1]);
    sensor->y = (int16_t)((((int16_t)tmp[2]) << 8) | tmp[3]);
    sensor->z = (int16_t)((((int16_t)tmp[4]) << 8) | tmp[5]);
}
void mpu9250_gyro_scaled(mpu9250_imu_data_t *sensor) {
    mpu9250_imu_raw_data_t tmp;
    mpu9250_gyro_raw(&tmp);

    sensor->x = (float)tmp.x * gyro_scale;
    sensor->y = (float)tmp.y * gyro_scale;
    sensor->z = (float)tmp.z * gyro_scale;
}

int16_t mpu9250_temp_raw() {
    uint8_t tmp[2];
    sensor_i2c_read_bytes(MPU9250_TEMP_OUT_H, tmp, 2);
    return (int16_t)((((int16_t)tmp[0]) << 8) | tmp[1]);
}

float mpu9250_temp_celsius() {
    int16_t temp;
    temp = mpu9250_temp_raw();
    return (float)temp / 340 + 36.53F;
}

void mpu9250_mag_raw(mpu9250_imu_raw_data_t *sensor) {
    UNUSED(sensor);
}

void mag9250_mag_scaled(mpu9250_imu_data_t *sensor) {
    UNUSED(sensor);
}

static inline void mpu9250_reset() {
    uint8_t tmp;
    sensor_i2c_read_byte(MPU9250_PWR_MGMT_1, &tmp);
    tmp &= (uint8_t) ~(1 << MPU9250_PWR_RESET_BIT);
    sensor_i2c_write_byte(MPU9250_PWR_MGMT_1, &tmp);
}

static inline void mpu9250_sleep(bool enable) {
    uint8_t tmp;
    sensor_i2c_read_byte(MPU9250_PWR_MGMT_1, &tmp);
    tmp &= (uint8_t) ~(1 << MPU9250_PWR_SLEEP_BIT);
    tmp |= (uint8_t) ((enable & 0x1) << MPU9250_PWR_SLEEP_BIT);
    sensor_i2c_write_byte(MPU9250_PWR_MGMT_1, &tmp);
}

static inline void mpu9250_clock_source(mpu9250_clock_t clk) {
    uint8_t tmp;
    sensor_i2c_read_byte(MPU9250_PWR_MGMT_1, &tmp);
    tmp &= 0xF8;
    tmp = (uint8_t)(tmp | (clk & 0x7));
    sensor_i2c_write_byte(MPU9250_PWR_MGMT_1, &tmp);
}

static inline void mpu9250_accel_scale(mpu9250_accel_config_t scale) {
    uint8_t tmp;
    sensor_i2c_read_byte(MPU9250_ACCEL_CONFIG, &tmp);
    tmp &= 0xE7;
    tmp = (uint8_t)(tmp | ((scale & 0x7) << 3));
    sensor_i2c_write_byte(MPU9250_ACCEL_CONFIG, &tmp);

    switch (scale) {
        case MPU9250_ACCEL_2G:
            accel_scale = (2 * GRAVITATIONAL_ACCEL) / (0xFFFF >> 1);
            break;
        case MPU9250_ACCEL_4G:
            accel_scale = (4 * GRAVITATIONAL_ACCEL) / (0xFFFF >> 1);
            break;
        case MPU9250_ACCEL_8G:
            accel_scale = (8 * GRAVITATIONAL_ACCEL) / (0xFFFF >> 1);
            break;
        case MPU9250_ACCEL_16G:
            accel_scale = (16 * GRAVITATIONAL_ACCEL) / (0xFFFF >> 1);
            break;
        default:
            break;
    }
}

static inline void mpu9250_gyro_scale(mpu9250_gyro_config_t scale) {
    uint8_t tmp;
    sensor_i2c_read_byte(MPU9250_GYRO_CONFIG, &tmp);
    tmp &= 0xE7;
    tmp = (uint8_t)(tmp | ((scale & 0x7) << 3));
    sensor_i2c_write_byte(MPU9250_GYRO_CONFIG, &tmp);

    switch (scale) {
        case MPU9250_GYRO_250:
            gyro_scale = (250) / (0xFFFF >> 1);
            break;
        case MPU9250_GYRO_500:
            gyro_scale = (500) / (0xFFFF >> 1);
            break;
        case MPU9250_GYRO_1000:
            gyro_scale = (1000) / (0xFFFF >> 1);
            break;
        case MPU9250_GYRO_2000:
            gyro_scale = (2000) / (0xFFFF >> 1);
            break;
        default:
            break;
    }
}

static inline void mpu9250_dlpf(mpu9250_dlpf_config_t filter) {
    uint8_t tmp;
    sensor_i2c_read_byte(MPU9250_CONFIG, &tmp);
    tmp &= 0xF8;
    tmp = (uint8_t)(tmp | (filter & 0x7));
    sensor_i2c_write_byte(MPU9250_CONFIG, &tmp);
}

static inline uint8_t mpu9250_get_sensor_address() {
    uint8_t tmp;
    sensor_i2c_read_byte(MPU9250_FIFO_WHO_AM_I, &tmp);
    return tmp;
}

static inline uint8_t mpu9250_get_int_status() {
    uint8_t tmp;
    sensor_i2c_read_byte(MPU9250_INT_STATUS, &tmp);
    return tmp;
}

bool mpu9250_is_initialized() {
    return hmpu9250.initialized;
}

static inline HAL_StatusTypeDef sensor_i2c_write_bytes(uint16_t MemAddres, uint8_t *pData, uint16_t size) {
    return HAL_I2C_Mem_Write(hmpu9250.hi2c, hmpu9250.addr, MemAddres, 1, pData, size, DATA_TRANSFER_TIMEOUT);
}

static inline HAL_StatusTypeDef sensor_i2c_read_bytes(uint16_t MemAddres, uint8_t *pData, uint16_t size) {
    return HAL_I2C_Mem_Read(hmpu9250.hi2c, hmpu9250.addr, MemAddres, 1, pData, size, DATA_TRANSFER_TIMEOUT);
}

void EXTI4_IRQHandler(void)
{
    uint8_t interrupt = mpu9250_get_int_status();
    UNUSED(interrupt);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
}