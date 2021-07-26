#include "main.h"

#include "ina219_reg.h"
#include "ina219.h"

#define DATA_TRANSFER_TIMEOUT 100

typedef struct {
    bool initialized;
    I2C_HandleTypeDef *hi2c;
    uint8_t addr;
    uint16_t calibrationVal;
    uint32_t currentDivider_mA;
    float powerMultiplier_mW;
} ina219_sensor_t;

static ina219_sensor_t hina219 = {false, NULL, 0, 0, 0, 0.0F};

static inline void i2c_write_reg(uint16_t, uint16_t);
static inline int16_t i2c_read_reg(uint16_t);

static void ina219_reset(void);
static void ina219_setMode(ina219_mode_config_t);
static void ina219_setBADC(ina219_adc_res_config_t, ina219_adc_samples_config_t);
static void ina219_setSADC(ina219_adc_res_config_t, ina219_adc_samples_config_t);
static void ina219_setPGA(ina219_pga_config_t);
static void ina219_setBRNG(ina219_busvol_config_t);

void ina219_init(I2C_HandleTypeDef *i2c) {
    hina219.hi2c = i2c;
    hina219.addr = INA219_I2C_ADDRESS4 << 1;

    if (HAL_I2C_IsDeviceReady(hina219.hi2c, hina219.addr, 3, 100) == HAL_OK) {
        // VBUS_MAX = 16V    (12V operating voltage)
        // VSHUNT_MAX = 0.16 (possible 0.32 0.16 0.08 0.04)
        // RSHUNT = 0.1

        // 1. Determine max possible current
        // MaxPossible_I = VSHUNT_MAX / RSHUNT
        // MaxPossible_I = 0.16 / 0.1 = 1.6A

        // 2. Determine max expected current
        // MaxExpected_I = 1.8A (Stall current)

        // LSB - Least Significant Bit
        // 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
        // MinimumLSB = MaxExpected_I/32767
        // MinimumLSB = 1.8 / 32767 = 0.000055 A (55uA per bit)
        // MaximumLSB = MaxExpected_I/4096
        // MaximumLSB = 1.8 /4096 = 0.000439 A (439uA per bit)

        // 4. Choose an LSB between the min and max values
        // CurrentLSB = 0.0001 (100uA per bit)

        // 5. Compute the calibration register
        // Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
        // Cal = trunc (0.04096 / (0.0001 * 0.1)) = 4096
        hina219.calibrationVal = 4096;

        // 6. Calculate the power LSB
        // PowerLSB = 20 * CurrentLSB
        // PowerLSB = 20 * 0.0001 = 0.002 (2mW per bit)

        // 7. Compute the maximum current and shunt voltage values before overflow
        // Max_Current = Current_LSB * 32767
        // Max_Current = 0.0001 * 32767 = 3.27 A

        // If Max_Current > Max_Possible_I then
        //    Max_Current_Before_Overflow = MaxPossible_I
        // Else
        //    Max_Current_Before_Overflow = Max_Current
        // End If

        // Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
        // Max_ShuntVoltage = 1.6 * 0.1 = 0.16 V

        // If Max_ShuntVoltage >= VSHUNT_MAX
        //    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
        // Else
        //    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
        // End If

        // 8. Compute the Maximum Power
        // MaximumPower = Max_Current_Before_Overflow * VBUS_MAX

        // Set multipliers to convert raw current/power values
        //hina219.currentDivider_mA = 10; // Current LSB = 100uA per bit (1000/100 = 10)
        //hina219.powerMultiplier_mW = 2; // Power LSB = 1mW per bit (2/1)

        ina219_setBRNG(INA219_BUSVOLTAGE_RANGE_32V);
        ina219_setPGA(INA219_PGA_BITS_8);
        ina219_setBADC(INA219_ADC_12BIT, INA219_ADC_SAMPLE_8_4260US);
        ina219_setSADC(INA219_ADC_12BIT, INA219_ADC_SAMPLE_8_4260US);
        ina219_setMode(INA219_SAndBVolCon);

        i2c_write_reg(INA219_REG_CALIBRATION, hina219.calibrationVal);
        hina219.initialized = true;
    } else {
        hina219.initialized = false;
    }
}

float ina219_BusVoltage_V() {
    return (float)((i2c_read_reg(INA219_REG_BUSVOLTAGE) >> 1) * 0.001F);
}

float ina219_ShuntVoltage_mV() {
    return (float)(i2c_read_reg(INA219_REG_SHUNTVOLTAGE) * 0.01);
}

float ina219_Current_mA() {
    return (float)(i2c_read_reg(INA219_REG_CURRENT));
}

float ina219_Power_mW() {
    return (float)(i2c_read_reg(INA219_REG_POWER) * 20);
}

static void ina219_setMode(ina219_mode_config_t mode) {
    int16_t config = i2c_read_reg(INA219_REG_CONFIG);
    config &= ~((int16_t)0x07);
    config = (int16_t)(config | mode);
    i2c_write_reg(INA219_REG_CONFIG, (uint16_t)config);
}

static void ina219_setBADC(ina219_adc_res_config_t resolution, ina219_adc_samples_config_t samples) {
    int16_t config = i2c_read_reg(INA219_REG_CONFIG);
    int16_t value;
    if (resolution < INA219_ADC_12BIT && samples > INA219_ADC_SAMPLE_1_532U) {
        return;
    }
    if (resolution < INA219_ADC_12BIT) {
        value = resolution;
    } else {
        value = 0x08 | samples;
    }
    config &= ~((int16_t)0x0f << 7);
    config = (int16_t)(config | value << 7);
    i2c_write_reg(INA219_REG_CONFIG, (uint16_t)config);
}

static void ina219_setSADC(ina219_adc_res_config_t resolution, ina219_adc_samples_config_t samples) {
    int16_t config = i2c_read_reg(INA219_REG_CONFIG);
    int16_t value;
    if (resolution < INA219_ADC_12BIT && samples > INA219_ADC_SAMPLE_1_532U) {
        return;
    }
    if (resolution < INA219_ADC_12BIT) {
        value = resolution;
    } else {
        value = 0x08 | samples;
    }
    config &= ~((int16_t)0x0f << 3);
    config = (int16_t)(config | value << 3);
    i2c_write_reg(INA219_REG_CONFIG, (uint16_t)config);
}

static void ina219_setPGA(ina219_pga_config_t pga_resolution) {
    int16_t config = i2c_read_reg(INA219_REG_CONFIG);
    config &= ~((int16_t)0x03 << 11);
    config = (int16_t)(config | pga_resolution << 11);
    i2c_write_reg(INA219_REG_CONFIG, (uint16_t)config);
}

static void ina219_setBRNG(ina219_busvol_config_t voltage_range) {
    int16_t config = i2c_read_reg(INA219_REG_CONFIG);
    config &= ~((uint16_t)1 << 13);
    config = (int16_t)(config | voltage_range << 13);
    i2c_write_reg(INA219_REG_CONFIG, (uint16_t)config);
}

bool ina219_is_initialized() {
    return hina219.initialized;
}

static void ina219_reset() {
    i2c_write_reg(INA219_REG_CONFIG, INA219_CONFIG_RESET);
}

static inline void i2c_write_reg(uint16_t MemAddres, uint16_t data) {
    uint8_t buf[2] = {(uint8_t)(data >> 8), (uint8_t)(data & 0xFF)};
    HAL_I2C_Mem_Write(hina219.hi2c, hina219.addr, MemAddres, 1, buf, 2, DATA_TRANSFER_TIMEOUT);
}

static inline int16_t i2c_read_reg(uint16_t MemAddres) {
    uint8_t buf[2];
    HAL_I2C_Mem_Read(hina219.hi2c, hina219.addr, MemAddres, 1, buf, 2, DATA_TRANSFER_TIMEOUT);
    return (int16_t)(buf[0]<<8 | buf[1]);
}