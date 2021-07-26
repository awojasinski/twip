#include "ina219_reg.h"
#include "ina219.h"

#define DATA_TRANSFER_TIMEOUT 100

// Values for voltage range
typedef enum ina219_busvol_config_t {
    INA219_BUSVOLTAGE_RANGE_16V = 0x0000,    // 0-16V Range
    INA219_BUSVOLTAGE_RANGE_32V = 0x2000,    // 0-32V Range
} ina219_busvol_config_t;

// Values for gain bits
typedef enum ina219_pga_config_t {
    INA219_PGA_BITS_1 = 0x0000,     // gain 1, 40mV Range
    INA219_PGA_BITS_2 = 0x0800,     // gain 2, 80mV Range
    INA219_PGA_BITS_4 = 0x1000,     // gain 4, 160mV Range
    INA219_PGA_BITS_8 = 0x1800,     // gain 8, 320mV Range
} ina219_pga_config_t;

// Values for ADC resolution and samples
typedef enum ina219_adc_config_t
{
    INA219_ADC_SAMPLE_9_BIT_1S_84US = 0x0000,    // 1x9-bit shunt sample
    INA219_ADC_SAMPLE_10BIT_1S_148US = 0x0008,   // 1x10-bit shunt sample
    INA219_ADC_SAMPLE_11BIT_1S_276US = 0x0010,   // 1x11-bit shunt sample
    INA219_ADC_SAMPLE_12BIT_1S_532US = 0x0018,   // 1x12-bit shunt sample
    INA219_ADC_SAMPLE_12BIT_2S_1060US = 0x0048,  // 2x12-bit shunt sample averaged together
    INA219_ADC_SAMPLE_12BIT_4S_2130US = 0x0050,  // 4x12-bit shunt sample averaged together
    INA219_ADC_SAMPLE_12BIT_8S_4260US = 0x0058,  // 8x12-bit shunt sample averaged together
    INA219_ADC_SAMPLE_12BIT_16S_8510US = 0x0060, // 16x12-bit shunt sample averaged together
    INA219_ADC_SAMPLE_12BIT_32S_17MS = 0x0060,   // 32x12-bit shunt sample averaged together
    INA219_ADC_SAMPLE_12BIT_64S_34MS = 0x0060,   // 64x12-bit shunt sample averaged together
    INA219_ADC_SAMPLE_12BIT_128S_69MS = 0x0060,  // 128x12-bit shunt sample averaged together

} ina219_adc_config_t;

// Values for operating mode
typedef enum ina219_mode_config_t {
    INA219_PowerDown,
    INA219_SVolTrig,
    INA219_BVolTrig,
    INA219_SAndBVolTrig,
    INA219_AdcOff,
    INA219_SVolCon,
    INA219_BVolCon,
    INA219_SAndBVolCon
} ina219_mode_config_t;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t addr;
    bool initialized;

    uint32_t calibrationVal;

    uint32_t currentDivider_mA;
    float powerMultiplier_mW;
} ina219_sensor_t;

static inline HAL_StatusTypeDef sensor_i2c_write_bytes(uint16_t, uint8_t *, uint16_t);
static inline HAL_StatusTypeDef sensor_i2c_read_bytes(uint16_t, uint8_t *, uint16_t);

#define sensor_i2c_write_byte(addr, ptr) sensor_i2c_write_bytes(addr, ptr, 1)
#define sensor_i2c_read_byte(addr, ptr) sensor_i2c_read_bytes(addr, ptr, 1)

static void ina219_reset(void);
static void ina219_setMode(ina219_mode_config_t);
static void ina219_setBADC(ina219_adc_config_t);
static void ina219_setSADC(ina219_adc_config_t);
static void ina219_setPGA(ina219_pga_config_t);
static void ina219_setBRNG(ina219_busvol_config_t);

static void ina219_setConfiguration(ina219_busvol_config_t, ina219_pga_config_t, ina219_adc_config_t, ina219_adc_config_t, ina219_mode_config_t);

static void ina219_setCalibration(void);

static inline int16_t ina219_BusVoltage_raw(void);
static inline int16_t ina219_ShuntVoltage_raw(void);
static inline int16_t ina219_Current_raw(void);
static inline int16_t ina219_Power_raw(void);

static ina219_sensor_t hina219 = {NULL, 0, false, 0, 0, 0.0F};

void ina219_init(I2C_HandleTypeDef *i2c) {
    hina219.hi2c = i2c;
    hina219.addr = INA219_I2C_ADDRESS4 << 1;

    if (HAL_I2C_IsDeviceReady(hina219.hi2c, hina219.addr, 3, 5) == HAL_OK) {
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
        hina219.currentDivider_mA = 10; // Current LSB = 100uA per bit (1000/100 = 10)
        hina219.powerMultiplier_mW = 2; // Power LSB = 1mW per bit (2/1)

        ina219_setConfiguration(INA219_BUSVOLTAGE_RANGE_16V, INA219_PGA_BITS_4, INA219_ADC_SAMPLE_12BIT_1S_532US, INA219_ADC_SAMPLE_12BIT_1S_532US, INA219_SAndBVolCon);
        sensor_i2c_write_bytes(INA219_REG_CALIBRATION, (uint8_t *)&hina219.calibrationVal, 4);
        hina219.initialized = true;
    } else {
        hina219.initialized = false;
    }
}

bool ina219_is_initialized() {
    return hina219.initialized;
}

static void ina219_reset() {
    uint16_t data = INA219_CONFIG_RESET;
    sensor_i2c_write_bytes(INA219_REG_CONFIG, (uint8_t*)&data, 2);
}

float ina219_ShuntVoltage_mV() {
    int16_t value = ina219_ShuntVoltage_raw();
    return (float) value * 0.01F;
}

float ina219_ShuntVoltage_V() {
    int16_t value = ina219_ShuntVoltage_raw();
    return (float)value * 0.00001F;
}

float ina219_BusVoltage_mV() {
    int16_t value = ina219_BusVoltage_raw();
    return (float)(value >> 2);
}

float ina219_BusVoltage_V() {
    int16_t value = ina219_BusVoltage_raw();
    return (float)(value >> 2) * 0.001F;
}

float ina219_Current_mA() {
    return (float)ina219_Current_raw();
}

float ina219_Power_mW() {
    return (float)ina219_Power_raw() * 20;
}

static void ina219_setMode(ina219_mode_config_t mode) {
    uint16_t config;
    sensor_i2c_read_bytes(INA219_REG_CONFIG, (uint8_t *)&config, 2);
    config &= (uint16_t) ~(INA219_MASK_MODE);
    config = (uint16_t) (config | mode);
    sensor_i2c_write_bytes(INA219_REG_CONFIG, (uint8_t *)&config, 2);
}

static void ina219_setBADC(ina219_adc_config_t adc_resolution) {
    uint16_t config;
    sensor_i2c_read_bytes(INA219_REG_CONFIG, (uint8_t *)&config, 2);
    config &= (uint16_t) ~(INA219_MASK_PGA);
    config = (uint16_t) (config | adc_resolution);
    sensor_i2c_write_bytes(INA219_REG_CONFIG, (uint8_t *)&config, 2);
}

static void ina219_setSADC(ina219_adc_config_t adc_resolution) {
    uint16_t config;
    sensor_i2c_read_bytes(INA219_REG_CONFIG, (uint8_t *)&config, 2);
    config &= (uint16_t) ~(INA219_MASK_BADC);
    config = (uint16_t) (config | adc_resolution);
    sensor_i2c_write_bytes(INA219_REG_CONFIG, (uint8_t *)&config, 2);
}

static void ina219_setPGA(ina219_pga_config_t pga_resolution) {
    uint16_t config;
    sensor_i2c_read_bytes(INA219_REG_CONFIG, (uint8_t*)&config, 2);
    config &= (uint16_t) ~(INA219_MASK_PGA);
    config = (uint16_t) (config | pga_resolution);
    sensor_i2c_write_bytes(INA219_REG_CONFIG, (uint8_t*)&config, 2);
}

static void ina219_setBRNG(ina219_busvol_config_t voltage_range) {
    uint16_t config;
    sensor_i2c_read_bytes(INA219_REG_CONFIG, (uint8_t*)&config, 2);
    config &= (uint16_t) ~(INA219_MASK_PGA);
    config = (uint16_t) (config | voltage_range);
    sensor_i2c_write_bytes(INA219_REG_CONFIG, (uint8_t*)&config, 2);
}

static void ina219_setConfiguration(ina219_busvol_config_t busVoltageRange, ina219_pga_config_t pgaGain, ina219_adc_config_t busADCrange, ina219_adc_config_t shuntADCrange, ina219_mode_config_t sensorMode) {
    uint16_t config = (uint16_t) (busVoltageRange | pgaGain | busADCrange | shuntADCrange | sensorMode);
    sensor_i2c_write_bytes(INA219_REG_CONFIG, (uint8_t*)&config, 2);
}

static inline int16_t ina219_BusVoltage_raw() {
    int16_t data;
    sensor_i2c_read_bytes(INA219_REG_BUSVOLTAGE, (uint8_t*)&data, 2);
    return data;
}

static inline int16_t ina219_ShuntVoltage_raw() {
    int16_t data;
    sensor_i2c_read_bytes(INA219_REG_SHUNTVOLTAGE, (uint8_t*)&data, 2);
    return data;
}

static inline int16_t ina219_Current_raw() {
    int16_t data;
    sensor_i2c_read_bytes(INA219_REG_CURRENT, (uint8_t*)&data, 2);
    return data;
}

static inline int16_t ina219_Power_raw() {
    int16_t data;
    sensor_i2c_read_bytes(INA219_REG_POWER, (uint8_t*)&data, 2);
    return data;
}

static inline HAL_StatusTypeDef sensor_i2c_write_bytes(uint16_t MemAddres, uint8_t *pData, uint16_t size)
{
    return HAL_I2C_Mem_Write(hina219.hi2c, hina219.addr, MemAddres, 1, pData, size, DATA_TRANSFER_TIMEOUT);
}

static inline HAL_StatusTypeDef sensor_i2c_read_bytes(uint16_t MemAddres, uint8_t *pData, uint16_t size)
{
    return HAL_I2C_Mem_Read(hina219.hi2c, hina219.addr, MemAddres, 1, pData, size, DATA_TRANSFER_TIMEOUT);
}