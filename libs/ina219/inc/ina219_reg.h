#ifndef INA219_REG_H
#define INA219_REG_H

#define INA219_I2C_ADDRESS1                 0x40 //(A0=GND, A1=GND)
#define INA219_I2C_ADDRESS2                 0x41 //(A0=VCC, A1=GND)
#define INA219_I2C_ADDRESS3                 0x44 //(A0=GND, A1=VCC)
#define INA219_I2C_ADDRESS4                 0x45 //(A0=VCC, A1=VCC)

#define DFROBOT_INA219_READ                 0x01

/*Register Configuration*/
#define INA219_REG_CONFIG                   0x00
#define INA219_CONFIG_RESET                 0x8000
#define INA219_CONFIG_BUSVOLTAGERANGE_MASK  0x2000

/*Shunt Voltage Register*/
#define INA219_REG_SHUNTVOLTAGE             0x01
/*Bus Voltage Register*/
#define INA219_REG_BUSVOLTAGE               0x02
/*Power Register*/
#define INA219_REG_POWER                    0x03
/*Current Register*/
#define INA219_REG_CURRENT                  0x04
/*Register Calibration*/
#define INA219_REG_CALIBRATION              0x05

// Values for voltage range
typedef enum ina219_busvol_config_t
{
    INA219_BUSVOLTAGE_RANGE_16V, // 0-16V Range
    INA219_BUSVOLTAGE_RANGE_32V, // 0-32V Range
} ina219_busvol_config_t;

// Values for gain bits
typedef enum ina219_pga_config_t
{
    INA219_PGA_BITS_1, // gain 1, 40mV Range
    INA219_PGA_BITS_2, // gain 2, 80mV Range
    INA219_PGA_BITS_4, // gain 4, 160mV Range
    INA219_PGA_BITS_8, // gain 8, 320mV Range
} ina219_pga_config_t;

typedef enum ina219_adc_res_config_t
{
    INA219_ADC_9BIT,  // 9-bit sample
    INA219_ADC_10BIT, // 10-bit sample
    INA219_ADC_11BIT, // 11-bit sample
    INA219_ADC_12BIT, // 12-bit sample

} ina219_adc_res_config_t;

// Values for ADC resolution and samples
typedef enum ina219_adc_samples_config_t
{
    INA219_ADC_SAMPLE_1_532U,      // 1 sample
    INA219_ADC_SAMPLE_2_1060US,    // 2 samples averaged together
    INA219_ADC_SAMPLE_4_2130US,    // 4 samples averaged together
    INA219_ADC_SAMPLE_8_4260US,    // 8 samples averaged together
    INA219_ADC_SAMPLE_16_8510US,   // 16 samples averaged together
    INA219_ADC_SAMPLE_32_17MS,     // 32 samples averaged together
    INA219_ADC_SAMPLE_64_34MS,     // 64 samples averaged together
    INA219_ADC_SAMPLE_128_69MS,    // 128 samples averaged together

} ina219_adc_samples_config_t;

// Values for operating mode
typedef enum ina219_mode_config_t
{
    INA219_PowerDown,
    INA219_SVolTrig,
    INA219_BVolTrig,
    INA219_SAndBVolTrig,
    INA219_AdcOff,
    INA219_SVolCon,
    INA219_BVolCon,
    INA219_SAndBVolCon
} ina219_mode_config_t;

#endif