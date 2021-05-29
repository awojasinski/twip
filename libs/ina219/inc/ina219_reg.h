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

#define INA219_MASK_RST                     1 << 15
#define INA219_MASK_BRNG                    1 << 13
#define INA219_MASK_PGA                     3 << 11
#define INA219_MASK_BADC                    15 << 7
#define INA219_MASK_SADC                    15 << 3
#define INA219_MASK_MODE                    7

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



#endif