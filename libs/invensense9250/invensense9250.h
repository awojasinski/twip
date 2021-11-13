#ifndef INVENSENSE_H
#define INVENSENSE_H

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "eMPL_outputs.h"
#include "mltypes.h"
#include "mpu.h"
#include "log.h"
#include "packet.h"

#define log_i MPL_LOGI
#define log_e MPL_LOGE

#define DEG_TO_RAD 0.0174532925199 ///< multiply to convert degrees to radians
#define RAD_TO_DEG 57.295779513    ///< multiply to convert radians to degrees
#define MS2_TO_G 0.10197162129     ///< multiply to convert m/s^2 to G
#define G_TO_MS2 9.80665           ///< multiply to convert G to m/s^2, standard gravity definition

/* Private typedef -----------------------------------------------------------*/
/* Data read from MPL. */
#define PRINT_ACCEL (0x01)
#define PRINT_GYRO (0x02)
#define PRINT_QUAT (0x04)
#define PRINT_COMPASS (0x08)
#define PRINT_EULER (0x10)
#define PRINT_ROT_MAT (0x20)
#define PRINT_HEADING (0x40)
#define PRINT_PEDO (0x80)
#define PRINT_LINEAR_ACCEL (0x100)
#define PRINT_GRAVITY_VECTOR (0x200)

#define ACCEL_ON (0x01)
#define GYRO_ON (0x02)
#define COMPASS_ON (0x04)

#define MOTION (0)
#define NO_MOTION (1)

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ (200)

#define FLASH_MEM_START ((void *)0x1800)

#define PEDO_READ_MS (1000)
#define TEMP_READ_MS (500)
#define COMPASS_READ_MS (100)

struct rx_s
{
    unsigned char header[3];
    unsigned char cmd;
};
struct hal_s
{
    unsigned char lp_accel_mode;
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned char motion_int_mode;
    unsigned long no_dmp_hz;
    unsigned long next_pedo_ms;
    unsigned long next_temp_ms;
    unsigned long next_compass_ms;
    unsigned int report;
    unsigned short dmp_features;
    struct rx_s rx;
};

void invensense_interrupt_handler(void);
int inv_set_callback(void (*func)(void));

/* Functions declatarion starts here */
void mpu9250_init(void);

void mpu9250_backend_init(void);
void mpu9250_backend_config(unsigned short *features);

int calibrate_accel(void);
int calibrate_gyro(void);
int calibrate_mag(void);

void run_self_test(void);

#endif
