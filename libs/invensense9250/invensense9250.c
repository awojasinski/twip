#include <stdbool.h>
#include <stdio.h>
#include "main.h"
#include <math.h>

#include "cli.h"
#include "invensense9250.h"
#include "app_fatfs.h"

#include "regs.h"
#include "vector.h"
#include "matrix.h"
#include "algebra.h"

#define SLV_ADDR 0x68

#define GYRO_CAL_THRESH 50   // std dev below which to consider still
#define ACCEL_CAL_THRESH 100 // std dev below which to consider still
#define GYRO_OFFSET_THRESH 500

unsigned char accel_fsr, new_temp = 0;
unsigned short gyro_rate, gyro_fsr;
unsigned long sensor_timestamp;
static unsigned long timestamp;

static int was_last_steady = 0;

static double mag_factory_adjust[3];
static double mag_offsets[3];
static double mag_scales[3];
static double accel_lengths[3];

static struct hal_s hal = {0};

unsigned char *mpl_key = (unsigned char *)"eMPL 5.1";

typedef struct mpu_data_t
{
    /** @name base sensor readings in real units */
    ///@{
    double accel[3]; ///< accelerometer (XYZ) in units of m/s^2
    double gyro[3];  ///< gyroscope (XYZ) in units of degrees/s
    double mag[3];   ///< magnetometer (XYZ) in units of uT
    double temp;     ///< thermometer, in units of degrees Celsius
    ///@}

    /** @name 16 bit raw adc readings and conversion rates*/
    ///@{
    int16_t raw_gyro[3];  ///< raw gyroscope (XYZ)from 16-bit ADC
    int16_t raw_accel[3]; ///< raw accelerometer (XYZ) from 16-bit ADC
    double accel_to_ms2;  ///< conversion rate from raw accelerometer to m/s^2
    double gyro_to_degs;  ///< conversion rate from raw gyroscope to degrees/s
    ///@}

    /** @name DMP data */
    ///@{
    double dmp_quat[4];      ///< normalized quaternion from DMP based on ONLY Accel/Gyro
    double dmp_TaitBryan[3]; ///< Tait-Bryan angles (roll pitch yaw) in radians from DMP based on ONLY Accel/Gyro
    int tap_detected;        ///< set to 1 if there was a tap detect on the last dmp sample, reset to 0 on next sample
    int last_tap_direction;  ///< direction of last tap, 1-6 corresponding to X+ X- Y+ Y- Z+ Z-
    int last_tap_count;      ///< current counter of rapid consecutive taps
    ///@}

    /** @name fused DMP data filtered with magnetometer */
    ///@{
    double fused_quat[4];       ///< fused and normalized quaternion
    double fused_TaitBryan[3];  ///< fused Tait-Bryan angles (roll pitch yaw) in radians
    double compass_heading;     ///< fused heading filtered with gyro and accel data, same as Tait-Bryan yaw
    double compass_heading_raw; ///< unfiltered heading from magnetometer
                                ///@}
} mpu_data_t;

/* Platform-specific information. Kinda like a boardfile. */
struct platform_data_s
{
    signed char orientation[9];
};

/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from the
 * driver(s).
 * TODO: The following matrices refer to the configuration on internal test
 * boards at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
static struct platform_data_s gyro_pdata = {
    .orientation = {1, 0, 0,
                    0, 1, 0,
                    0, 0, 1}};

#if defined MPU9150 || defined MPU9250
static struct platform_data_s compass_pdata = {
    .orientation = {0, 1, 0,
                    1, 0, 0,
                    0, 0, -1}};
#define COMPASS_ENABLED 1
#elif defined AK8975_SECONDARY
static struct platform_data_s compass_pdata = {
    .orientation = {-1, 0, 0,
                    0, 1, 0,
                    0, 0, -1}};
#define COMPASS_ENABLED 1
#elif defined AK8963_SECONDARY
static struct platform_data_s compass_pdata = {
    .orientation = {-1, 0, 0,
                    0, -1, 0,
                    0, 0, 1}};
#define COMPASS_ENABLED 1
#endif

#ifdef COMPASS_ENABLED
unsigned short compass_fsr;
unsigned char new_compass = 0;
unsigned long compass_timestamp;
#endif

static int __write_gyro_cal_to_disk(int16_t *);
static int __write_mag_cal_to_disk(double *, double *);
static int __write_acc_cal_to_disk(double *, double *);

static int __load_mag_calibration(void);
static int __load_gyro_calibration(void);
static int __load_acc_calibration(void);

static int __reset_mpu(void);

void mpu9250_init(void)
{

    inv_error_t result = mpu_init(NULL);
    if (result)
    {
        log_e("Could not initialize gyro.");
        HAL_Delay(100);
        Error_Handler(__FILE__, __LINE__);
    }
    else
    {
        log_i("MPU9250 initialized");

        result = inv_init_mpl();
        if (result)
        {
            log_e("Could not initialize MPL.");
        }
        inv_enable_quaternion();
        //inv_enable_9x_sensor_fusion();

        inv_enable_fast_nomot();
        inv_enable_gyro_tc();

        inv_enable_in_use_auto_calibration();

#ifdef COMPASS_ENABLED
        /* Compass calibration algorithms. */
        inv_enable_vector_compass_cal();
        inv_enable_magnetic_disturbance();
#endif
        inv_enable_eMPL_outputs();

        result = inv_start_mpl();
        if (result == INV_ERROR_NOT_AUTHORIZED)
        {
            while (1)
            {
                MPL_LOGE("Not authorized.");
            }
        }
        if (result)
        {
            MPL_LOGE("Could not start the MPL.");
        }

#ifdef COMPASS_ENABLED
        mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
#else
        mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
#endif
        /* Push both gyro and accel data into the FIFO. */
        mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
        mpu_set_sample_rate(100);
#ifdef COMPASS_ENABLED
        /* The compass sampling rate can be less than the gyro/accel sampling rate.
       * Use this function for proper power management.
       */
        mpu_set_compass_sample_rate(1000 / COMPASS_READ_MS);
#endif
        /* Read back configuration in case it was set improperly. */
        mpu_set_accel_fsr(8);
        mpu_set_gyro_fsr(1000);

        mpu_get_sample_rate(&gyro_rate);
        mpu_get_gyro_fsr(&gyro_fsr);
        mpu_get_accel_fsr(&accel_fsr);
        mpu_set_lpf(48);
#ifdef COMPASS_ENABLED
        mpu_get_compass_fsr(&compass_fsr);
#endif
        /* Sync driver configuration with MPL. */
        /* Sample rate expected in microseconds. */
        inv_set_gyro_sample_rate(1000000L / gyro_rate);
        inv_set_accel_sample_rate(1000000L / gyro_rate);
        inv_set_quat_sample_rate(1000000L / gyro_rate);
#ifdef COMPASS_ENABLED
        /* The compass rate is independent of the gyro and accel rates. As long as
     * inv_set_compass_sample_rate is called with the correct value, the 9-axis
     * fusion algorithm's compass correction gain will work properly.
     */
        inv_set_compass_sample_rate(COMPASS_READ_MS * 1000L);
#endif
        /* Set chip-to-body orientation matrix.
     * Set hardware units to dps/g's/degrees scaling factor.
     */
        inv_set_gyro_orientation_and_scale(
            inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
            (long)gyro_fsr << 15);
        inv_set_accel_orientation_and_scale(
            inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
            (long)accel_fsr << 15);
#ifdef COMPASS_ENABLED
        inv_set_compass_orientation_and_scale(
            inv_orientation_matrix_to_scalar(compass_pdata.orientation),
            (long)compass_fsr << 15);
#endif
        /* Initialize HAL state variables. */
#ifdef COMPASS_ENABLED
        hal.sensors = ACCEL_ON | GYRO_ON | COMPASS_ON;
#else
        hal.sensors = ACCEL_ON | GYRO_ON;
#endif
        hal.dmp_on = 0;
        hal.report = 0;
        hal.rx.cmd = 0;
        hal.next_compass_ms = 0;
        hal.next_temp_ms = 0;

        /* Compass reads are handled by scheduler. */
        get_tick(&timestamp);
        mpu9250_backend_init();

        hal.dmp_features = DMP_FEATURE_6X_LP_QUAT |
                           DMP_FEATURE_SEND_RAW_ACCEL |
                           DMP_FEATURE_SEND_CAL_GYRO |
                           DMP_FEATURE_GYRO_CAL;
        //long accel_bias[] = {-12124160, 17506304, 23789568};
        //long gyro_bias[] = {1153116, -4075826, 1072953};

        //inv_set_accel_bias(accel_bias, 2);
        //inv_set_gyro_bias(gyro_bias, 2);

        mpu9250_backend_config(&hal.dmp_features);

        HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(EXTI4_IRQn);
    }
}

void mpu9250_backend_init()
{
    inv_error_t result = dmp_load_motion_driver_firmware();

    if (result != 0)
    {
        log_e("Could not load DMP firmware.");
    }
    else
    {
        hal.dmp_on = 1;
        MPL_LOGI("DMP enabled.");
    }
}

void mpu9250_backend_config(unsigned short *features)
{
    dmp_set_orientation(
        inv_orientation_matrix_to_scalar(gyro_pdata.orientation));
    dmp_enable_feature(hal.dmp_features);
    dmp_set_fifo_rate(100);
    mpu_set_dmp_state(1);
    //dmp_set_interrupt_mode(DMP_INT_CONTINUOUS);
    hal.dmp_on = 1;
}

static inline void gyro_data_ready_cb(void)
{
    hal.new_gyro = 1;

    unsigned long timestamp;

    get_tick(&timestamp);
#ifdef COMPASS_ENABLED
    if ((timestamp > hal.next_compass_ms) && !hal.lp_accel_mode &&
        hal.new_gyro && (hal.sensors & COMPASS_ON))
    {
        hal.next_compass_ms = timestamp + COMPASS_READ_MS;
        new_compass = 1;
    }
#endif
    /* Temperature data doesn't need to be read with every gyro sample.
	           * Let's make them timer-based like the compass reads.
	           */
    if (timestamp > hal.next_temp_ms)
    {
        hal.next_temp_ms = timestamp + TEMP_READ_MS;
        new_temp = 1;
    }
}

void run_self_test(void)
{
    int result;
    long gyro[3], accel[3];

#if defined(MPU6500) || defined(MPU9250)
    result = mpu_run_6500_self_test(gyro, accel, 1);
#elif defined(MPU6050) || defined(MPU9150)
    result = mpu_run_self_test(gyro, accel);
#endif
    if (result == 0x7)
    {
        MPL_LOGI("Passed!\n");
        MPL_LOGI("accel: %7.4f %7.4f %7.4f\n",
                 (float)accel[0] / 65536.f,
                 (float)accel[1] / 65536.f,
                 (float)accel[2] / 65536.f);
        MPL_LOGI("gyro: %7.4f %7.4f %7.4f\n",
                 (float)gyro[0] / 65536.f,
                 (float)gyro[1] / 65536.f,
                 (float)gyro[2] / 65536.f);
        /* Test passed. We can trust the gyro data here, so now we need to update calibrated data*/

#ifdef USE_CAL_HW_REGISTERS
        /*
         * This portion of the code uses the HW offset registers that are in the MPUxxxx devices
         * instead of pushing the cal data to the MPL software library
         */
        unsigned char i = 0;

        for (i = 0; i < 3; i++)
        {
            gyro[i] = (long)(gyro[i] * 32.8f); //convert to +-1000dps
            accel[i] *= 2048.f;                //convert to +-16G
            accel[i] = accel[i] >> 16;
            gyro[i] = (long)(gyro[i] >> 16);
        }

        mpu_set_gyro_bias_reg(gyro);

#if defined(MPU6500) || defined(MPU9250)
        mpu_set_accel_bias_6500_reg(accel);
#elif defined(MPU6050) || defined(MPU9150)
        mpu_set_accel_bias_6050_reg(accel);
#endif
#else
        /* Push the calibrated data to the MPL library.
         *
         * MPL expects biases in hardware units << 16, but self test returns
		 * biases in g's << 16.
		 */
        unsigned short accel_sens;
        float gyro_sens;

        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        inv_set_accel_bias(accel, 3);
        mpu_get_gyro_sens(&gyro_sens);
        gyro[0] = (long)((float)gyro[0] * gyro_sens);
        gyro[1] = (long)((float)gyro[1] * gyro_sens);
        gyro[2] = (long)((float)gyro[2] * gyro_sens);
        inv_set_gyro_bias(gyro, 3);
        MPL_LOGI("accel: %ld %ld %ld\n",
                 accel[0],
                 accel[1],
                 accel[2]);
        MPL_LOGI("gyro: %ld %ld %ld\n",
                 gyro[0],
                 gyro[1],
                 gyro[2]);
#endif
    }
    else
    {
        if (!(result & 0x1))
            MPL_LOGE("Gyro failed.\n");
        if (!(result & 0x2))
            MPL_LOGE("Accel failed.\n");
        if (!(result & 0x4))
            MPL_LOGE("Compass failed.\n");
    }
}

static int __reset_mpu(void)
{
    uint8_t data_send = MPU0250_H_RESET;
    if (Sensors_I2C_WriteRegister(SLV_ADDR, MPU9250_PWR_MGMT_1, 1, &data_send) == -1)
    {
        // wait and try again
        HAL_Delay(10);
        if (Sensors_I2C_WriteRegister(SLV_ADDR, MPU9250_PWR_MGMT_1, 1, &data_send) == -1)
        {
            cli_printf("ERROR resetting MPU, I2C write to reset bit failed");
            return -1;
        }
    }
    HAL_Delay(10);
    return 0;
}

static int __load_accel_calibration(void)
{
    FIL *fd;
    uint8_t raw[6] = {0, 0, 0, 0, 0, 0};
    double x, y, z, sx, sy, sz; // offsets and scales in xyz
    int16_t bias[3], factory[3];

    if (f_open(fd, "acc_cal.txt", FA_READ) != FR_OK)
    {
        // calibration file doesn't exist yet
        cli_printf("WARNING: no accelerometer calibration data founn");
        // use zero offsets
        accel_lengths[0] = 1.0;
        accel_lengths[1] = 1.0;
        accel_lengths[2] = 1.0;
        return 0;
    }
    char buff[100];
    f_gets(buff, 100, fd);
    // read in data
    if (sscanf(buff, "%lf,%lf,%lf,%lf,%lf,%lf", &x, &y, &z, &sx, &sy, &sz) != 6)
    {
        cli_printf("ERROR loading accel offsets, calibration file empty or malformed");
        // use zero offsets
        accel_lengths[0] = 1.0;
        accel_lengths[1] = 1.0;
        accel_lengths[2] = 1.0;
        return 0;
    }
    f_close(fd);

#ifdef DEBUG
    cli_printf("accel offsets: %lf %lf %lf\n", x, y, z);
    cli_printf("accel scales:  %lf %lf %lf\n", sx, sy, sz);
#endif

    // save scales globally
    accel_lengths[0] = sx;
    accel_lengths[1] = sy;
    accel_lengths[2] = sz;

    // read factory bias
    if (Sensors_I2C_ReadRegister(SLV_ADDR, MPU9250_XA_OFFSET_H, 2, &raw[0]) < 0)
    {
        return -1;
    }
    if (Sensors_I2C_ReadRegister(SLV_ADDR, MPU9250_YA_OFFSET_H, 2, &raw[2]) < 0)
    {
        return -1;
    }
    if (Sensors_I2C_ReadRegister(SLV_ADDR, MPU9250_ZA_OFFSET_H, 2, &raw[4]) < 0)
    {
        return -1;
    }
    // Turn the MSB and LSB into a signed 16-bit value
    factory[0] = (int16_t)(((uint16_t)raw[0] << 7) | (raw[1] >> 1));
    factory[1] = (int16_t)(((uint16_t)raw[2] << 7) | (raw[3] >> 1));
    factory[2] = (int16_t)(((uint16_t)raw[4] << 7) | (raw[5] >> 1));

    // convert offset in g to bias register which is 15-bits, 16G FSR
    bias[0] = factory[0] - round(x / 0.0009765615);
    bias[1] = factory[1] - round(y / 0.0009765615);
    bias[2] = factory[2] - round(z / 0.0009765615);

    // convert 16-bit bias to characters to write
    raw[0] = (bias[0] >> 7) & 0xFF;
    raw[1] = (bias[0] << 1) & 0xFF;
    raw[2] = (bias[1] >> 7) & 0xFF;
    raw[3] = (bias[1] << 1) & 0xFF;
    raw[4] = (bias[2] >> 7) & 0xFF;
    raw[5] = (bias[2] << 1) & 0xFF;

    // Push accel biases to hardware registers
    if (Sensors_I2C_WriteRegister(SLV_ADDR, MPU9250_XA_OFFSET_H, 2, &raw[0]) < 0)
    {
        cli_printf("ERROR: failed to write X accel offsets into IMU register\n");
        return -1;
    }
    if (Sensors_I2C_WriteRegister(SLV_ADDR, MPU9250_YA_OFFSET_H, 2, &raw[2]) < 0)
    {
        cli_printf("ERROR: failed to write Y accel offsets into IMU register\n");
        return -1;
    }
    if (Sensors_I2C_WriteRegister(SLV_ADDR, MPU9250_ZA_OFFSET_H, 2, &raw[4]) < 0)
    {
        cli_printf("ERROR: failed to write Z accel offsets into IMU register\n");
        return -1;
    }

    return 0;
}

static int __load_mag_calibration(void)
{
    FIL *fd;
    double x, y, z, sx, sy, sz;

    if (f_open(fd, "mag_cal.txt", FA_READ) != FR_OK)
    {
        // calibration file doesn't exist yet
        cli_printf("WARNING: no magnetometer calibration data found");
        x = 0.0;
        y = 0.0;
        z = 0.0;
        sx = 1.0;
        sy = 1.0;
        sz = 1.0;
    }
    else
    { // read in data
        char buff[100];
        f_gets(buff, 100, fd);
        if (sscanf(buff, "%lf,%lf,%lf,%lf,%lf,%lf", &x, &y, &z, &sx, &sy, &sz) != 6)
        {
            cli_printf("ERROR loading magnetometer calibration file, empty or malformed");
            x = 0.0;
            y = 0.0;
            z = 0.0;
            sx = 1.0;
            sy = 1.0;
            sz = 1.0;
        }
        f_close(fd);
    }

#ifdef DEBUG
    cli_printf("magcal: %lf %lf %lf %lf %lf %lf\n", x, y, z, sx, sy, sz);
#endif

    // write to global variables for use by mpu_read_mag
    mag_offsets[0] = x;
    mag_offsets[1] = y;
    mag_offsets[2] = z;
    mag_scales[0] = sx;
    mag_scales[1] = sy;
    mag_scales[2] = sz;

    return 0;
}

static int __load_gyro_calibration(void)
{
    FIL *fd;
    uint8_t data[6];
    int x, y, z;

    if (f_open(fd, "gyro_cal.txt", FA_READ) != FR_OK)
    {
        // calibration file doesn't exist yet
        cli_printf("WARNING: no gyro calibration data found");
        // use zero offsets
        x = 0;
        y = 0;
        z = 0;
    }
    else
    {
        // read in data
        char buff[100];
        f_gets(buff, 100, fd);
        if (sscanf(buff, "%d,%d,%d", &x, &y, &z) != 3)
        {
            cli_printf("ERROR loading gyro offsets, calibration file empty or malformed");
            // use zero offsets
            x = 0;
            y = 0;
            z = 0;
        }
        f_close(fd);
    }

#ifdef DEBUG
    cli_printf("offsets: %d %d %d\n", x, y, z);
#endif

    // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input
    // format. also make negative since we wish to subtract out the steady
    // state offset
    data[0] = (-x / 4 >> 8) & 0xFF;
    data[1] = (-x / 4) & 0xFF;
    data[2] = (-y / 4 >> 8) & 0xFF;
    data[3] = (-y / 4) & 0xFF;
    data[4] = (-z / 4 >> 8) & 0xFF;
    data[5] = (-z / 4) & 0xFF;

    // Push gyro biases to hardware registers
    if (Sensors_I2C_WriteRegister(SLV_ADDR, MPU9250_XG_OFFSET_H, 6, &data[0]))
    {
        cli_printf("ERROR: failed to load gyro offsets into IMU register\n");
        return -1;
    }
    return 0;
}

static int __write_gyro_cal_to_disk(int16_t offsets[3])
{
    FIL *fd;
    int ret;

    if (f_open(fd, "gyro_cal.txt", FA_WRITE | FA_CREATE_ALWAYS) != FR_OK)
    {
        cli_printf("ERROR in opening gyro calibration file for writing");
        return -1;
    }

    // write to the file, close, and exit
    if (f_printf(fd, "%d\n%d\n%d\n", offsets[0], offsets[1], offsets[2]) < 0)
    {
        cli_printf("ERROR in writing to file");
        f_close(fd);
        return -1;
    }
    f_close(fd);
    return 0;
}

static int __write_mag_cal_to_disk(double offsets[3], double scale[3])
{
    FIL *fd;
    int ret;

    if (f_open(fd, "mag_cal.txt", FA_WRITE | FA_CREATE_ALWAYS) != FR_OK)
    {
        cli_printf("ERROR in opening mag calibration file for writing");
        return -1;
    }

    // write to the file, close, and exit
    ret = f_printf(fd, "%.10f,%.10f,%.10f,%.10f,%.10f,%.10f",
                   offsets[0],
                   offsets[1],
                   offsets[2],
                   scale[0],
                   scale[1],
                   scale[2]);
    if (ret < 0)
    {
        cli_printf("ERROR in writing to file");
        f_close(fd);
        return -1;
    }
    f_close(fd);
    return 0;
}

static int __write_accel_cal_to_disk(double *center, double *lengths)
{
    FIL *fd;
    int ret;

    if (f_open(fd, "acc_cal.txt", FA_WRITE | FA_CREATE_ALWAYS) != FR_OK)
    {
        cli_printf("ERROR in opening acc calibration file for writing");
        return -1;
    }

    // write to the file, close, and exit
    ret = f_printf(fd, "%.10f,%.10f,%.10f,%.10f,%.10f,%.10f",
                   center[0],
                   center[1],
                   center[2],
                   lengths[0],
                   lengths[1],
                   lengths[2]);
    if (ret < 0)
    {
        perror("ERROR in writing to file\n");
        f_close(fd);
        return -1;
    }
    f_close(fd);
    return 0;
}

int __init_magnetometer(int cal_mode)
{
    uint8_t raw[3]; // calibration data stored here

    // Enable i2c bypass to allow talking to magnetometer
    if (__mpu_set_bypass(1))
    {
        cli_printf("failed to set mpu9250 into bypass i2c mode\n");
        return -1;
    }
    // magnetometer is actually a separate device with its
    // own address inside the mpu9250
    if (i2c_set_device_address(config.i2c_bus, 1, AK8963_ADDR))
    {
        cli_printf("ERROR: in __init_magnetometer, failed to set i2c device address\n");
        return -1;
    }
    // Power down magnetometer
    if (Sensors_I2C_WriteRegister(SLV_ADDR, AK8963_CNTL, 1, MAG_POWER_DN) < 0)
    {
        cli_printf("ERROR: in __init_magnetometer, failed to write to AK8963_CNTL register to power down\n");
        return -1;
    }
    HAL_Delay(1);
    // Enter Fuse ROM access mode
    if (Sensors_I2C_WriteRegister(SLV_ADDR, AK8963_CNTL, 1, MAG_FUSE_ROM))
    {
        cli_printf("ERROR: in __init_magnetometer, failed to write to AK8963_CNTL register\n");
        return -1;
    }
    HAL_Delay(1);
    // Read the xyz sensitivity adjustment values
    if (i2c_read_bytes(config.i2c_bus, AK8963_ASAX, 3, &raw[0]) < 0)
    {
        cli_printf("failed to read magnetometer adjustment register\n");
        i2c_set_device_address(config.i2c_bus, config.i2c_addr);
        //__mpu_set_bypass(0);
        return -1;
    }
    // Return sensitivity adjustment values
    mag_factory_adjust[0] = (raw[0] - 128) / 256.0 + 1.0;
    mag_factory_adjust[1] = (raw[1] - 128) / 256.0 + 1.0;
    mag_factory_adjust[2] = (raw[2] - 128) / 256.0 + 1.0;
    // Power down magnetometer again
    if (Sensors_I2C_WriteRegister(SLV_ADDR, AK8963_CNTL, 1, MAG_POWER_DN))
    {
        cli_printf("ERROR: in __init_magnetometer, failed to write to AK8963_CNTL register to power on\n");
        return -1;
    }
    HAL_Delay(1);
    // Configure the magnetometer for 16 bit resolution
    // and continuous sampling mode 2 (100hz)
    uint8_t c = MSCALE_16 | MAG_CONT_MES_2;
    if (Sensors_I2C_WriteRegister(SLV_ADDR, AK8963_CNTL, 1, &c))
    {
        cli_printf("ERROR: in __init_magnetometer, failed to write to AK8963_CNTL register to set sampling mode\n");
        return -1;
    }
    HAL_Delay(1);
    // go back to configuring the IMU, leave bypass on
    i2c_set_device_address(config.i2c_bus, config.i2c_addr);
    // load in magnetometer calibration
    if (!cal_mode)
    {
        __load_mag_calibration();
    }
    return 0;
}

int mpu_read_mag(mpu_data_t *data)
{
    uint8_t raw[7];
    int16_t adc[3];
    double factory_cal_data[3];

    // don't worry about checking data ready bit, not worth thet time
    // read the data ready bit to see if there is new data
    uint8_t st1;
    if (unlikely(i2c_read_byte(config.i2c_bus, AK8963_ST1, &st1) < 0))
    {
        cli_printf("ERROR reading Magnetometer, i2c_bypass is probably not set\n");
        return -1;
    }
#ifdef DEBUG
    cli_printf("st1: %d", st1);
#endif
    if (!(st1 & MAG_DATA_READY))
    {
        if (config.show_warnings)
        {
            cli_printf("no new magnetometer data ready, skipping read\n");
        }
        return 0;
    }
    // Read the six raw data regs into data array
    if (unlikely(i2c_read_bytes(config.i2c_bus, AK8963_XOUT_L, 7, &raw[0]) < 0))
    {
        cli_printf("ERROR: mpu_read_mag failed to read data register\n");
        return -1;
    }
    // check if the readings saturated such as because
    // of a local field source, discard data if so
    if (raw[6] & MAGNETOMETER_SATURATION)
    {
        if (config.show_warnings)
        {
            cli_printf("WARNING: magnetometer saturated, discarding data\n");
        }
        return -1;
    }
    // Turn the MSB and LSB into a signed 16-bit value
    // Data stored as little Endian
    adc[0] = (int16_t)(((int16_t)raw[1] << 8) | raw[0]);
    adc[1] = (int16_t)(((int16_t)raw[3] << 8) | raw[2]);
    adc[2] = (int16_t)(((int16_t)raw[5] << 8) | raw[4]);
#ifdef DEBUG
    cli_printf("raw mag:%d %d %d\n", adc[0], adc[1], adc[2]);
#endif

    // multiply by the sensitivity adjustment and convert to units of uT micro
    // Teslas. Also correct the coordinate system as someone in invensense
    // thought it would be bright idea to have the magnetometer coordinate
    // system aligned differently than the accelerometer and gyro.... -__-
    factory_cal_data[0] = adc[1] * mag_factory_adjust[1] * MAG_RAW_TO_uT;
    factory_cal_data[1] = adc[0] * mag_factory_adjust[0] * MAG_RAW_TO_uT;
    factory_cal_data[2] = -adc[2] * mag_factory_adjust[2] * MAG_RAW_TO_uT;

    // now apply out own calibration,
    data->mag[0] = (factory_cal_data[0] - mag_offsets[0]) * mag_scales[0];
    data->mag[1] = (factory_cal_data[1] - mag_offsets[1]) * mag_scales[1];
    data->mag[2] = (factory_cal_data[2] - mag_offsets[2]) * mag_scales[2];

    return 0;
}

int calibrate_gyro(void)
{
    uint8_t c, data[6];
    int32_t gyro_sum[3] = {0, 0, 0};
    int16_t offsets[3];
    was_last_steady = 1;

    // reset device, reset all registers
    if (__reset_mpu() == -1)
    {
        cli_printf("ERROR in mpu_calibrate_gyro_routine, failed to reset MPU9250");
        return -1;
    }

    // set up the IMU specifically for calibration.
    uint8_t data_send = 0x01;
    Sensors_I2C_WriteRegister(SLV_ADDR, MPU9250_PWR_MGMT_1, 1, &data_send);
    data_send = 0x00;
    Sensors_I2C_WriteRegister(SLV_ADDR, MPU9250_PWR_MGMT_2, 1, &data_send);
    HAL_Delay(200);

    // // set bias registers to 0
    // // Push gyro biases to hardware registers
    // uint8_t zeros[] = {0,0,0,0,0,0};
    // if(i2c_write_bytes(conf.i2c_bus, XG_OFFSET_H, 6, zeros)){
    // cli_printf("ERROR: failed to load gyro offsets into IMU register\n");
    // return -1;
    // }

    Sensors_I2C_WriteRegister(SLV_ADDR, MPU9250_INT_ENABLE, 1, &data_send);   // Disable all interrupts
    Sensors_I2C_WriteRegister(SLV_ADDR, MPU9250_FIFO_EN, 1, &data_send);      // Disable FIFO
    Sensors_I2C_WriteRegister(SLV_ADDR, MPU9250_PWR_MGMT_1, 1, &data_send);   // Turn on internal clock source
    Sensors_I2C_WriteRegister(SLV_ADDR, MPU9250_I2C_MST_CTRL, 1, &data_send); // Disable I2C master
    Sensors_I2C_WriteRegister(SLV_ADDR, MPU9250_USER_CTRL, 1, &data_send);    // Disable FIFO and I2C master
    data_send = 0x0C;
    Sensors_I2C_WriteRegister(SLV_ADDR, MPU9250_USER_CTRL, 1, &data_send); // Reset FIFO and DMP

    HAL_Delay(15);

    // Configure MPU9250 gyro and accelerometer for bias calculation
    data_send = 0x01;
    Sensors_I2C_WriteRegister(SLV_ADDR, MPU9250_CONFIG, 1, &data_send); // Set low-pass filter to 188 Hz
    data_send = 0x04;
    Sensors_I2C_WriteRegister(SLV_ADDR, MPU9250_SMPLRT_DIV, 1, &data_send); // Set sample rate to 200hz
    // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    data_send = 0x00;
    Sensors_I2C_WriteRegister(SLV_ADDR, MPU9250_GYRO_CONFIG, 1, &data_send);
    // Set accelerometer full-scale to 2 g, maximum sensitivity
    Sensors_I2C_WriteRegister(SLV_ADDR, MPU9250_ACCEL_CONFIG, 1, &data_send);

COLLECT_DATA:

    // Configure FIFO to capture gyro data for bias calculation
    data_send = 0x40;
    Sensors_I2C_WriteRegister(SLV_ADDR, MPU9250_USER_CTRL, 1, &data_send); // Enable FIFO
    // Enable gyro sensors for FIFO (max size 512 bytes in MPU-9250)
    c = MPU9250_FIFO_GYRO_X_EN | MPU9250_FIFO_GYRO_Y_EN | MPU9250_FIFO_GYRO_Z_EN;
    Sensors_I2C_WriteRegister(SLV_ADDR, MPU9250_FIFO_EN, 1, &c);
    // 6 bytes per sample. 200hz. wait 0.4 seconds
    HAL_Delay(400);

    // At end of sample accumulation, turn off FIFO sensor read
    data_send = 0x00;
    Sensors_I2C_WriteRegister(SLV_ADDR, MPU9250_FIFO_EN, 1, &data_send);
    // read FIFO sample count and log number of samples
    Sensors_I2C_ReadRegister(SLV_ADDR, MPU9250_FIFO_COUNTH, 2, &data[0]);
    int16_t fifo_count = ((uint16_t)data[0] << 8) | data[1];
    int samples = fifo_count / 6;

#ifdef DEBUG
    cli_printf("calibration samples: %d\n", samples);
#endif

    int i;
    int16_t x, y, z;
    vector_t vx = vector_empty();
    vector_t vy = vector_empty();
    vector_t vz = vector_empty();
    vector_alloc(&vx, samples);
    vector_alloc(&vy, samples);
    vector_alloc(&vz, samples);
    double dev_x, dev_y, dev_z;
    gyro_sum[0] = 0;
    gyro_sum[1] = 0;
    gyro_sum[2] = 0;
    for (i = 0; i < samples; i++)
    {
        // read data for averaging
        if (Sensors_I2C_ReadRegister(SLV_ADDR, MPU9250_FIFO_R_W, 6, data) < 0)
        {
            cli_printf("ERROR: failed to read FIFO\n");
            return -1;
        }
        x = (int16_t)(((int16_t)data[0] << 8) | data[1]);
        y = (int16_t)(((int16_t)data[2] << 8) | data[3]);
        z = (int16_t)(((int16_t)data[4] << 8) | data[5]);
        gyro_sum[0] += (int32_t)x;
        gyro_sum[1] += (int32_t)y;
        gyro_sum[2] += (int32_t)z;
        vx.d[i] = (double)x;
        vy.d[i] = (double)y;
        vz.d[i] = (double)z;
    }
    dev_x = vector_std_dev(vx);
    dev_y = vector_std_dev(vy);
    dev_z = vector_std_dev(vz);
    vector_free(&vx);
    vector_free(&vy);
    vector_free(&vz);

#ifdef DEBUG
    cli_printf("gyro sums: %d %d %d\n", gyro_sum[0], gyro_sum[1], gyro_sum[2]);
    cli_printf("std_deviation: %6.2f %6.2f %6.2f\n", dev_x, dev_y, dev_z);
#endif

    // try again is standard deviation is too high
    if (dev_x > GYRO_CAL_THRESH || dev_y > GYRO_CAL_THRESH || dev_z > GYRO_CAL_THRESH)
    {
        cli_printf("Gyro data too noisy, put me down on a solid surface!\n");
        cli_printf("trying again\n");
        was_last_steady = 0;
        goto COLLECT_DATA;
    }
    // this skips the first steady reading after a noisy reading
    // to make sure IMU has settled after being picked up.
    if (was_last_steady == 0)
    {
        was_last_steady = 1;
        goto COLLECT_DATA;
    }
    // average out the samples
    offsets[0] = (int16_t)(gyro_sum[0] / (int32_t)samples);
    offsets[1] = (int16_t)(gyro_sum[1] / (int32_t)samples);
    offsets[2] = (int16_t)(gyro_sum[2] / (int32_t)samples);

    // also check for values that are way out of bounds
    if (abs(offsets[0]) > GYRO_OFFSET_THRESH || abs(offsets[1]) > GYRO_OFFSET_THRESH || abs(offsets[2]) > GYRO_OFFSET_THRESH)
    {
        cli_printf("Gyro data out of bounds, put me down on a solid surface!\n");
        cli_printf("trying again\n");
        goto COLLECT_DATA;

#ifdef DEBUG
        cli_printf("offsets: %d %d %d\n", offsets[0], offsets[1], offsets[2]);
#endif
        // write to disk
        if (__write_gyro_cal_to_disk(offsets) < 0)
        {
            cli_printf("ERROR in calibrate_gyro_routine, failed to write to disk\n");
            return -1;
        }
        return 0;
    }
}

int calibrate_mag(void)
{
    int i;
    double new_scale[3];
    const int samples = 200;
    const int sample_time_us = 12000000; // 12 seconds ()
    const int loop_wait_us = sample_time_us / samples;
    const int sample_rate_hz = 1000000 / loop_wait_us;

    matrix_t A = matrix_empty();
    vector_t center = vector_empty();
    vector_t lengths = vector_empty();
    mpu_data_t imu_data; // to collect magnetometer data

    if (__init_magnetometer(1))
    {
        cli_printf("ERROR: failed to initialize_magnetometer\n");
        return -1;
    }

    // set local calibration to initial values and prepare variables
    mag_offsets[0] = 0.0;
    mag_offsets[1] = 0.0;
    mag_offsets[2] = 0.0;
    mag_scales[0] = 1.0;
    mag_scales[1] = 1.0;
    mag_scales[2] = 1.0;
    if (matrix_alloc(&A, samples, 3))
    {
        cli_printf("ERROR: in calibrate_mag_routine, failed to alloc data matrix\n");
        return -1;
    }

    // sample data
    i = 0;
    while (i < samples)
    {
        if (mpu_read_mag(&imu_data) < 0)
        {
            cli_printf("ERROR: failed to read magnetometer\n");
            break;
        }
        // make sure the data is non-zero
        if (abs(imu_data.mag[0]) < zero_tolerance &&
            abs(imu_data.mag[1]) < zero_tolerance &&
            abs(imu_data.mag[2]) < zero_tolerance)
        {
            cli_printf("ERROR: retreived all zeros from magnetometer\n");
            break;
        }
        // save data to matrix for ellipse fitting
        A.d[i][0] = imu_data.mag[0];
        A.d[i][1] = imu_data.mag[1];
        A.d[i][2] = imu_data.mag[2];
        i++;

        // print "keep going" every 4 seconds
        if (i % (sample_rate_hz * 4) == sample_rate_hz * 2)
        {
            cli_printf("keep spinning\n");
        }
        // print "you're doing great" every 4 seconds
        if (i % (sample_rate_hz * 4) == 0)
        {
            cli_printf("you're doing great\n");
        }

        HAL_Delay(loop_wait_us / 1000);
    }

    cli_printf("\n\nOkay Stop!\n");
    cli_printf("Calculating calibration constants.....\n");

    // if data collection loop exited without getting enough data, warn the
    // user and return -1, otherwise keep going normally
    if (i < samples)
    {
        cli_printf("exiting calibrate_mag_routine without saving new data\n");
        return -1;
    }
    // make empty vectors for ellipsoid fitting to populate
    if (algebra_fit_ellipsoid(A, &center, &lengths) < 0)
    {
        cli_printf("failed to fit ellipsoid to magnetometer data\n");
        matrix_free(&A);
        return -1;
    }
    // empty memory, we are done with A
    matrix_free(&A);
    // do some sanity checks to make sure data is reasonable
    if (abs(center.d[0]) > 200 || abs(center.d[1]) > 200 ||
        abs(center.d[2]) > 200)
    {
        cli_printf("ERROR: center of fitted ellipsoid out of bounds\n");
        vector_free(&center);
        vector_free(&lengths);
        return -1;
    }
    if (lengths.d[0] > 200 || lengths.d[0] < 5 ||
        lengths.d[1] > 200 || lengths.d[1] < 5 ||
        lengths.d[2] > 200 || lengths.d[2] < 5)
    {
        cli_printf("WARNING: length of fitted ellipsoid out of bounds\n");
        cli_printf("Saving suspicious calibration data anyway in case this is intentional\n");
    }
    // all seems well, calculate scaling factors to map ellipse lengths to
    // a sphere of radius 70uT, this scale will later be multiplied by the
    // factory corrected data
    new_scale[0] = 70.0 / lengths.d[0];
    new_scale[1] = 70.0 / lengths.d[1];
    new_scale[2] = 70.0 / lengths.d[2];
    // print results
    cli_printf("\n");
    cli_printf("Offsets X: %7.3f Y: %7.3f Z: %7.3f\n", center.d[0],
               center.d[1],
               center.d[2]);
    cli_printf("Scales  X: %7.3f Y: %7.3f Z: %7.3f\n", new_scale[0],
               new_scale[1],
               new_scale[2]);
    // write to disk
    if (__write_mag_cal_to_disk(center.d, new_scale) < 0)
    {
        vector_free(&center);
        vector_free(&lengths);
        return -1;
    }
    vector_free(&center);
    vector_free(&lengths);
    return 0;
}

int __collect_accel_samples(int *avg_raw)
{
    uint8_t data[6];
    uint8_t data_send;
    int32_t sum[3];
    int i, samples, fifo_count;
    int16_t x, y, z;
    double dev_x, dev_y, dev_z;
    vector_t vx = vector_empty();
    vector_t vy = vector_empty();
    vector_t vz = vector_empty();

    // Configure FIFO to capture gyro data for bias calculation
    data_send = 0x40;
    Sensors_I2C_WriteRegister(SLV_ADDR, MPU9250_USER_CTRL, 1, &data_send); // Enable FIFO
    // Enable accel sensors for FIFO (max size 512 bytes in MPU-9250)
    data_send = MPU9250_FIFO_ACCEL_EN;
    Sensors_I2C_WriteRegister(SLV_ADDR, MPU9250_FIFO_EN, 1, &data_send);
    // 6 bytes per sample. 200hz. wait 0.4 seconds
    HAL_Delay(400);

    // At end of sample accumulation, turn off FIFO sensor read
    data_send = 0x00;
    Sensors_I2C_WriteRegister(SLV_ADDR, MPU9250_FIFO_EN, 1, &data_send);
    // read FIFO sample count and log number of samples
    Sensors_I2C_WriteRegister(SLV_ADDR, MPU9250_FIFO_COUNTH, 2, &data[0]);
    fifo_count = ((uint16_t)data[0] << 8) | data[1];
    samples = fifo_count / 6;

#ifdef DEBUG
    cli_printf("calibration samples: %d\n", samples);
#endif

    vector_alloc(&vx, samples);
    vector_alloc(&vy, samples);
    vector_alloc(&vz, samples);

    sum[0] = 0;
    sum[1] = 0;
    sum[2] = 0;
    for (i = 0; i < samples; i++)
    {
        // read data for averaging
        if (Sensors_I2C_WriteRegister(SLV_ADDR, MPU9250_FIFO_R_W, 6, data) < 0)
        {
            cli_printf("ERROR in mpu_calibrate_accel_routine, failed to read FIFO\n");
            return -1;
        }
        x = (int16_t)(((int16_t)data[0] << 8) | data[1]);
        y = (int16_t)(((int16_t)data[2] << 8) | data[3]);
        z = (int16_t)(((int16_t)data[4] << 8) | data[5]);
        sum[0] += (int32_t)x;
        sum[1] += (int32_t)y;
        sum[2] += (int32_t)z;
        vx.d[i] = (double)x;
        vy.d[i] = (double)y;
        vz.d[i] = (double)z;
    }
    dev_x = vector_std_dev(vx);
    dev_y = vector_std_dev(vy);
    dev_z = vector_std_dev(vz);
    vector_free(&vx);
    vector_free(&vy);
    vector_free(&vz);

#ifdef DEBUG
    cli_printf("sums: %d %d %d\n", sum[0], sum[1], sum[2]);
    cli_printf("std_deviation: %6.2f %6.2f %6.2f\n", dev_x, dev_y, dev_z);
#endif

    // try again if standard deviation is too high
    if (dev_x > ACCEL_CAL_THRESH || dev_y > ACCEL_CAL_THRESH || dev_z > ACCEL_CAL_THRESH)
    {
        was_last_steady = 0;
        cli_printf("data too noisy, please hold me still\n");
        return 1;
    }
    // this skips the first steady reading after a noisy reading
    // to make sure IMU has settled after being picked up.
    if (was_last_steady == 0)
    {
        was_last_steady = 1;
        return 1;
    }
    // average out the samples
    avg_raw[0] = (sum[0] / (int32_t)samples);
    avg_raw[1] = (sum[1] / (int32_t)samples);
    avg_raw[2] = (sum[2] / (int32_t)samples);

#ifdef DEBUG
    cli_printf("avg: %d %d %d\n", avg_raw[0], avg_raw[1], avg_raw[2]);
#endif
    return 0;
}

int calibrate_accel(void)
{
    uint8_t data_send;
    int ret, i, j;
    int avg_raw[6][3];

    // reset device, reset all registers
    if (__reset_mpu() < 0)
    {
        cli_printf("ERROR in mpu_calibrate_accel_routine failed to reset MPU9250\n");
        return -1;
    }

    // set up the IMU specifically for calibration.
    data_send = 0x01;
    Sensors_I2C_WriteRegister(SLV_ADDR, MPU9250_PWR_MGMT_1, 1, &data_send);
    data_send = 0x00;
    Sensors_I2C_WriteRegister(SLV_ADDR, MPU9250_PWR_MGMT_2, 1, &data_send);
    HAL_Delay(200);

    Sensors_I2C_WriteRegister(SLV_ADDR, MPU9250_INT_ENABLE, 1, &data_send);   // Disable all interrupts
    Sensors_I2C_WriteRegister(SLV_ADDR, MPU9250_FIFO_EN, 1, &data_send);      // Disable FIFO
    Sensors_I2C_WriteRegister(SLV_ADDR, MPU9250_PWR_MGMT_1, 1, &data_send);   // Turn on internal clock source
    Sensors_I2C_WriteRegister(SLV_ADDR, MPU9250_I2C_MST_CTRL, 1, &data_send); // Disable I2C master
    Sensors_I2C_WriteRegister(SLV_ADDR, MPU9250_USER_CTRL, 1, &data_send);    // Disable FIFO and I2C master
    data_send = 0x0C;
    Sensors_I2C_WriteRegister(SLV_ADDR, MPU9250_USER_CTRL, 1, &data_send); // Reset FIFO and DMP
    HAL_Delay(15);

    // Configure MPU9250 gyro and accelerometer for bias calculation
    data_send = 0x01;
    Sensors_I2C_WriteRegister(SLV_ADDR, MPU9250_CONFIG, 1, &data_send); // Set low-pass filter to 188 Hz
    data_send = 0x04;
    Sensors_I2C_WriteRegister(SLV_ADDR, MPU9250_SMPLRT_DIV, 1, &data_send); // Set sample rate to 200hz
    data_send = 0x00;
    Sensors_I2C_WriteRegister(SLV_ADDR, MPU9250_GYRO_CONFIG, 1, &data_send);  // set G FSR to 250dps
    Sensors_I2C_WriteRegister(SLV_ADDR, MPU9250_ACCEL_CONFIG, 1, &data_send); // set A FSR to 2G

    // collect an orientation
    cli_printf("\nOrient Z pointing up and hold as still as possible\n");
    cli_printf("When ready, press any key to sample accelerometer\n");
    cli_get_char();
    ret = 1;
    was_last_steady = 0;
    while (ret)
    {
        ret = __collect_accel_samples(avg_raw[0]);
        if (ret == -1)
            return -1;
    }
    cli_printf("success\n");
    // collect an orientation
    cli_printf("\nOrient Z pointing down and hold as still as possible\n");
    cli_printf("When ready, press any key to sample accelerometer\n");
    cli_get_char();
    ret = 1;
    was_last_steady = 0;
    while (ret)
    {
        ret = __collect_accel_samples(avg_raw[1]);
        if (ret == -1)
            return -1;
    }
    cli_printf("success\n");
    // collect an orientation
    cli_printf("\nOrient X pointing up and hold as still as possible\n");
    cli_printf("When ready, press any key to sample accelerometer\n");
    cli_get_char();
    ret = 1;
    was_last_steady = 0;
    while (ret)
    {
        ret = __collect_accel_samples(avg_raw[2]);
        if (ret == -1)
            return -1;
    }
    cli_printf("success\n");
    // collect an orientation
    cli_printf("\nOrient X pointing down and hold as still as possible\n");
    cli_printf("When ready, press any key to sample accelerometer\n");
    cli_get_char();
    ret = 1;
    was_last_steady = 0;
    while (ret)
    {
        ret = __collect_accel_samples(avg_raw[3]);
        if (ret == -1)
            return -1;
    }
    cli_printf("success\n");
    // collect an orientation
    cli_printf("\nOrient Y pointing up and hold as still as possible\n");
    cli_printf("When ready, press any key to sample accelerometer\n");
    cli_get_char();
    ret = 1;
    was_last_steady = 0;
    while (ret)
    {
        ret = __collect_accel_samples(avg_raw[4]);
        if (ret == -1)
            return -1;
    }
    cli_printf("success\n");
    // collect an orientation
    cli_printf("\nOrient Y pointing down and hold as still as possible\n");
    cli_printf("When ready, press any key to sample accelerometer\n");
    cli_get_char();
    ret = 1;
    was_last_steady = 0;
    while (ret)
    {
        ret = __collect_accel_samples(avg_raw[5]);
        if (ret == -1)
            return -1;
    }
    cli_printf("success\n");

    // fit the ellipse
    matrix_t A = matrix_empty();
    vector_t center = vector_empty();
    vector_t lengths = vector_empty();

    if (matrix_alloc(&A, 6, 3))
    {
        cli_printf("ERROR: failed to alloc data matrix\n");
        return -1;
    }

    // convert to G and put in matrix
    for (i = 0; i < 6; i++)
    {
        for (j = 0; j < 3; j++)
        {
            A.d[i][j] = (avg_raw[i][j] / 16384.0);
        }
    }

    // make empty vectors for ellipsoid fitting to populate
    if (algebra_fit_ellipsoid(A, &center, &lengths) < 0)
    {
        cli_printf("failed to fit ellipsoid to magnetometer data\n");
        matrix_free(&A);
        return -1;
    }
    // empty memory, we are done with A
    matrix_free(&A);
    // do some sanity checks to make sure data is reasonable
    for (i = 0; i < 3; i++)
    {
        if (fabs(center.d[i]) > 0.3)
        {
            cli_printf("ERROR in mpu_calibrate_accel_routine, center of fitted ellipsoid out of bounds\n");
            cli_printf("most likely the unit was held in incorrect orientation during data collection\n");
            vector_free(&center);
            vector_free(&lengths);
            return -1;
        }
        if (isnan(center.d[i]) || isnan(lengths.d[i]))
        {
            cli_printf("ERROR in mpu_calibrate_accel_routine, data fitting produced NaN\n");
            cli_printf("most likely the unit was held in incorrect orientation during data collection\n");
            vector_free(&center);
            vector_free(&lengths);
            return -1;
        }
        if (lengths.d[i] > 1.3 || lengths.d[i] < 0.7)
        {
            cli_printf("ERROR in mpu_calibrate_accel_routine, scale out of bounds\n");
            cli_printf("most likely the unit was held in incorrect orientation during data collection\n");
            vector_free(&center);
            vector_free(&lengths);
            return -1;
        }
    }

    // print results
    cli_printf("\n");
    cli_printf("Offsets X: %7.3f Y: %7.3f Z: %7.3f\n", center.d[0],
               center.d[1],
               center.d[2]);
    cli_printf("Scales  X: %7.3f Y: %7.3f Z: %7.3f\n", lengths.d[0],
               lengths.d[1],
               lengths.d[2]);
    // write to disk
    if (__write_accel_cal_to_disk(center.d, lengths.d) == -1)
    {
        cli_printf("ERROR failed to write to disk\n");
        return -1;
    }
    vector_free(&center);
    vector_free(&lengths);
    return 0;
}

void EXTI4_IRQHandler(void)
{
    gyro_data_ready_cb();
    if (hal.new_gyro && hal.dmp_on)
    {
        short gyro[3], accel_short[3], sensors;
        unsigned char more;
        long accel[3], quat[4], temperature;

        dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors, &more);
        if (!more)
        {
            hal.new_gyro = 0;
        }
        if (sensors & INV_XYZ_GYRO)
        {
            inv_build_gyro(gyro, sensor_timestamp);
            if (new_temp)
            {
                new_temp = 0;
                mpu_get_temperature(&temperature, &sensor_timestamp);
                inv_build_temp(temperature, sensor_timestamp);
            }
        }
        if (sensors & INV_XYZ_ACCEL)
        {
            accel[0] = (long)accel_short[0];
            accel[1] = (long)accel_short[1];
            accel[2] = (long)accel_short[2];
            inv_build_accel(accel, 3, sensor_timestamp);
        }
        if (sensors & INV_WXYZ_QUAT)
        {
            inv_build_quat(quat, 9, sensor_timestamp);
        }
    }

#ifdef COMPASS_ENABLED
    if (new_compass)
    {
        short compass_short[3];
        long compass[3];
        new_compass = 0;
        if (!mpu_get_compass_reg(compass_short, &sensor_timestamp))
        {
            compass[0] = (long)compass_short[0];
            compass[1] = (long)compass_short[1];
            compass[2] = (long)compass_short[2];
            inv_build_compass(compass, 0, sensor_timestamp);
        }
    }
#endif
    inv_execute_on_data();
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
}
