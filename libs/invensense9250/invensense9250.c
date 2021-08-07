#include <stdbool.h>
#include "main.h"

#include "cli.h"
#include "invensense9250.h"

unsigned char accel_fsr, new_temp = 0;
unsigned short gyro_rate, gyro_fsr;
unsigned long sensor_timestamp;
struct int_param_s int_param;

static struct hal_s hal = {0};

/* USB RX binary semaphore. Actually, it's just a flag. Not included in struct
 * because it's declared extern elsewhere.
 */
volatile unsigned char rx_new;

unsigned char *mpl_key = (unsigned char *)"eMPL 5.1";

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
    .orientation = {-1, 0, 0,
                    0, -1, 0,
                    0, 0, 1}};

#if defined MPU9150 || defined MPU9250
static struct platform_data_s compass_pdata = {
    .orientation = {0, 1, 0,
                    -1, 0, 0,
                    0, 0, 1}};
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
#endif

void mpu9250_init(void)
{
    inv_error_t result = inv_init_mpl();
    if (result)
    {
        log_e("Could not initialize MPL.");
    }

    result = mpu_init(NULL);
    if (result)
    {
        log_e("Could not initialize gyro.");
        HAL_Delay(100);
        Error_Handler(__FILE__, __LINE__);
    }
    else
    {
        log_i("MPU9250 initialized");

        inv_enable_quaternion();
        inv_enable_9x_sensor_fusion();

        inv_enable_fast_nomot();
        inv_enable_gyro_tc();

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
        mpu_set_sample_rate(1000);

        /* Read back configuration in case it was set improperly. */
        mpu_get_sample_rate(&gyro_rate);
        mpu_get_gyro_fsr(&gyro_fsr);
        log_i("Sampling rate: %d", gyro_rate);
        mpu_get_accel_fsr(&accel_fsr);
#ifdef COMPASS_ENABLED
        mpu_get_compass_fsr(&compass_fsr);
#endif
        /* Sync driver configuration with MPL. */
        /* Sample rate expected in microseconds. */
        inv_set_gyro_sample_rate(1000000L / gyro_rate);
        inv_set_accel_sample_rate(1000000L / gyro_rate);
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
        get_tick(&sensor_timestamp);

        mpu9250_backend_init();

        hal.dmp_features = /**DMP_FEATURE_6X_LP_QUAT |*/ DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
                           DMP_FEATURE_GYRO_CAL;
        
        mpu9250_backend_config(&hal.dmp_features);

        HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(EXTI4_IRQn);

    }
}

void mpu9250_backend_init()
{
    inv_error_t result = dmp_load_motion_driver_firmware();
    if (result)
    {
        log_e("Could not load DMP firmware.");
    } else {
        hal.dmp_on = 1;
        MPL_LOGI("DMP enabled.");
    }
}

void mpu9250_backend_config(unsigned short * features)
{
    dmp_set_orientation(
        inv_orientation_matrix_to_scalar(gyro_pdata.orientation));
    dmp_enable_feature(hal.dmp_features);
    dmp_set_fifo_rate(40);
    mpu_set_dmp_state(1);
    hal.dmp_on = 1;
}

static inline void gyro_data_ready_cb(void)
{
    hal.new_gyro = 1;
    new_temp = 1;
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

static inline void read_from_mpl(void)
{
    long msg, data[9];
    int8_t accuracy;
    unsigned long timestamp;
    float float_data[3] = {0};

    if (inv_get_sensor_type_quat(data, &accuracy, (inv_time_t *)&timestamp))
    {
        /* Sends a quaternion packet to the PC. Since this is used by the Python
        * test app to visually represent a 3D quaternion, it's sent each time
        * the MPL has new data.
        */
        eMPL_send_quat(data);

        /* Specific data packets can be sent or suppressed using USB commands. */
        if (hal.report & PRINT_QUAT)
            eMPL_send_data(PACKET_DATA_QUAT, data);
    }

    if (hal.report & PRINT_ACCEL)
    {
        if (inv_get_sensor_type_accel(data, &accuracy,
                                      (inv_time_t *)&timestamp))
            eMPL_send_data(PACKET_DATA_ACCEL, data);
    }
    if (hal.report & PRINT_GYRO)
    {
        if (inv_get_sensor_type_gyro(data, &accuracy,
                                     (inv_time_t *)&timestamp))
            eMPL_send_data(PACKET_DATA_GYRO, data);
    }
#ifdef COMPASS_ENABLED
    if (hal.report & PRINT_COMPASS)
    {
        if (inv_get_sensor_type_compass(data, &accuracy,
                                        (inv_time_t *)&timestamp))
            eMPL_send_data(PACKET_DATA_COMPASS, data);
    }
#endif
    if (hal.report & PRINT_EULER)
    {
        if (inv_get_sensor_type_euler(data, &accuracy,
                                      (inv_time_t *)&timestamp))
            eMPL_send_data(PACKET_DATA_EULER, data);
    }
    if (hal.report & PRINT_ROT_MAT)
    {
        if (inv_get_sensor_type_rot_mat(data, &accuracy,
                                        (inv_time_t *)&timestamp))
            eMPL_send_data(PACKET_DATA_ROT, data);
    }
    if (hal.report & PRINT_HEADING)
    {
        if (inv_get_sensor_type_heading(data, &accuracy,
                                        (inv_time_t *)&timestamp))
            eMPL_send_data(PACKET_DATA_HEADING, data);
    }
    if (hal.report & PRINT_LINEAR_ACCEL)
    {
        if (inv_get_sensor_type_linear_acceleration(float_data, &accuracy, (inv_time_t *)&timestamp))
        {
            MPL_LOGI("Linear Accel: %7.5f %7.5f %7.5f\r\n",
                     float_data[0], float_data[1], float_data[2]);
        }
    }
    if (hal.report & PRINT_GRAVITY_VECTOR)
    {
        if (inv_get_sensor_type_gravity(float_data, &accuracy,
                                        (inv_time_t *)&timestamp))
            MPL_LOGI("Gravity Vector: %7.5f %7.5f %7.5f\r\n",
                     float_data[0], float_data[1], float_data[2]);
    }
    if (hal.report & PRINT_PEDO)
    {
        unsigned long timestamp;
        get_tick(&timestamp);
        if (timestamp > hal.next_pedo_ms)
        {
            hal.next_pedo_ms = timestamp + PEDO_READ_MS;
            unsigned long step_count, walk_time;
            dmp_get_pedometer_step_count(&step_count);
            dmp_get_pedometer_walk_time(&walk_time);
            MPL_LOGI("Walked %ld steps over %ld milliseconds..\n", step_count,
                     walk_time);
        }
    }

    /* Whenever the MPL detects a change in motion state, the application can
     * be notified. For this example, we use an LED to represent the current
     * motion state.
     */
    msg = inv_get_message_level_0(INV_MSG_MOTION_EVENT |
                                  INV_MSG_NO_MOTION_EVENT);
    if (msg)
    {
        if (msg & INV_MSG_MOTION_EVENT)
        {
            MPL_LOGI("Motion!\n");
        }
        else if (msg & INV_MSG_NO_MOTION_EVENT)
        {
            MPL_LOGI("No motion!\n");
        }
    }
}

void EXTI4_IRQHandler(void)
{
    gyro_data_ready_cb();
    while (hal.new_gyro && hal.dmp_on)
    {
        short gyro[3], accel_short[3], sensors;
        unsigned char more;
        long accel[3], quat[4], temperature;

        dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors, &more);
        if (!more)
            hal.new_gyro = 0;
        if (sensors & INV_XYZ_GYRO)
        {
            /* Push the new data to the MPL. */
            inv_build_gyro(gyro, sensor_timestamp);
            if (new_temp)
            {
                new_temp = 0;
                /* Temperature only used for gyro temp comp. */
                mpu_get_temperature(&temperature, &sensor_timestamp);
                inv_build_temp(temperature, sensor_timestamp);
            }
        }
        if (sensors & INV_XYZ_ACCEL)
        {
            accel[0] = (long)accel_short[0];
            accel[1] = (long)accel_short[1];
            accel[2] = (long)accel_short[2];
            inv_build_accel(accel, 0, sensor_timestamp);
        }
        if (sensors & INV_WXYZ_QUAT)
        {
            inv_build_quat(quat, 0, sensor_timestamp);
        }
    }
    
#ifdef COMPASS_ENABLED
    short compass_short[3];
    long compass[3];
    /* For any MPU device with an AKM on the auxiliary I2C bus, the raw
         * magnetometer registers are copied to special gyro registers.
         */
    if (!mpu_get_compass_reg(compass_short, &sensor_timestamp))
    {
        compass[0] = (long)compass_short[0];
        compass[1] = (long)compass_short[1];
        compass[2] = (long)compass_short[2];
        /* NOTE: If using a third-party compass calibration library,
             * pass in the compass data in uT * 2^16 and set the second
             * parameter to INV_CALIBRATED | acc, where acc is the
             * accuracy from 0 to 3.
             */
        inv_build_compass(compass, 0, sensor_timestamp);
    }
#endif
    inv_execute_on_data();
    /* This function reads bias-compensated sensor data and sensor
         * fusion outputs from the MPL. The outputs are formatted as seen
         * in eMPL_outputs.c. This function only needs to be called at the
         * rate requested by the host.
         */

    hal.report = PRINT_ACCEL | PRINT_GYRO | PRINT_EULER | PRINT_QUAT;
    //read_from_mpl();
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
}