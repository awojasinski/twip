#ifndef CONTROL_H
#define CONTROL_H

#include "tim.h"
#include "gpio.h"

#define MAX_CONTROL_PWM 1062

typedef enum controler_type_t
{
    CONTROLER_P,
    CONTROLER_PI,
    CONTROLER_PD,
    CONTROLER_PID,
    CONTROLER_LQ
} controler_type_t;

typedef enum control_channel_t
{
    CONTROL_RIGHT_WHEEL = TIM_CHANNEL_4,
    CONTROL_LEFT_WHEEL = TIM_CHANNEL_1,
} control_channel_t;

/** @brief Structure describing vector. */
typedef struct
{
    uint8_t len;      //< Lenght of the vector.
    float *buff;      //< Pointer to the vector values.
    bool initialized; //< Flag for signaling initialization of the vectorl.
} control_vect;

/** @brief Structure describing controler. */
typedef struct
{
    float dt;              //< Sampilng time.
    int order;             //< Order of discrete-time transfer function of the controler.
    control_vect num;      //< Vector with numerator of the discrete-time transfer function.
    control_vect den;      //< Vector with denumeratr of the discrete-time transfer function.
    control_vect in_buff;  //< Ring buffer that stores input signals for given controler.
    control_vect out_buff; //< Ring buffer that stores output signals for given controler.
    float min_limit;       //< Minmium value of the controlers output. Used when saturation in enabled.
    float max_limit;       //< Maxmium value of the controlers output. Used when saturation in enabled.
    bool saturation;       //< Bool value for enabling saturation in controler
} control_pid_t;

/** @brief Function for initializing controler.
 *
 * @param[in] p_pwm    Pointer to the structure describing PWM timer used by controled motors.
 * @param[in] ch_right Timer channel used by right motor of the robot.
 * @param[in] ch_left  Timer channel used by left motor of the robot.
 */
void control_init(TIM_HandleTypeDef *p_pwm, uint32_t ch_right, uint32_t ch_left);

/** @brief Function for initialization of the controler.
 *
 * @param[in] p_pid Pointer to the structure describing controler.
 * @param[in] P     Value of the proporcional part of the PID controler.
 * @param[in] I     Value of the integral part of the PID conteoler.
 * @param[in] D     Value of the derivative part of the PID controler.
 * @param[in] N     Rolloff filter value.
 * @param[in] dt    Sampling time of the controler.
 * @param[in] type  Type of PID controler used.
 */
void control_pid_set(control_pid_t *p_pid, float P, float I, float D, uint8_t N, float dt, controler_type_t type);

/** @brief Function for enabling saturation capability in given controler.
 *
 * @param[in] p_pid Pointer to the structure describing controler.
 * @param[in] min   Minimal value for saturation filter.
 * @param[in] max   Maximum value for saturation filter.
 */
void control_enable_saturation(control_pid_t *p_pid, float min, float max);

/** @brief Function for computing output signal from controler.
 *
 * @param[in] p_control Pointer to the structure describing controler.
 * @param[in] input     Input value.
 *
 * @return Output signal from controler.
 */
float control_signal_get(control_pid_t *p_control, float input);

/** @brief Function driving motor with given control signal.
 *
 * @param[in] input         Infill value of PWM signal.
 * @param[in] motor_channel Channel of the motor to be set.
 */
void control_dirve_motors(int8_t input, control_channel_t motor_channel);

#endif
