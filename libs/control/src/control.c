#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include "stm32g4xx_hal.h"
#include "tim.h"

#include "control.h"

typedef struct
{
    TIM_HandleTypeDef *p_tim;
    uint32_t ch_r;
    uint32_t ch_l;

} control_config_t;

/** @brief Structure describing ring buffer. */
typedef struct
{
    float *buff;     ///< Pointer to dynamically allocated data
    int len;         ///< Number of elements the buffer can hold
    int index;       ///< Index of the most recently added value
    int initialized; ///< Flag indicating if memory has been allocated for the buffer
} ringbuf_t;

static control_config_t m_control_config;

/** @brief Function for allocating memory for ring buffer.
 *
 * @param[in] p_ring Pointer to the structure describing ring buffer.
 * @param[in] len    Lenght of allocated ring buffer.
 *
 * @return True if allocation was successful, otherwise false.
 */
static bool allocate_ringbuff(ringbuf_t *p_ring, uint8_t len)
{
    if (!p_ring->initialized)
    {
        p_ring->buff = (control_vect *)malloc(len * sizeof(float));
        if (p_ring->buff)
        {
            p_ring->len = len;
            p_ring->index = 0;
            p_ring->initialized = true;
            return true;
        }
        else
        {
            return false;
        }
    }
    return false;
}

/** @brief Function for deallocating memory used by ring buffer.
 *
 * @param[in] p_ring Pointer to the structure describing ring buffer.
 *
 * @return True if buffer was released, otherwise false.
 */
static bool free_ringbuff(ringbuf_t *p_ring)
{
    if (p_ring->buff)
    {
        free(p_ring->buff);
        p_ring->len = 0;
        p_ring->initialized = false;
        return true;
    }
    else
    {
        return false;
    }
}

/** @brief Function for pushing new value on given ring buffer.
 *
 * @param[in] p_ring Pointer to the structure describing ring buffer.
 * @param[in] value  Input value to be pushed on top of the ring buffer.
 *
 * @return True if operation was successful, otherwise false.
 */
static bool push_ringbuff(ringbuf_t *p_ring, float value)
{
    if (!p_ring)
    {
        return false;
    }
    if (!p_ring->initialized)
    {
        return false;
    }
    p_ring->index = (p_ring->index + 1) % p_ring->len;
    p_ring->buff[p_ring->index] = value;
    return true;
}

/** @brief Function for getting value from given ring buffer.
 *
 * @param[in] p_ring   Pointer to the structure describing ring buffer.
 * @param[in] position Steps back in ring buffer to fetch the value from.
 *
 * @return Value stored on that position in ring buffer.
 */
static float get_ringbuff(ringbuf_t *p_ring, uint8_t position)
{

    uint8_t return_index = p_ring->index - position;
    if (return_index < 0)
    {
        return_index += p_ring->len;
    }
    return p_ring->buff[return_index];
}

/** @brief Function for allocating memory for vector.
 *
 * @param[in] p_vect Pointer to the structure describing vector.
 * @param[in] len    Lenght of allocated vector.
 *
 * @return True if allocation was successful, otherwise false.
 */
static bool allocate_vector(control_vect *p_vect, uint8_t len)
{
    if (!p_vect->initialized)
    {
        p_vect->buff = (control_vect *)malloc(len * sizeof(float));
        if (p_vect->buff)
        {
            p_vect->len = len;
            p_vect->initialized = true;
            return true;
        }
        else
        {
            return false;
        }
    }
    return false;
}

/** @brief Function for duplicating vector.
 *
 * @param[in] p_destination Pointer to the structure describing vector that will store coppied values.
 * @param[in] p_source      Pointer to the structure describing vector that will be coppied.
 *
 * @return True if operation was successful, otherwise false.
 */
static bool duplicate_vector(control_vect *p_destination, control_vect *p_source)
{
    if (!p_source->initialized)
    {
        return false;
    }

    if (!allocate_vector(p_destination, p_source->len))
    {
        return false;
    }

    memcpy(p_destination->buff, p_source->buff, p_source->len * sizeof(float));
    return true;
}

/** @brief Function for deallocating memory for vector.
 *
 * @param[in] p_vect Pointer to the structure describing vector.
 *
 * @return True if memory was released, otherwise false.
 */
static bool free_vector(control_vect *p_vect)
{
    if (p_vect->buff)
    {
        free(p_vect->buff);
    }
    p_vect->len = 0;
    p_vect->initialized = false;
    return true;
}

void control_init(TIM_HandleTypeDef *p_pwm, uint32_t channel_right, uint32_t channel_left)
{
    m_control_config.p_tim = p_pwm;
    m_control_config.ch_r = channel_right;
    m_control_config.ch_l = channel_left;

    HAL_TIM_PWM_Start(p_pwm, channel_right);
    HAL_TIM_PWM_Start(p_pwm, channel_left);
}

void control_pid_set(control_pid_t *p_pid, float P, float I, float D, uint8_t N, float dt, controler_type_t type)
{
    control_vect num, den;

    p_pid->dt = dt;

    switch (type)
    {
    case CONTROLER_P:
        allocate_vector(&num, 1);
        allocate_vector(&den, 1);

        num.buff[0] = P;
        den.buff[0] = 1;
        break;
    case CONTROLER_PD:
        allocate_vector(&num, 2);
        allocate_vector(&den, 2);

        num.buff[0] = P + D * N;
        num.buff[1] = P * N * dt - P - D * N;

        den.buff[0] = 1;
        den.buff[1] = N * dt - 1;
        break;
    case CONTROLER_PI:
        allocate_vector(&num, 2);
        allocate_vector(&den, 2);

        num.buff[0] = P;
        num.buff[1] = I * dt - P;

        den.buff[0] = 1;
        den.buff[1] = -1;
        break;
    case CONTROLER_PID:
        allocate_vector(&num, 3);
        allocate_vector(&den, 3);

        num.buff[0] = P + D * N;
        num.buff[1] = -2 * P + P * N * dt + I * dt - 2 * D * N;
        num.buff[2] = P - P * N * dt - I * dt + I * N * dt + D * N;

        den.buff[0] = 1;
        den.buff[1] = -2 + N * dt;
        den.buff[2] = 1 - N * dt;
        break;
    }

    p_pid->order = den.len - 1;

    duplicate_vector(&p_pid->num, &num);
    duplicate_vector(&p_pid->den, &den);

    allocate_ringbuff(&p_pid->in_buff, p_pid->den.len);
    allocate_ringbuff(&p_pid->out_buff, p_pid->den.len);

    free_vector(&num);
    free_vector(&den);
}

void control_enable_saturation(control_pid_t *p_pid, float min, float max)
{
    p_pid->saturation = true;
    p_pid->min_limit = min;
    p_pid->max_limit = max;
}

float control_signal_get(control_pid_t *p_control, float input)
{
    int rel_deg;
    float temp1 = 0.0f;
    float temp2 = 0.0f;

    push_ringbuff(&p_control->in_buff, input);

    rel_deg = p_control->den.len - p_control->num.len;
    for (int i = 0; i < p_control->num.len; i++)
    {
        temp1 += p_control->num.buff[i] * get_ringbuff(&p_control->in_buff, i + rel_deg);
    }

    for (int i = 0; i < p_control->order; i++)
    {
        temp2 -= p_control->den.buff[i + 1] * get_ringbuff(&p_control->out_buff, i);
    }

    float control = temp1 + temp2;

    if (p_control->saturation)
    {
        if (control > p_control->max_limit)
        {
            return p_control->max_limit;
        }
        else if (control < p_control->min_limit)
        {
            return p_control->min_limit;
        }
    }

    push_ringbuff(&p_control->out_buff, control);
    return control;
}

void control_dirve_motors(int8_t control, control_channel_t ch)
{
    uint32_t pwm = (uint32_t)abs((int)((control / 100.f) * MAX_CONTROL_PWM));

    GPIO_TypeDef *gpio1 = ch == CONTROL_RIGHT_WHEEL ? Motor_R_direction1_GPIO_Port : Motor_L_direction1_GPIO_Port;
    GPIO_TypeDef *gpio2 = ch == CONTROL_RIGHT_WHEEL ? Motor_R_direction2_GPIO_Port : Motor_L_direction2_GPIO_Port;

    uint16_t pin1 = ch == CONTROL_RIGHT_WHEEL ? Motor_R_direction1_Pin : Motor_L_direction1_Pin;
    uint16_t pin2 = ch == CONTROL_RIGHT_WHEEL ? Motor_R_direction2_Pin : Motor_L_direction2_Pin;

    pwm = pwm > MAX_CONTROL_PWM ? MAX_CONTROL_PWM : pwm;

    if (control > 0)
    {
        HAL_GPIO_WritePin(gpio1, pin1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(gpio2, pin2, GPIO_PIN_SET);
    }
    else if (control < 0)
    {
        HAL_GPIO_WritePin(gpio1, pin1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(gpio2, pin2, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(gpio1, pin1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(gpio2, pin2, GPIO_PIN_RESET);
    }
    __HAL_TIM_SET_COMPARE(m_control_config.p_tim, ch, pwm);
}
