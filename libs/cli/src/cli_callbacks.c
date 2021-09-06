#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include "gpio.h"
#include "tim.h"

#include "cli.h"
#include "cli_callbacks.h"
#include "uart.h"
#include "mpu9250.h"
#include "encoder.h"
#include "control.h"

static cmd_error_t cli_pwm_callback(char *);
static cmd_error_t cli_led_callback(char *);
static cmd_error_t cli_pause_callback(char *);
static cmd_error_t cli_continue_callback(char *);
static cmd_error_t cli_help_callback(char *);
static cmd_error_t cli_cls_callback(char *);
static cmd_error_t cli_imu_callback(char *);
static cmd_error_t cli_pid_motor_callback(char *);

static bool twip_paused = false;

const cmd_t cmd_list[CALLBACKS_CNT] = {
    {CLI_CALLBACK_PWM, "pwm", &cli_pwm_callback},
    {CLI_CALLBACK_LED, "led", &cli_led_callback},
    {CLI_CALLBACK_PAUSE, "pause", &cli_pause_callback},
    {CLI_CALLBACK_CONTINUE, "continue", &cli_continue_callback},
    {CLI_CALLBACK_HELP, "help", &cli_help_callback},
    {CLI_CALLBACK_CLEAR, "cls", &cli_cls_callback},
    {CLI_CALLBACK_IMU, "imu", &cli_imu_callback},
    {CLI_CALLBACK_PID_MOTOR, "pid", &cli_pid_motor_callback},
};

static cmd_error_t cli_pwm_callback(char *cmd)
{
    strtok(cmd, EOF_BYTE_SET);
    char *param = strtok(NULL, EOF_BYTE_SET);

    int8_t pwm_filling = (int8_t)atoi(param);
    control_dirve_motors(pwm_filling, CONTROL_RIGHT_WHEEL);
    control_dirve_motors(pwm_filling, CONTROL_LEFT_WHEEL);
    return CMD_OK;
}

static cmd_error_t cli_led_callback(char *cmd)
{
    strtok(cmd, EOF_BYTE_SET);
    char *cmd_chunk = strtok(NULL, EOF_BYTE_SET);

    if (strcmp(cmd_chunk, "blink") == 0)
    {
        char *param = strtok(NULL, EOF_BYTE_SET);
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        HAL_Delay((uint32_t)atoi(param));
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        return CMD_OK;
    }
    else if (strcmp(cmd_chunk, "power") == 0)
    {
        char *param = strtok(NULL, EOF_BYTE_SET);
        if (strcmp(param, "on") == 0 || strcmp(param, "1") == 0)
        {
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
            return CMD_OK;
        }
        else if (strcmp(param, "off") == 0 || strcmp(param, "0") == 0)
        {
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
            return CMD_OK;
        }
        return CMD_WRONG_PARAM;
    }
    else if (stpcpy(cmd_chunk, "on") == 0)
    {
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
        return CMD_OK;
    }
    else if (stpcpy(cmd_chunk, "off") == 0)
    {
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    }
    return CMD_WRONG_PARAM;
}

static cmd_error_t cli_help_callback(char *cmd)
{
    UNUSED(cmd);
    cli_info();
    cli_printf("Available commands");
    for (uint8_t i = 0; i < CLI_CMD_CALLBACKS_CNT; i++)
    {
        cli_printf("twip %s", cmd_list[i].command);
    }
    return CMD_OK;
}

static cmd_error_t cli_cls_callback(char *cmd)
{
    UNUSED(cmd);
    cli_clear_console();
    cli_info();
    return CMD_OK;
}

static cmd_error_t cli_pid_motor_callback(char *cmd)
{
    static int error_threshold_deg = 1;

    static float P = 35.0F;
    static float I = 0.0F;
    static float D = 20;
    static uint32_t sample_time = 5;

    strtok(cmd, EOF_BYTE_SET);
    char *cmd_chunk = strtok(NULL, EOF_BYTE_SET);

    if (strcmp(cmd_chunk, "config") == 0)
    {
        cli_printf("Previous PID parameters:");
        cli_printf("P: %f I: %f D: %f", P, I, D);
        P = atoff(strtok(NULL, EOF_BYTE_SET));
        I = atoff(strtok(NULL, EOF_BYTE_SET));
        D = atoff(strtok(NULL, EOF_BYTE_SET));

        cli_printf("New PID parameters:");
        cli_printf("P: %f I: %f D: %f", P, I, D);
        return CMD_OK;
    }
    else if (strcmp(cmd_chunk, "set") == 0)
    {
        uart_show_recived_input(false);

        float set_value = atoff(strtok(NULL, EOF_BYTE_SET));

        float control, err, err_div;
        float err_int = 0;
        float err_prev = 0;

        uint32_t timestamp;

        char c;
        do
        {
            err = set_value - encoder_get_angle_deg((encoder_t *)&encoder_left);

            timestamp = HAL_GetTick();

            if (err < error_threshold_deg && err > -error_threshold_deg)
            {
                err = 0;
            }
            err_int += err * (float)sample_time / 1000;
            err_div = (err - err_prev) / ((float)sample_time / 1000);
            err_prev = err;

            control = P * err + I * err_int + D * err_div;

            control_dirve_motors((int8_t)control, CONTROL_LEFT_WHEEL);

            while ((HAL_GetTick() - timestamp) < sample_time)
            {
                __NOP();
            }

            c = cli_get_char();
        } while (c != 'q' && c != 'Q' && c != 27);

        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        HAL_GPIO_WritePin(Motor_L_direction1_GPIO_Port, Motor_L_direction1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Motor_L_direction2_GPIO_Port, Motor_L_direction2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Motor_R_direction1_GPIO_Port, Motor_R_direction1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Motor_R_direction2_GPIO_Port, Motor_R_direction2_Pin, GPIO_PIN_RESET);
        cli_clear_buffer();
        uart_show_recived_input(true);
        return CMD_OK;
    }
    return CMD_WRONG_PARAM;
}

static cmd_error_t cli_imu_callback(char *cmd)
{
    strtok(cmd, EOF_BYTE_SET);
    char *cmd_chunk = strtok(NULL, EOF_BYTE_SET);

    if (strcmp(cmd_chunk, "show") == 0)
    {
        uart_show_recived_input(false);
        cli_clear_line(1);
        char c;
        do
        {
            mpu9250_data_scaled((mpu9250_data_t *)&hmpu9250_data);
            cli_printf("ACCEL:\tX:%6.3f Y:%6.3f Z:%6.3f", hmpu9250_data.accel.x, hmpu9250_data.accel.y, hmpu9250_data.accel.z);
            cli_printf("GYRO:\tX:%6.3f Y:%6.3f Z:%6.3f", hmpu9250_data.gyro.x, hmpu9250_data.gyro.y, hmpu9250_data.gyro.z);
            cli_printf("TEMP: %4.2f", hmpu9250_data.temp);

            cli_delay(200);
            cli_clear_line(3);
            c = cli_get_char();
        } while (c != 'q' && c != 'Q' && c != 27);
        cli_clear_buffer();
        uart_show_recived_input(true);

        return CMD_OK;
    }
    else if (strcmp(cmd_chunk, "log") == 0)
    {
        char *filename;
        cmd_chunk = strtok(NULL, EOF_BYTE_SET);
        if (strcmp(cmd_chunk, "-n") == 0)
        {
            filename = strtok(NULL, EOF_BYTE_SET);
            cli_printf("Logging data to %s file", filename);
        }
        else
        {
            filename = "imu9250_log";
            cli_printf("Logging data to %s file", filename);
        }

        return CMD_OK;
    }

    return CMD_WRONG_PARAM;
}

static cmd_error_t cli_pause_callback(char *cmd)
{
    UNUSED(cmd);
    twip_paused = true;
    cli_printf("PAUSED");
    cli_clear_buffer();
    while (twip_paused)
    {
        cli_main();
    }
    return CMD_OK;
}

static cmd_error_t cli_continue_callback(char *cmd)
{
    UNUSED(cmd);
    twip_paused = false;
    cli_printf("CONTINUE");
    return CMD_OK;
}
