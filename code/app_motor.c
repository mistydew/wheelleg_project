#include "app_motor.h"
#include "app_safety.h"
#include "small_driver_uart_control.h"

volatile uint32 communication_count = 0;

#if (MOTOR_OUTPUT_ENABLE)
static int AppMotor_LimitDuty(int duty)
{
    if(duty > MOTOR_DUTY_LIMIT)
    {
        return MOTOR_DUTY_LIMIT;
    }

    if(duty < -MOTOR_DUTY_LIMIT)
    {
        return -MOTOR_DUTY_LIMIT;
    }

    return duty;
}
#endif

void AppMotor_Init(void)
{
    communication_count = 0;
    small_driver_uart_init();
}

void AppMotor_StopAll(void)
{
    small_driver_set_duty(&small_driver_value, 0, 0);
}

void AppMotor_SetDuty(int left_duty, int right_duty)
{
#if (MOTOR_OUTPUT_ENABLE)
    int left_output;
    int right_output;
#endif

    if(!AppSafety_IsSafe())
    {
        AppMotor_StopAll();
        return;
    }

#if (MOTOR_OUTPUT_ENABLE)
    left_output = AppMotor_LimitDuty(left_duty * MOTOR_LEFT_SIGN);
    right_output = AppMotor_LimitDuty(right_duty * MOTOR_RIGHT_SIGN);

    small_driver_set_duty(&small_driver_value, (int16)left_output, (int16)right_output);
#else
    (void)left_duty;
    (void)right_duty;
    AppMotor_StopAll();
#endif
}

void AppMotor_SetPercent(float left_percent, float right_percent)
{
    int left_duty;
    int right_duty;

    left_duty = (int)(left_percent * ((float)PWM_DUTY_MAX / 100.0f));
    right_duty = (int)(right_percent * ((float)PWM_DUTY_MAX / 100.0f));

    AppMotor_SetDuty(left_duty, right_duty);
}

void AppMotor_RequestSpeedFeedback(void)
{
    small_driver_get_speed(&small_driver_value);
}

int16 AppMotor_GetLeftSpeed(void)
{
    return small_driver_value.receive_left_speed_data;
}

int16 AppMotor_GetRightSpeed(void)
{
    return small_driver_value.receive_right_speed_data;
}

uint32 AppMotor_GetCommunicationCount(void)
{
    return communication_count;
}
