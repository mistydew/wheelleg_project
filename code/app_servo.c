#include "app_servo.h"

static uint32 AppServo_LimitDuty(uint32 duty)
{
    if(duty < APP_SERVO_DUTY_MIN)
    {
        return APP_SERVO_DUTY_MIN;
    }

    if(duty > APP_SERVO_DUTY_MAX)
    {
        return APP_SERVO_DUTY_MAX;
    }

    return duty;
}

void AppServo_Init(void)
{
    pwm_init(APP_SERVO_PWM_CH,
             APP_SERVO_PWM_FREQ_HZ,
             AppServo_LimitDuty(APP_SERVO_DUTY_NEUTRAL));
}

void AppServo_SetNeutral(void)
{
    pwm_set_duty(APP_SERVO_PWM_CH, AppServo_LimitDuty(APP_SERVO_DUTY_NEUTRAL));
}
