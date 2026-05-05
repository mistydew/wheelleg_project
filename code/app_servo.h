#ifndef APP_SERVO_H
#define APP_SERVO_H

#include "zf_common_headfile.h"

/* Adjust this channel to the actual BDS300 signal pin used on the car. */
#define APP_SERVO_PWM_CH               (TCPWM_CH59_P11_2)
#define APP_SERVO_PWM_FREQ_HZ          (50u)

/* For 50 Hz servo PWM: 500=5.0%, 750=7.5%, 1000=10.0% when PWM_DUTY_MAX is 10000. */
#define APP_SERVO_DUTY_MIN             (500u)
#define APP_SERVO_DUTY_NEUTRAL         (750u)
#define APP_SERVO_DUTY_MAX             (1000u)

void AppServo_Init(void);
void AppServo_SetNeutral(void);

#endif
