#ifndef APP_MOTOR_H
#define APP_MOTOR_H

#include "zf_common_headfile.h"

#ifndef MOTOR_OUTPUT_ENABLE
#define MOTOR_OUTPUT_ENABLE      (1)
#endif

#ifndef MOTOR_OPEN_LOOP_TEST
#define MOTOR_OPEN_LOOP_TEST     (0)
#endif

#ifndef MOTOR_LEFT_SIGN
#define MOTOR_LEFT_SIGN          (1)
#endif

#ifndef MOTOR_RIGHT_SIGN
#define MOTOR_RIGHT_SIGN         (-1)
#endif

#ifndef MOTOR_DUTY_LIMIT
#define MOTOR_DUTY_LIMIT         ((int)(PWM_DUTY_MAX * 50 / 100))
#endif

extern volatile uint32 communication_count;

void AppMotor_Init(void);
void AppMotor_StopAll(void);
void AppMotor_SetDuty(int left_duty, int right_duty);
void AppMotor_SetPercent(float left_percent, float right_percent);
void AppMotor_RequestSpeedFeedback(void);
int16 AppMotor_GetLeftSpeed(void);
int16 AppMotor_GetRightSpeed(void);
uint32 AppMotor_GetCommunicationCount(void);

#endif
