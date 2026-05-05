#ifndef APP_BALANCE_H
#define APP_BALANCE_H

#include "zf_common_headfile.h"

#define BALANCE_GYRO_PITCH_AXIS    (2)
#define MOTOR_OUTPUT_ENABLE        (0)

typedef struct
{
    float kp_angle;
    float kd_gyro;
    float pitch_target_deg;
    float pitch_error_deg;
    float gyro_pitch_dps;
    float output;
    float output_limit;
    uint8 ready;
} app_balance_t;

extern app_balance_t g_balance;

void AppBalance_Init(void);
void AppBalance_Update(void);
float AppBalance_GetOutput(void);

#endif
