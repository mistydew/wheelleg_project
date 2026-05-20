#ifndef APP_BALANCE_H
#define APP_BALANCE_H

#include "zf_common_headfile.h"

#define BALANCE_GYRO_PITCH_AXIS    (2)
#ifndef BALANCE_MOTOR_SIGN
#define BALANCE_MOTOR_SIGN         (-1.0f)
#endif
#ifndef BALANCE_OUTPUT_LIMIT
#define BALANCE_OUTPUT_LIMIT       (5000.0f)
#endif
#ifndef SPEED_LEFT_SIGN
#define SPEED_LEFT_SIGN            (-1.0f)
#endif
#ifndef SPEED_RIGHT_SIGN
#define SPEED_RIGHT_SIGN           ( 1.0f)
#endif

typedef struct
{
    float kp_angle;
    float kd_gyro;
    float pitch_target_deg;
    float pitch_error_deg;
    float gyro_pitch_dps;
    float angle_output;
    float forward_speed;
    float kv_speed;
    float speed_output;
    float output;
    float output_limit;
    uint8 ready;
} app_balance_t;

extern app_balance_t g_balance;

void AppBalance_Init(void);
void AppBalance_Update(void);
float AppBalance_GetOutput(void);

#endif
