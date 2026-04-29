#ifndef APP_QUATERNION_H
#define APP_QUATERNION_H

#include "zf_common_headfile.h"

typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;

    float pitch_deg;
    float roll_deg;
    float yaw_deg;

    float pitch_zero;
    float roll_zero;
    float pitch_balance_deg;
    float roll_balance_deg;

    float integral_x;
    float integral_y;
    float integral_z;

    uint8 ready;
} app_quaternion_t;

extern app_quaternion_t g_quat;

void AppQuat_Init(void);
void AppQuat_SetZero(void);
void AppQuat_Update(float gx_dps, float gy_dps, float gz_dps,
                    float ax_g, float ay_g, float az_g,
                    float dt_s);

#endif
