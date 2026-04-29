#include "app_attitude.h"
#include "app_imu.h"
#include <math.h>

#define RAD_TO_DEG   (57.2957795f)

app_attitude_t g_attitude;

void AppAttitude_Init(void)
{
    g_attitude.pitch_deg = 0.0f;
    g_attitude.roll_deg = 0.0f;
    g_attitude.pitch_zero = 0.0f;
    g_attitude.roll_zero = 0.0f;
    g_attitude.ready = 1;
}

void AppAttitude_UpdateByAcc(void)
{
    float ax = g_imu.acc_g[0];
    float ay = g_imu.acc_g[1];
    float az = g_imu.acc_g[2];

    // 第一版静态估算，主要用于验证轴向
    g_attitude.roll_deg  = atan2f(ay, az) * RAD_TO_DEG - g_attitude.roll_zero;
    g_attitude.pitch_deg = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG - g_attitude.pitch_zero;
}

void AppAttitude_SetZero(void)
{
    float ax = g_imu.acc_g[0];
    float ay = g_imu.acc_g[1];
    float az = g_imu.acc_g[2];

    g_attitude.roll_zero  = atan2f(ay, az) * RAD_TO_DEG;
    g_attitude.pitch_zero = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
}