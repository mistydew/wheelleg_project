#include "app_balance.h"
#include "app_imu.h"
#include "app_quaternion.h"
#include "app_safety.h"

app_balance_t g_balance;

static float AppBalance_Limit(float value, float limit)
{
    if(value > limit)
    {
        return limit;
    }

    if(value < -limit)
    {
        return -limit;
    }

    return value;
}

static void AppBalance_ApplyMotorOutput(float output)
{
#if (MOTOR_OUTPUT_ENABLE)
    (void)output;
    /* Motor output is intentionally not implemented in the first balance frame. */
#else
    (void)output;
#endif
}

void AppBalance_Init(void)
{
    g_balance.kp_angle = 10.0f;
    g_balance.kd_gyro = 0.3f;
    g_balance.pitch_target_deg = 0.0f;
    g_balance.pitch_error_deg = 0.0f;
    g_balance.gyro_pitch_dps = 0.0f;
    g_balance.output = 0.0f;
    g_balance.output_limit = 1000.0f;
    g_balance.ready = 1;

    AppBalance_ApplyMotorOutput(0.0f);
}

void AppBalance_Update(void)
{
    float output;

    if(!g_balance.ready)
    {
        g_balance.output = 0.0f;
        AppBalance_ApplyMotorOutput(0.0f);
        return;
    }

    g_balance.gyro_pitch_dps = g_imu.gyro_dps[BALANCE_GYRO_PITCH_AXIS];
    g_balance.pitch_error_deg = g_balance.pitch_target_deg - g_quat.pitch_balance_deg;

    if(!AppSafety_IsSafe())
    {
        g_balance.output = 0.0f;
        AppBalance_ApplyMotorOutput(0.0f);
        return;
    }

    output = g_balance.kp_angle * g_balance.pitch_error_deg -
             g_balance.kd_gyro * g_balance.gyro_pitch_dps;

    g_balance.output = AppBalance_Limit(output, g_balance.output_limit);
    AppBalance_ApplyMotorOutput(g_balance.output);
}

float AppBalance_GetOutput(void)
{
    return g_balance.output;
}
