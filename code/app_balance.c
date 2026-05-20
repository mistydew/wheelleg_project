#include "app_balance.h"
#include "app_imu.h"
#include "app_quaternion.h"
#include "app_safety.h"
#include "app_motor.h"

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
    (void)output;
}

void AppBalance_Init(void)
{
    g_balance.kp_angle = 21.34f;
    g_balance.kd_gyro = 0.896f;
    g_balance.pitch_target_deg = -0.6f;
    g_balance.pitch_error_deg = 0.0f;
    g_balance.gyro_pitch_dps = 0.0f;
    g_balance.angle_output = 0.0f;
    g_balance.forward_speed = 0.0f;
    /* Speed feedback signs are unified below, so kv_speed must be re-tuned per feedback unit. */
    g_balance.kv_speed = 0.8f;
    g_balance.speed_output = 0.0f;
    g_balance.output = 0.0f;
    g_balance.output_limit = BALANCE_OUTPUT_LIMIT;
    g_balance.ready = 1;

    AppBalance_ApplyMotorOutput(0.0f);
}

void AppBalance_Update(void)
{
    float output;
    float left_speed;
    float right_speed;

    if(!g_balance.ready)
    {
        g_balance.angle_output = 0.0f;
        g_balance.forward_speed = 0.0f;
        g_balance.speed_output = 0.0f;
        g_balance.output = 0.0f;
        AppBalance_ApplyMotorOutput(0.0f);
        return;
    }

    g_balance.gyro_pitch_dps = g_imu.gyro_dps[BALANCE_GYRO_PITCH_AXIS];
    g_balance.pitch_error_deg = g_balance.pitch_target_deg - g_quat.pitch_balance_deg;

    if(!AppSafety_IsSafe())
    {
        g_balance.angle_output = 0.0f;
        g_balance.forward_speed = 0.0f;
        g_balance.speed_output = 0.0f;
        g_balance.output = 0.0f;
        AppBalance_ApplyMotorOutput(0.0f);
        return;
    }

    left_speed = SPEED_LEFT_SIGN * (float)AppMotor_GetLeftSpeed();
    right_speed = SPEED_RIGHT_SIGN * (float)AppMotor_GetRightSpeed();
    g_balance.forward_speed = 0.5f * (left_speed + right_speed);

    g_balance.angle_output = g_balance.kp_angle * g_balance.pitch_error_deg -
                             g_balance.kd_gyro * g_balance.gyro_pitch_dps;
    g_balance.speed_output = -g_balance.kv_speed * g_balance.forward_speed;
    output = g_balance.angle_output + g_balance.speed_output;

    /* If output_limit is 600 and output stays at +/-600, test 800 or 1000 next. */
    g_balance.output = AppBalance_Limit(output, g_balance.output_limit);
    AppBalance_ApplyMotorOutput(g_balance.output);
}

float AppBalance_GetOutput(void)
{
    return g_balance.output;
}
