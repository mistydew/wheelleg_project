#include "app_quaternion.h"
#include <math.h>

#define APP_QUAT_DEG_TO_RAD        (0.01745329252f)
#define APP_QUAT_RAD_TO_DEG        (57.295779513f)
#define APP_QUAT_ACC_EPS           (1.0e-6f)
#define APP_QUAT_NORM_EPS          (1.0e-6f)
#define APP_QUAT_KP                (2.0f)
#define APP_QUAT_KI                (0.0f)

app_quaternion_t g_quat;

static float AppQuat_Clamp(float value, float min_value, float max_value)
{
    if(value < min_value)
    {
        return min_value;
    }

    if(value > max_value)
    {
        return max_value;
    }

    return value;
}

static void AppQuat_ResetIntegral(void)
{
    g_quat.integral_x = 0.0f;
    g_quat.integral_y = 0.0f;
    g_quat.integral_z = 0.0f;
}

static void AppQuat_NormalizeQuaternion(void)
{
    float norm;
    float inv_norm;

    norm = sqrtf(g_quat.q0 * g_quat.q0 +
                 g_quat.q1 * g_quat.q1 +
                 g_quat.q2 * g_quat.q2 +
                 g_quat.q3 * g_quat.q3);

    if(norm > APP_QUAT_NORM_EPS)
    {
        inv_norm = 1.0f / norm;
        g_quat.q0 *= inv_norm;
        g_quat.q1 *= inv_norm;
        g_quat.q2 *= inv_norm;
        g_quat.q3 *= inv_norm;
    }
    else
    {
        g_quat.q0 = 1.0f;
        g_quat.q1 = 0.0f;
        g_quat.q2 = 0.0f;
        g_quat.q3 = 0.0f;
        AppQuat_ResetIntegral();
    }
}

static void AppQuat_UpdateEuler(void)
{
    float q0;
    float q1;
    float q2;
    float q3;
    float sin_pitch;

    q0 = g_quat.q0;
    q1 = g_quat.q1;
    q2 = g_quat.q2;
    q3 = g_quat.q3;

    g_quat.roll_deg = atan2f(2.0f * (q0 * q1 + q2 * q3),
                             1.0f - 2.0f * (q1 * q1 + q2 * q2)) * APP_QUAT_RAD_TO_DEG;

    sin_pitch = 2.0f * (q0 * q2 - q3 * q1);
    sin_pitch = AppQuat_Clamp(sin_pitch, -1.0f, 1.0f);
    g_quat.pitch_deg = asinf(sin_pitch) * APP_QUAT_RAD_TO_DEG;

    g_quat.pitch_balance_deg = g_quat.pitch_deg - g_quat.pitch_zero;
    g_quat.roll_balance_deg = g_quat.roll_deg - g_quat.roll_zero;

    g_quat.yaw_deg = atan2f(2.0f * (q0 * q3 + q1 * q2),
                            1.0f - 2.0f * (q2 * q2 + q3 * q3)) * APP_QUAT_RAD_TO_DEG;
}

static void AppQuat_SetFromAcc(float ax_g, float ay_g, float az_g)
{
    float pitch_rad;
    float roll_rad;
    float half_pitch;
    float half_roll;
    float cp;
    float sp;
    float cr;
    float sr;

    roll_rad = atan2f(ay_g, az_g);
    pitch_rad = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g));

    half_pitch = 0.5f * pitch_rad;
    half_roll = 0.5f * roll_rad;

    cp = cosf(half_pitch);
    sp = sinf(half_pitch);
    cr = cosf(half_roll);
    sr = sinf(half_roll);

    g_quat.q0 = cr * cp;
    g_quat.q1 = sr * cp;
    g_quat.q2 = cr * sp;
    g_quat.q3 = -sr * sp;

    AppQuat_ResetIntegral();
    AppQuat_NormalizeQuaternion();
    g_quat.ready = 1;
    AppQuat_UpdateEuler();
}

void AppQuat_Init(void)
{
    g_quat.q0 = 1.0f;
    g_quat.q1 = 0.0f;
    g_quat.q2 = 0.0f;
    g_quat.q3 = 0.0f;

    g_quat.pitch_deg = 0.0f;
    g_quat.roll_deg = 0.0f;
    g_quat.yaw_deg = 0.0f;
    g_quat.pitch_zero = 0.0f;
    g_quat.roll_zero = 0.0f;
    g_quat.pitch_balance_deg = 0.0f;
    g_quat.roll_balance_deg = 0.0f;

    AppQuat_ResetIntegral();
    g_quat.ready = 0;
}

void AppQuat_SetZero(void)
{
    g_quat.pitch_zero = g_quat.pitch_deg;
    g_quat.roll_zero = g_quat.roll_deg;
    g_quat.pitch_balance_deg = 0.0f;
    g_quat.roll_balance_deg = 0.0f;
}

void AppQuat_Update(float gx_dps, float gy_dps, float gz_dps,
                    float ax_g, float ay_g, float az_g,
                    float dt_s)
{
    float gx;
    float gy;
    float gz;
    float ax;
    float ay;
    float az;
    float acc_norm;
    float inv_acc_norm;
    float vx;
    float vy;
    float vz;
    float ex;
    float ey;
    float ez;
    float q0;
    float q1;
    float q2;
    float q3;
    float half_dt;
    uint8 acc_valid;

    if(dt_s <= 0.0f)
    {
        return;
    }

    ax = ax_g;
    ay = ay_g;
    az = az_g;
    acc_valid = 0;

    acc_norm = sqrtf(ax * ax + ay * ay + az * az);
    if(acc_norm > APP_QUAT_ACC_EPS)
    {
        inv_acc_norm = 1.0f / acc_norm;
        ax *= inv_acc_norm;
        ay *= inv_acc_norm;
        az *= inv_acc_norm;
        acc_valid = 1;
    }

    if(!g_quat.ready)
    {
        if(acc_valid)
        {
            AppQuat_SetFromAcc(ax, ay, az);
        }
        return;
    }

    gx = gx_dps * APP_QUAT_DEG_TO_RAD;
    gy = gy_dps * APP_QUAT_DEG_TO_RAD;
    gz = gz_dps * APP_QUAT_DEG_TO_RAD;

    if(acc_valid)
    {
        vx = 2.0f * (g_quat.q1 * g_quat.q3 - g_quat.q0 * g_quat.q2);
        vy = 2.0f * (g_quat.q0 * g_quat.q1 + g_quat.q2 * g_quat.q3);
        vz = g_quat.q0 * g_quat.q0 - g_quat.q1 * g_quat.q1 -
             g_quat.q2 * g_quat.q2 + g_quat.q3 * g_quat.q3;

        ex = ay * vz - az * vy;
        ey = az * vx - ax * vz;
        ez = ax * vy - ay * vx;

        if(APP_QUAT_KI > 0.0f)
        {
            g_quat.integral_x += APP_QUAT_KI * ex * dt_s;
            g_quat.integral_y += APP_QUAT_KI * ey * dt_s;
            g_quat.integral_z += APP_QUAT_KI * ez * dt_s;

            gx += g_quat.integral_x;
            gy += g_quat.integral_y;
            gz += g_quat.integral_z;
        }
        else
        {
            AppQuat_ResetIntegral();
        }

        gx += APP_QUAT_KP * ex;
        gy += APP_QUAT_KP * ey;
        gz += APP_QUAT_KP * ez;
    }
    else
    {
        AppQuat_ResetIntegral();
    }

    q0 = g_quat.q0;
    q1 = g_quat.q1;
    q2 = g_quat.q2;
    q3 = g_quat.q3;
    half_dt = 0.5f * dt_s;

    g_quat.q0 += (-q1 * gx - q2 * gy - q3 * gz) * half_dt;
    g_quat.q1 += ( q0 * gx + q2 * gz - q3 * gy) * half_dt;
    g_quat.q2 += ( q0 * gy - q1 * gz + q3 * gx) * half_dt;
    g_quat.q3 += ( q0 * gz + q1 * gy - q2 * gx) * half_dt;

    AppQuat_NormalizeQuaternion();
    AppQuat_UpdateEuler();
}
