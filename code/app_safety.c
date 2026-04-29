#include "app_safety.h"
#include "app_imu.h"
#include "app_quaternion.h"

#define APP_SAFETY_ANGLE_LIMIT_DEG  (45.0f)

static uint8 safe;

void AppSafety_Init(void)
{
    safe = 0;
}

void AppSafety_Update(void)
{
    if(!g_imu.ready)
    {
        safe = 0;
        return;
    }

    if(!g_quat.ready)
    {
        safe = 0;
        return;
    }

    if((g_quat.pitch_balance_deg > APP_SAFETY_ANGLE_LIMIT_DEG) ||
       (g_quat.pitch_balance_deg < -APP_SAFETY_ANGLE_LIMIT_DEG))
    {
        safe = 0;
        return;
    }

    if((g_quat.roll_balance_deg > APP_SAFETY_ANGLE_LIMIT_DEG) ||
       (g_quat.roll_balance_deg < -APP_SAFETY_ANGLE_LIMIT_DEG))
    {
        safe = 0;
        return;
    }

    safe = 1;
}

uint8 AppSafety_IsSafe(void)
{
    return safe;
}
