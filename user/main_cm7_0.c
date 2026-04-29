#include "zf_common_headfile.h"
#include "app_imu.h"
#include "app_quaternion.h"
#include "app_safety.h"

#define DEBUG_PRINT_MODE_WAVE 1
#define QUAT_ZERO_STABLE_MS   (1000u)

volatile uint8 flag_1ms   = 0;
volatile uint8 flag_2ms   = 0;
volatile uint8 flag_10ms  = 0;
volatile uint8 flag_100ms = 0;

int main(void)
{
    uint8 quat_zero_done = 0;
    uint16 quat_zero_count = 0;

    clock_init(SYSTEM_CLOCK_250M);
    debug_init();

#if (DEBUG_PRINT_MODE_WAVE == 0)
    printf("wheel leg main start\r\n");
#endif

    AppImu_Init();
    AppQuat_Init();
    AppSafety_Init();

    pit_ms_init(PIT_CH0, 1);

    while(true)
    {
        if(flag_1ms)
        {
            flag_1ms = 0;

            AppImu_UpdateRaw();
            AppImu_UpdatePhysical();
            AppQuat_Update(g_imu.gyro_dps[0], g_imu.gyro_dps[1], g_imu.gyro_dps[2],
                           g_imu.acc_g[0], g_imu.acc_g[1], g_imu.acc_g[2],
                           0.001f);

            if(!quat_zero_done)
            {
                if(g_imu.ready && g_quat.ready)
                {
                    if(quat_zero_count < QUAT_ZERO_STABLE_MS)
                    {
                        quat_zero_count++;
                    }
                    else
                    {
                        AppQuat_SetZero();
                        quat_zero_done = 1;
                    }
                }
                else
                {
                    quat_zero_count = 0;
                }
            }
        }

        if(flag_2ms)
        {
            flag_2ms = 0;
        }

        if(flag_10ms)
        {
            flag_10ms = 0;
            AppSafety_Update();

#if (DEBUG_PRINT_MODE_WAVE == 1)
            printf("%d,%d,%d,%d\r\n",
                   (int)g_quat.pitch_balance_deg,
                   (int)g_quat.roll_balance_deg,
                   (int)g_quat.yaw_deg,
                   (int)AppSafety_IsSafe());
#endif
        }

        if(flag_100ms)
        {
            flag_100ms = 0;

#if (DEBUG_PRINT_MODE_WAVE == 0)
            printf("ready:%d quat:%d pitch:%d roll:%d pitch_balance:%d roll_balance:%d yaw:%d gyro_dps:%d,%d,%d acc_mg:%d,%d,%d\r\n",
                   g_imu.ready,
                   g_quat.ready,
                   (int)g_quat.pitch_deg,
                   (int)g_quat.roll_deg,
                   (int)g_quat.pitch_balance_deg,
                   (int)g_quat.roll_balance_deg,
                   (int)g_quat.yaw_deg,
                   (int)g_imu.gyro_dps[0],
                   (int)g_imu.gyro_dps[1],
                   (int)g_imu.gyro_dps[2],
                   (int)(g_imu.acc_g[0] * 1000.0f),
                   (int)(g_imu.acc_g[1] * 1000.0f),
                   (int)(g_imu.acc_g[2] * 1000.0f));
#endif
        }
    }
}

void uart_rx_interrupt_handler(void)
{
}
