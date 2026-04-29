#include "app_imu.h"

app_imu_t g_imu;

void AppImu_Init(void)
{
    uint8 ret;

    ret = imu963ra_init();

    printf("imu963ra_init ret = %d\r\n", ret);
    printf("IMU963RA mode: soft IIC\r\n");

    if(ret == 0)
    {
        printf("IMU963RA init success\r\n");
        AppImu_CalibrateGyroOffset(1000);
        g_imu.ready = 1;
    }
    else
    {
        printf("IMU963RA init failed, skip imu update\r\n");
        g_imu.ready = 0;
    }
}

void AppImu_UpdateRaw(void)
{
    imu963ra_get_acc();
    imu963ra_get_gyro();

    g_imu.acc_raw[0] = imu963ra_acc_x;
    g_imu.acc_raw[1] = imu963ra_acc_y;
    g_imu.acc_raw[2] = imu963ra_acc_z;

    g_imu.gyro_raw[0] = imu963ra_gyro_x;
    g_imu.gyro_raw[1] = imu963ra_gyro_y;
    g_imu.gyro_raw[2] = imu963ra_gyro_z;
}

void AppImu_CalibrateGyroOffset(uint16 samples)
{
    uint16 i;
    float sum[3] = {0};

    printf("gyro offset calibrating...\r\n");

    for(i = 0; i < samples; i++)
    {
        imu963ra_get_gyro();

        sum[0] += imu963ra_gyro_transition(imu963ra_gyro_x);
        sum[1] += imu963ra_gyro_transition(imu963ra_gyro_y);
        sum[2] += imu963ra_gyro_transition(imu963ra_gyro_z);

        system_delay_ms(1);
    }

    g_imu.gyro_offset[0] = sum[0] / samples;
    g_imu.gyro_offset[1] = sum[1] / samples;
    g_imu.gyro_offset[2] = sum[2] / samples;

    printf("gyro offset done\r\n");
}

void AppImu_UpdatePhysical(void)
{
    g_imu.acc_g[0] = imu963ra_acc_transition(g_imu.acc_raw[0]);
    g_imu.acc_g[1] = imu963ra_acc_transition(g_imu.acc_raw[1]);
    g_imu.acc_g[2] = imu963ra_acc_transition(g_imu.acc_raw[2]);

    g_imu.gyro_dps[0] = imu963ra_gyro_transition(g_imu.gyro_raw[0]) - g_imu.gyro_offset[0];
    g_imu.gyro_dps[1] = imu963ra_gyro_transition(g_imu.gyro_raw[1]) - g_imu.gyro_offset[1];
    g_imu.gyro_dps[2] = imu963ra_gyro_transition(g_imu.gyro_raw[2]) - g_imu.gyro_offset[2];
}