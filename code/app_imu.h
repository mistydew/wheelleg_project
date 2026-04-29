#ifndef APP_IMU_H
#define APP_IMU_H

#include "zf_common_headfile.h"

typedef struct
{
    int16 acc_raw[3]; // 加速度原始值
    int16 gyro_raw[3]; // 陀螺仪原始值

    float gyro_offset[3]; // 陀螺仪零偏
    float gyro_dps[3]; // 陀螺仪度每秒
    float acc_g[3]; // 加速度(以重力加速度为单位)

    uint8 ready;  //等于1为可用
} app_imu_t;

extern app_imu_t g_imu;

void AppImu_Init(void);
void AppImu_UpdateRaw(void);
void AppImu_CalibrateGyroOffset(uint16 samples);
void AppImu_UpdatePhysical(void);

#endif