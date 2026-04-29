#ifndef APP_ATTITUDE_H
#define APP_ATTITUDE_H

#include "zf_common_headfile.h"

typedef struct
{
    float pitch_deg;
    float roll_deg;

    float pitch_zero;
    float roll_zero;

    uint8 ready;
} app_attitude_t;

extern app_attitude_t g_attitude;

void AppAttitude_Init(void);
void AppAttitude_UpdateByAcc(void);
void AppAttitude_SetZero(void);

#endif