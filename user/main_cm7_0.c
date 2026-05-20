#include "zf_common_headfile.h"
#include "app_imu.h"
#include "app_quaternion.h"
#include "app_safety.h"
#include "app_balance.h"
#include "app_servo.h"
#include "app_motor.h"

#define DEBUG_PRINT_MODE_WAVE      (1)
#define DEBUG_PRINT_MODE_IMU_RAW   (2)
#define DEBUG_PRINT_MODE_TUNER     (3)
#define DEBUG_PRINT_MODE           DEBUG_PRINT_MODE_WAVE
#define QUAT_ZERO_STABLE_MS   (1000u)
#define TUNER_RX_LINE_MAX     (96u)

volatile uint8 flag_1ms   = 0;
volatile uint8 flag_2ms   = 0;
volatile uint8 flag_10ms  = 0;
volatile uint8 flag_100ms = 0;

static volatile uint8 tuner_command_ready = 0;
static uint8 tuner_rx_count = 0;
static char tuner_rx_line[TUNER_RX_LINE_MAX];
static char tuner_command_line[TUNER_RX_LINE_MAX];
static uint32 tuner_time_ms = 0;

static int Tuner_ScaleFloat(float value, float scale)
{
    float scaled = value * scale;

    if(scaled >= 0.0f)
    {
        return (int)(scaled + 0.5f);
    }

    return (int)(scaled - 0.5f);
}

static int Tuner_LimitInt(int value, int min_value, int max_value)
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

static void Tuner_SendAck(const char *name)
{
    printf("WL,ACK,%s\r\n", name);
}

static void Tuner_SendErr(const char *reason)
{
    printf("WL,ERR,%s\r\n", reason);
}

static void Tuner_SendTelemetry(void)
{
    printf("WL,T,%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
           (unsigned long)tuner_time_ms,
           Tuner_ScaleFloat(g_quat.pitch_balance_deg, 100.0f),
           Tuner_ScaleFloat(g_balance.gyro_pitch_dps, 10.0f),
           (int)g_balance.forward_speed,
           (int)g_balance.output,
           (int)AppMotor_GetLeftSpeed(),
           (int)AppMotor_GetRightSpeed(),
           (int)AppSafety_IsSafe(),
           Tuner_ScaleFloat(g_balance.kp_angle, 100.0f),
           Tuner_ScaleFloat(g_balance.kd_gyro, 1000.0f),
           Tuner_ScaleFloat(g_balance.kv_speed, 100000.0f),
           Tuner_ScaleFloat(g_balance.pitch_target_deg, 100.0f),
           (int)g_balance.output_limit);
}

static void Tuner_HandleCommand(char *line)
{
    int value_a = 0;
    int value_b = 0;
    int value_c = 0;

    if(0 == strncmp(line, "WL,PID,", 7))
    {
        if(3 == sscanf(line + 7, "%d,%d,%d", &value_a, &value_b, &value_c))
        {
            value_a = Tuner_LimitInt(value_a, 0, 8000);
            value_b = Tuner_LimitInt(value_b, 0, 6000);
            value_c = Tuner_LimitInt(value_c, 0, 8000);

            g_balance.kp_angle = (float)value_a / 100.0f;
            g_balance.kd_gyro = (float)value_b / 1000.0f;
            g_balance.kv_speed = (float)value_c / 100000.0f;
            Tuner_SendAck("PID");
        }
        else
        {
            Tuner_SendErr("PID_FORMAT");
        }
        return;
    }

    if(0 == strncmp(line, "WL,TARGET,", 10))
    {
        if(1 == sscanf(line + 10, "%d", &value_a))
        {
            value_a = Tuner_LimitInt(value_a, -1200, 1200);
            g_balance.pitch_target_deg = (float)value_a / 100.0f;
            Tuner_SendAck("TARGET");
        }
        else
        {
            Tuner_SendErr("TARGET_FORMAT");
        }
        return;
    }

    if(0 == strncmp(line, "WL,LIMIT,", 9))
    {
        if(1 == sscanf(line + 9, "%d", &value_a))
        {
            value_a = Tuner_LimitInt(value_a, 500, 9000);
            g_balance.output_limit = (float)value_a;
            Tuner_SendAck("LIMIT");
        }
        else
        {
            Tuner_SendErr("LIMIT_FORMAT");
        }
        return;
    }

    if(0 == strcmp(line, "WL,STOP"))
    {
        g_balance.ready = 0;
        g_balance.pitch_target_deg = 0.0f;
        g_balance.output = 0.0f;
        AppMotor_StopAll();
        Tuner_SendAck("STOP");
        return;
    }

    if(0 == strcmp(line, "WL,ARM"))
    {
        g_balance.ready = 1;
        Tuner_SendAck("ARM");
        return;
    }

    if(0 == strcmp(line, "WL,ZERO"))
    {
        AppQuat_SetZero();
        Tuner_SendAck("ZERO");
        return;
    }

    if(0 == strcmp(line, "WL,PING"))
    {
        Tuner_SendAck("PING");
        return;
    }

    if(0 == strcmp(line, "WL,GET"))
    {
        Tuner_SendTelemetry();
        return;
    }

    Tuner_SendErr("UNKNOWN");
}

static void Tuner_ProcessCommand(void)
{
    char line[TUNER_RX_LINE_MAX];

    if(!tuner_command_ready)
    {
        return;
    }

    memcpy(line, tuner_command_line, TUNER_RX_LINE_MAX);
    line[TUNER_RX_LINE_MAX - 1u] = '\0';
    tuner_command_ready = 0;
    Tuner_HandleCommand(line);
}

int main(void)
{
    uint8 quat_zero_done = 0;
    uint16 quat_zero_count = 0;

    clock_init(SYSTEM_CLOCK_250M);
    debug_init();

#if (DEBUG_PRINT_MODE != DEBUG_PRINT_MODE_WAVE)
    printf("wheel leg main start\r\n");
#endif

    AppImu_Init();
    AppQuat_Init();
    AppSafety_Init();
    AppServo_Init();
    AppServo_SetNeutral();
    AppBalance_Init();
    AppMotor_Init();
    AppMotor_StopAll();

    pit_ms_init(PIT_CH0, 1);
#if (DEBUG_PRINT_MODE != DEBUG_PRINT_MODE_WAVE)
    printf("WL,HELLO,CYT4BB7,WHEEL_LEG,1\r\n");
#endif

    while(true)
    {
#if (DEBUG_PRINT_MODE == DEBUG_PRINT_MODE_TUNER)
        Tuner_ProcessCommand();
#endif

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
            float motor_cmd;

            flag_2ms = 0;
            AppSafety_Update();
            AppBalance_Update();

            if(AppSafety_IsSafe())
            {
                motor_cmd = (float)BALANCE_MOTOR_SIGN * g_balance.output;
                AppMotor_SetDuty((int)motor_cmd, (int)motor_cmd);
            }
            else
            {
                AppMotor_StopAll();
            }
        }

        if(flag_10ms)
        {
            flag_10ms = 0;
            tuner_time_ms += 10u;
            AppMotor_RequestSpeedFeedback();

#if (DEBUG_PRINT_MODE == DEBUG_PRINT_MODE_TUNER)
            Tuner_SendTelemetry();
#elif (DEBUG_PRINT_MODE == DEBUG_PRINT_MODE_WAVE)
            printf("%d,%d,%d,%d,%d,%d\r\n",
                   (int)g_quat.pitch_balance_deg,
                   (int)g_balance.forward_speed,
                   (int)g_balance.angle_output,
                   (int)(g_balance.speed_output * 100.0f),
                   (int)g_balance.output,
                   (int)AppSafety_IsSafe());
#endif
        }

        if(flag_100ms)
        {
            flag_100ms = 0;

#if (DEBUG_PRINT_MODE == DEBUG_PRINT_MODE_IMU_RAW)
            printf("ready:%d acc_raw:%d,%d,%d gyro_raw:%d,%d,%d acc_mg:%d,%d,%d gyro_dps:%d,%d,%d\r\n",
                   g_imu.ready,
                   g_imu.acc_raw[0],
                   g_imu.acc_raw[1],
                   g_imu.acc_raw[2],
                   g_imu.gyro_raw[0],
                   g_imu.gyro_raw[1],
                   g_imu.gyro_raw[2],
                   (int)(g_imu.acc_g[0] * 1000.0f),
                   (int)(g_imu.acc_g[1] * 1000.0f),
                   (int)(g_imu.acc_g[2] * 1000.0f),
                   (int)g_imu.gyro_dps[0],
                   (int)g_imu.gyro_dps[1],
                   (int)g_imu.gyro_dps[2]);
#endif
        }
    }
}

void uart_rx_interrupt_handler(void)
{
    uint8 data;
    uint8 i;

    while(uart_query_byte(DEBUG_UART_INDEX, &data))
    {
        if((data == '\r') || (data == '\n'))
        {
            if(tuner_rx_count > 0u)
            {
                tuner_rx_line[tuner_rx_count] = '\0';
                if(!tuner_command_ready)
                {
                    for(i = 0u; i < TUNER_RX_LINE_MAX; i++)
                    {
                        tuner_command_line[i] = tuner_rx_line[i];
                        if('\0' == tuner_rx_line[i])
                        {
                            break;
                        }
                    }
                    tuner_command_ready = 1;
                }
                tuner_rx_count = 0u;
            }
            continue;
        }

        if(tuner_rx_count < (TUNER_RX_LINE_MAX - 1u))
        {
            tuner_rx_line[tuner_rx_count] = (char)data;
            tuner_rx_count++;
        }
        else
        {
            tuner_rx_count = 0u;
        }
    }
}
