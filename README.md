# CYT4BB7 轮腿小车平衡调试工程

本工程基于 CYT4BB7 + IAR + 逐飞库，用于轮腿小车的 IMU 姿态采集、四元数姿态解算、安全保护、电机驱动通信和平衡控制调试。

当前重点是轮式平衡控制调试，不包含腿部 IK、完整速度 PID 或速度环接腿。

## 当前状态

- IMU 初始化、原始数据读取和物理量换算已接入。
- 四元数姿态解算已接入，输出 `pitch_balance_deg` / `roll_balance_deg`。
- 上电后等待姿态稳定约 1000ms 自动调用 `AppQuat_SetZero()`。
- `app_safety` 已接入，pitch / roll 超过 `±45°` 或 IMU/quaternion 未 ready 时进入 unsafe。
- 电机驱动通信已接入，UART4 接收回调解析小驱速度反馈。
- 平衡控制已接入：角度 P + 角速度 D + 轮速阻尼。
- 左右轮速度反馈符号已统一，`forward_speed` 能反映车体前后运动。

## 主要文件

```text
code/app_imu.c/.h          IMU 初始化、原始数据和物理量
code/app_quaternion.c/.h   四元数姿态解算和零点
code/app_safety.c/.h       姿态安全保护
code/app_motor.c/.h        小驱电机输出和速度反馈接口
code/app_balance.c/.h      平衡控制和调试变量
user/main_cm7_0.c          主循环、串口输出、调参命令
user/cm7_0_isr.c           PIT 和 UART 中断
```

## 平衡控制结构

当前 `AppBalance_Update()` 的核心逻辑：

```c
left_speed = SPEED_LEFT_SIGN * (float)AppMotor_GetLeftSpeed();
right_speed = SPEED_RIGHT_SIGN * (float)AppMotor_GetRightSpeed();
g_balance.forward_speed = 0.5f * (left_speed + right_speed);

g_balance.angle_output = g_balance.kp_angle * g_balance.pitch_error_deg -
                         g_balance.kd_gyro * g_balance.gyro_pitch_dps;
g_balance.speed_output = -g_balance.kv_speed * g_balance.forward_speed;
output = g_balance.angle_output + g_balance.speed_output;

g_balance.output = AppBalance_Limit(output, g_balance.output_limit);
```

速度反馈符号定义：

```c
#define SPEED_LEFT_SIGN            (-1.0f)
#define SPEED_RIGHT_SIGN           ( 1.0f)
```

注意：这些宏只用于速度反馈方向统一，不用于电机输出方向。

## 当前默认参数

默认值位于 `code/app_balance.c` 的 `AppBalance_Init()`：

```c
g_balance.kp_angle = 21.34f;
g_balance.kd_gyro = 0.896f;
g_balance.pitch_target_deg = -0.6f;
g_balance.kv_speed = 0.8f;
g_balance.output_limit = BALANCE_OUTPUT_LIMIT;
```

相关含义：

```text
pitch_target_deg   平衡目标角 / 机械零点补偿
kp_angle           角度回正力度
kd_gyro            角速度阻尼
kv_speed           速度阻尼
output_limit       平衡输出限幅
```

安全角度限制在 `code/app_safety.c`：

```c
#define APP_SAFETY_ANGLE_LIMIT_DEG  (45.0f)
```

## 串口波形输出

当前 `DEBUG_PRINT_MODE` 默认为 `DEBUG_PRINT_MODE_WAVE`，10ms 输出一次纯数字，方便串口示波器解析。

当前格式：

```text
pitch_balance,forward_speed,angle_output,speed_output_x100,output,safe
```

对应代码：

```c
printf("%d,%d,%d,%d,%d,%d\r\n",
       (int)g_quat.pitch_balance_deg,
       (int)g_balance.forward_speed,
       (int)g_balance.angle_output,
       (int)(g_balance.speed_output * 100.0f),
       (int)g_balance.output,
       (int)AppSafety_IsSafe());
```

示例解读：

```text
-24,-52,520,4160,561,1
```

```text
pitch_balance     = -24 deg
forward_speed     = -52
angle_output      = 520
speed_output_x100 = 4160, 即 speed_output 约 41.6
output            = 561
safe              = 1
```

## 调试判断

- 车向前跑时，`forward_speed` 应明显为正。
- 车向前跑时，`speed_output` 应为负，用来抑制向前跑。
- 车向后跑时，`forward_speed` 应为负。
- `speed_output_x100` 是速度项乘 100 后的整数，实际速度项需要除以 100。
- 如果 `angle_output` 和 `speed_output` 一正一负，说明角度环和速度阻尼正在对抗。
- 如果 `output` 长期贴近 `±output_limit`，说明输出已经限幅。

调参顺序建议：

```text
1. 先确认速度方向，不要反转电机方向。
2. 小步调整 pitch_target_deg 补机械零点。
3. 逐步调整 kv_speed，让速度阻尼明显但不抽搐。
4. 如果 output 经常打满，再考虑提高 output_limit。
5. 如果加 kv_speed 后来回抽搐，回退 kv_speed 或降低 kp_angle。
```

## 电机方向说明

平衡输出到电机的路径在 `user/main_cm7_0.c`：

```c
motor_cmd = (float)BALANCE_MOTOR_SIGN * g_balance.output;
AppMotor_SetDuty((int)motor_cmd, (int)motor_cmd);
```

左右电机安装方向在 `code/app_motor.h`：

```c
#define MOTOR_LEFT_SIGN          (1)
#define MOTOR_RIGHT_SIGN         (-1)
```

不要把速度反馈符号 `SPEED_LEFT_SIGN / SPEED_RIGHT_SIGN` 和电机输出方向 `MOTOR_LEFT_SIGN / MOTOR_RIGHT_SIGN` 混在一起。

## 调参命令模式

代码中保留了 `DEBUG_PRINT_MODE_TUNER`。切换后可使用 `WL,...` 命令调参：

```text
WL,PID,kp_x100,kd_x1000,kv_x100000
WL,TARGET,target_x100
WL,LIMIT,output_limit
WL,STOP
WL,ARM
WL,ZERO
WL,GET
```

当前默认是 `DEBUG_PRINT_MODE_WAVE`，该模式下不会主动输出 `WL,HELLO`，避免干扰串口示波器解析。

## 当前进度

- [x] IMU 数据采集
- [x] 四元数姿态解算
- [x] 姿态零点校准
- [x] 姿态安全保护
- [x] 小驱 UART 通信
- [x] 电机方向验证
- [x] 左右轮速度反馈符号统一
- [x] 角度 P + 角速度 D 平衡控制
- [x] 速度阻尼项
- [x] 串口波形输出拆分
- [ ] 单侧电机链路诊断
- [ ] 长时间稳定调参
- [ ] 腿部机构控制
