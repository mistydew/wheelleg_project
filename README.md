# CYT4BB7 轮腿小车姿态测试工程

本工程基于 CYT4BB7、IAR 和逐飞库，用于轮腿小车的 IMU 姿态采集、四元数姿态解算、姿态零点校准和姿态安全保护测试。

## 已实现功能

### 1. IMU 数据采集

- 使用 IMU963RA 作为姿态传感器。
- 已完成 IMU 初始化和 ready 状态判断。
- 已能读取加速度计和陀螺仪原始数据。
- 已能转换得到物理量：
  - `gyro_dps[3]`
  - `acc_g[3]`

### 2. 四元数姿态解算

- 新增并使用 `app_quaternion` 模块。
- 已能根据陀螺仪和加速度数据更新四元数。
- 已能输出欧拉角：
  - `pitch_deg`
  - `roll_deg`
  - `yaw_deg`

### 3. 姿态零点功能

- `app_quaternion_t` 已增加零点和相对平衡角字段：
  - `pitch_zero`
  - `roll_zero`
  - `pitch_balance_deg`
  - `roll_balance_deg`
- 已提供接口：
  - `AppQuat_SetZero()`
- 主程序在 `g_imu.ready` 和 `g_quat.ready` 后，等待四元数稳定约 `1000ms`，自动置零一次。
- 置零后串口主要观察：
  - `pitch_balance_deg`
  - `roll_balance_deg`
  - `yaw_deg`

### 4. 姿态安全保护

- 新增 `app_safety` 模块：
  - `app_safety.h`
  - `app_safety.c`
- 已提供接口：
  - `AppSafety_Init()`
  - `AppSafety_Update()`
  - `AppSafety_IsSafe()`
- 当前安全判断条件：
  - `g_imu.ready == 0` 时 unsafe
  - `g_quat.ready == 0` 时 unsafe
  - `pitch_balance_deg > 45` 或 `< -45` 时 unsafe
  - `roll_balance_deg > 45` 或 `< -45` 时 unsafe
  - 其他情况 safe
- `safe = 1` 表示安全，`safe = 0` 表示不安全。

### 5. 串口示波器输出

当前 10ms 任务中输出四个纯数字，格式如下：

```text
pitch_balance_deg,roll_balance_deg,yaw_deg,safe
```

示例：

```text
0,1,-10,1
52,3,-10,0
```

## 当前进度

- [x] 工程基础启动流程
- [x] IMU963RA 初始化和 ready 判断
- [x] IMU 原始数据读取
- [x] 加速度和陀螺仪物理量转换
- [x] 四元数姿态解算
- [x] pitch、roll、yaw 输出
- [x] 姿态零点自动校准
- [x] pitch_balance、roll_balance 输出
- [x] 姿态安全保护模块
- [x] 串口示波器四路数字输出
- [ ] PID 控制
- [ ] 电机驱动控制
- [ ] 舵机控制
- [ ] 轮腿平衡控制
- [ ] 安全保护与执行机构停机联动

## 当前说明

当前工程仍处于姿态测试和安全状态验证阶段，暂未接入 PID、电机驱动和舵机控制。后续控制代码建议在姿态输出稳定、零点校准可靠、安全保护状态验证完成后再接入。
