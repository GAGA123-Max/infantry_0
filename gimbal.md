# gimbal

四全向轮步兵的云台应用：**单 yaw(DM4310) + 单 pitch(DM4310)**，两轴均走 `DMMotor` 接口。

> 从哨兵（大yaw DM4310 + 小yaw GM6020 双轴）迁移而来。巡逻、正弦扫描、变速小陀螺等哨兵逻辑已全部移除。

## 总体说明

初始化 yaw/pitch 两个 DM4310 和一个 IMU。订阅 `gimbal_cmd` 话题（来自 robot_cmd），发布 `gimbal_feed` 话题。

控制策略：**yaw 始终锁 IMU 绝对角（世界坐标系）**，底盘模式（跟随 / 不跟随 / 小陀螺）由 robot_cmd 决定，云台侧不区分——三档下云台行为完全一致，只是底盘在"追"云台指向。

## 单位约定（重要）

两轴的目标值与反馈值**统一使用弧度制**，这是本应用最容易踩坑的地方：

| 量 | 单位 | 来源 |
|---|---|---|
| `yaw` 角反馈 | rad | IMU `YawTotalAngle`(原生度) × `DEGREE_2_RAD` |
| `yaw` 速度反馈 | rad/s | IMU `Gyro[Z]`（**原生就是 rad/s，无需换算**） |
| `pitch` 角反馈 | rad | DM4310 `measure.angle_total` |
| `pitch` 速度反馈 | rad/s | DM4310 `measure.velocity` |

之所以选择全弧度制而不是全度制：`Gyro[Z]` 原生是 rad/s，用弧度可以省掉每周期一次乘法；且与 pitch 单位一致。若改成度制，速度环的 `Kp` 会差 57.3 倍。

注意 `offset_angle` 仍然是**度**（chassis 用 `arm_cos_f32(offset_angle * DEGREE_2_RAD)` 做解算），这一处不要跟着改成弧度。

## IMU 安装位置与 offset_angle（最容易搞错的地方）

IMU 装在 **yaw 轴旁边，随云台一起转动**（步兵标准装法：yaw 读数干净、不受 pitch 影响，而 pitch 本来就走电机编码器）。由此得到两个**坐标系完全不同**的量，绝不能混用：

| 量 | 坐标系 | 物理含义 |
|---|---|---|
| IMU `YawTotalAngle` | **世界系绝对** | 云台指向哪个绝对方位。底盘转动时它**不变** |
| yaw 电机 `angle_total` | **云台相对底盘** | 云台相对底盘转了多少 |

- **yaw 目标 / 反馈** 用 IMU 绝对角 → 云台锁世界坐标系 ✓
- **`offset_angle`（跟随环 + 平移坐标系映射）** 必须用 **yaw 电机角** ✓

用错会怎样：拿 IMU 绝对角去算 `offset_angle`，底盘转动时该值不变 →
跟随环误差恒为常数、**增益为 0，底盘会一直转不收敛**；平移的前进方向也不会跟着云台走。

### 上电必须把云台摆正

`DMMotorInit` 每次上电都会执行 `DMMotorCaliEncoder`，**把上电瞬间的姿态设为电机零点**。所以 yaw 电机角是**相对上电姿态**的：

> **上电时请把云台摆到与底盘正前方一致**，此时电机角为 0，`YAW_CHASSIS_ALIGN_ANGLE` 保持 `0.0f` 即可。

若上电时云台是歪的，跟随模式和高速平移都会带上一个固定的角度偏差。

## 代码结构

### 外部接口

```c
void GimbalInit(); // 被 RobotInit() 调用
void GimbalTask(); // 放入 RobotTask(), 200Hz
```

### 订阅 / 发布

- **订阅** `gimbal_cmd`（`Gimbal_Ctrl_Cmd_s`）：来自 robot_cmd 的目标角度与前馈
- **发布** `gimbal_feed`（`Gimbal_Upload_Data_s`）：`gimbal_imu_data`（整份 IMU 姿态，robot_cmd 用它算 `offset_angle`）+ `pitch_motor_angle`

### 私有函数

- `GimbalLimitPitch(ref)` — pitch 软件限位，0 表示该方向不限制
- `GimbalLimitYaw(ref)` — yaw 软件限位。yaw 反馈是 IMU 绝对角，本身多圈无界，而机械限位是相对底盘的，所以**先把目标转到"相对对齐零位"的角度再限位，限完转回绝对角**。`YAW_MAX_ANGLE` 与 `YAW_MIN_ANGLE` 同时为 0 时（默认）视为可无限旋转（有滑环），直接放行

## 关键实现细节

### OTHER_FEED 指针无判空保护

`yaw_angle_rad` / `yaw_speed_rad_s` 这两个静态变量每周期从 IMU 刷新，DM 电机以 `OTHER_FEED` 解引用它们的地址。

`DMMotor` 对 `other_angle_feedback_ptr` / `other_speed_feedback_ptr` **没有判空保护**（见 `dmmotor.c` 的角度环/速度环），走 `OTHER_FEED` 就必须把两个指针配全，否则直接解引用野指针 HardFault。这两个变量具有静态存储期，天然满足"必须在 `DMMotorInit` 之前有效"的条件——**绝不能**改成局部变量或运行时才分配的指针。

另外 `DMMotor` 没有 `DMMotorChangeFeed()`（只有 DJI 侧有 `DJIMotorChangeFeed`），运行期要改反馈源只能直接写 `motor->motor_settings.angle_feedback_source` 和指针。

### DM 离线保护（应用层兜底）

`DMMotor` 模块自身**没有任何掉线处理**：`DMMotorLostCallback` 是空函数，`lost_cnt` 声明了但从未被读写。掉线后 `DMMotorTask` 仍会每 2ms 把最后一次的力矩指令重发出去，云台不会自己停。

因此 `GimbalTask()` 里用 daemon 兜底：任一轴离线就 `DMMotorStop` 两轴并直接返回。DM 收到一帧 CAN 就喂狗，`reload_count = 10` 在 100Hz 的 daemon 任务下约等于 **100ms 无反馈判定离线**（比 DJI 的 20ms 松 5 倍）。

### 模式切换与急停恢复

`gimbal` 侧在模式变化时会把参考值置成当前真实姿态兜底，但真正的对齐发生在 **robot_cmd** 侧：急停期间不读取摇杆/鼠标，恢复时由 `GimbalRefResync()` 把 `yaw`/`pitch` 目标重新对齐到当前姿态，否则停机期间积累的旧目标会让云台瞬间猛冲。

## 使用说明

修改 `robot_def.h` 中的宏即可适配新车，本文件通常不需要改动：

```c
#define CAN_YAW        hcan1     // yaw 所在总线
#define ID_YAW_TX      0x01      // 达妙控制帧 id, [实车核对]
#define ID_YAW_RX      0x03      // 达妙反馈帧 id, [实车核对]
#define CAN_PITCH      hcan2
#define ID_PITCH_TX    0x04
#define ID_PITCH_RX    0x02
#define YAW_CHASSIS_ALIGN_ANGLE 0.0f  // 云台指向底盘正前方时的 **yaw电机角**(度), 不是IMU角!
#define YAW_MAX_ANGLE  0.0f      // rad, 云台相对底盘的夹角限位(在电机角空间判断), 0=不限位
#define YAW_MIN_ANGLE  0.0f
#define PITCH_MAX_ANGLE 0.0f     // rad, 相对电机零位(上电姿态), 0=不限位
#define PITCH_MIN_ANGLE 0.0f
```

### 上电调试顺序

1. 先确认 DM 电机的 tx/rx id 与达妙配置软件里烧进去的值一致（不走官方 0x1XX/0x2XX 规律，是自定义的）
2. 确认转向：`motor_reverse_flag` 全部先设 `MOTOR_DIRECTION_NORMAL`，实车看转向再改。注意 DM 的 `motor_reverse_flag` **只对指令取反，不反转反馈量**，装反了需要在源变量上取反
3. 标定 `YAW_CHASSIS_ALIGN_ANGLE`：把云台摆到与底盘正前方一致，读此时 IMU 的 `YawTotalAngle`
4. 标定 pitch 限位：手动推到底，读 `angle_total`（上电姿态即零位，注意 `DMMotorInit` 每次上电都会执行 `DMMotorCaliEncoder` 把上电瞬间姿态设为零点）
5. 整定 PID。yaw 的初值沿用哨兵大 yaw 的值（angle `Kp=30`/`MaxOut=3.0`，speed `Kp=2.0`/`MaxOut=10`），pitch 同。DM 速度环输出是**力矩 N·m**（`LIMIT_MIN_MAX(set, DM_T_MIN, DM_T_MAX)`，直接力矩模式，Kp/Kd 下发为 0），与 DJI 的电流量纲完全不同，增益不可照搬
