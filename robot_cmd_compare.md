# robot_cmd.c 学长对等实现整理（供对照）

> ⚠️ **本文档是历史资料，已与当前代码不一致。**
> 2026-09-27 迁移到「四全向轮单yaw单pitch步兵」时，`AutoModeSet()`、上位机通信、
> 裁判系统交互、巡逻/自瞄相关逻辑已全部删除，下表中提到的这些函数在当前
> `robot_cmd.c` 中**已不存在**。保留本文仅供追溯哨兵时期的实现思路，
> 对照当前实现请看 `robot_cmd.md`。

> 来源：`ltg-2026shaobing-continue-update/application/cmd/robot_cmd.c`（约 1270 行）
> 对照对象：本工作区 `application/cmd/robot_cmd.c`（约 460 行）
> 整理原则：只摘录**与我们新版已实现功能对等**的学长代码；纯测试/重复/写死的部分标注说明，不展开。

---

## 一、功能对照总览

| 新版函数 | 学长对应实现 | 差异要点 |
|---|---|---|
| `RobotCMDInit` | `RobotCMDInit` | 学长初始化了低通滤波、`shoot_mode=SHOOT_ON`；我们未初始化滤波 |
| `AutoModeSet` | `AUTOKeySet` + `TestKeySet`/`ApplyStartupPatrolControl` | 学长拆成 3 个并行函数（比赛/测试/写死），靠全局 flag 选一；我们合成 1 个 |
| `RemoteControlSet` | `RemoteControlSet` | 学长多了单发限位 `shoot_flag`、拨盘 `mode_flag` 切模式、`dadan` 反馈联动 |
| `MouseKeySet` | `MouseKeySet` | **几乎完全一致**，我们原样保留 |
| `EmergencyHandler` | `EmergencyHandler` | 学长**整体注释掉**；我们启用 |
| `CalcOffsetAngle` | `CalcOffsetAngle` | 学长退化为单行赋值（未处理环绕）；我们保留完整分段换算 |
| `RobotCMDTask` | `RobotCMDTask` | 学长靠 `rc_offline_flag` 判定自动；我们靠遥控开关仲裁 |
| （无直接对等） | `gimbal_behavior_to_motor` | 学长在 cmd 层做三电机模式映射；我们封装进 gimbal 应用 |
| （无直接对等） | 裁判系统交互段 | 学长大量 `RefereeSendSentry*`；我们暂未实现（字段已备好） |
| （无直接对等） | `RobotCMDStartupPatrolTask` | 学长的上电底盘巡逻；我们未实现 |

---

## 二、初始化 —— `RobotCMDInit`

```c
void RobotCMDInit()
{
    rc_data = RemoteControlInit(&huart3);

    // bubing_vision_recv_data = BubingVisionInit(&huart6); // 步兵,已注释
    // daoohang_vision_recv_data = DaohangVisionInit(&huart6);
    usb_recv_data = USBVisionInit(&huart1);   // 自瞄(USART1)
    micro_usb_data = MICROUSBVisionInit();    // 导航(USB VCP)
    LowPassFilter_Init_ByFreq(&auto_pitch_filter, AUTO_VISION_FILTER_SAMPLE_FREQ, AUTO_VISION_PITCH_FILTER_CUTOFF_FREQ);
    LowPassFilter_Init_ByFreq(&auto_yaw_filter, AUTO_VISION_FILTER_SAMPLE_FREQ, AUTO_VISION_YAW_FILTER_CUTOFF_FREQ);

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
#ifdef ONE_BOARD
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANComm_Init_Config_s comm_conf = {
        .can_config = { .can_handle = &hcan1, .tx_id = 0x312, .rx_id = 0x311, },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    };
    cmd_can_comm = CANCommInit(&comm_conf);
#endif
    gimbal_cmd_send.pitch = 0;
    shoot_cmd_send.shoot_mode = SHOOT_ON;      // 学长启动即开射击
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    shoot_cmd_send.load_mode = LOAD_STOP;
    robot_state = ROBOT_READY;
}
```

**差异**：
- 学长额外初始化了**视觉低通滤波**（`auto_pitch_filter`/`auto_yaw_filter`）；新版未做（可在 gimbal 补）。
- 学长默认 `shoot_mode = SHOOT_ON`；新版 `AutoModeSet` 里每次显式 `SHOOT_ON`。

---

## 三、云台电机模式映射 —— `gimbal_behavior_to_motor`（学长特有）

> 新版将此逻辑封装进 gimbal 应用（`GimbalTask` 内按 5 种行为模式直接控三电机），cmd 层不暴露 `*_motor_mode`。

```c
static void gimbal_behavior_to_motor()
{
    if (gimbal_cmd_send.gimbal_mode == GIMBAL_ZERO_FORCE) // 无力
    {
        gimbal_cmd_send.yaw_motor_mode = GIMBAL_MOTOR_RAW;
        gimbal_cmd_send.big_yaw_motor_mode = GIMBAL_MOTOR_RAW;
        gimbal_cmd_send.pitch_motor_mode = GIMBAL_MOTOR_RAW;
    }
    else if (gimbal_cmd_send.gimbal_mode == GIMBAL_ABSOLUTE_ANGLE) // 底盘跟随云台
    {
        gimbal_cmd_send.yaw_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_cmd_send.big_yaw_motor_mode = GIMBAL_MOTOR_GYRO;
        gimbal_cmd_send.pitch_motor_mode = GIMBAL_MOTOR_GYRO;
    }
    else if (gimbal_cmd_send.gimbal_mode == GIMBAL_RELATIVE_ANGLE) // 小陀螺
    {
        gimbal_cmd_send.yaw_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_cmd_send.big_yaw_motor_mode = GIMBAL_MOTOR_ROTATE;
        gimbal_cmd_send.pitch_motor_mode = GIMBAL_MOTOR_GYRO;
    }
    else if (gimbal_cmd_send.gimbal_mode == GIMBAL_MOTIONLESS) // 调试
    {
        gimbal_cmd_send.yaw_motor_mode = GIMBAL_MOTOR_TOQUE;
        gimbal_cmd_send.big_yaw_motor_mode = GIMBAL_MOTOR_ROTATE;
        gimbal_cmd_send.pitch_motor_mode = GIMBAL_MOTOR_GYRO;
    }
    else if (gimbal_cmd_send.gimbal_mode == GIMBAL_AUTO) // 自瞄
    {
        gimbal_cmd_send.yaw_motor_mode = GIMBAL_MOTOR_TOQUE;
        gimbal_cmd_send.big_yaw_motor_mode = GIMBAL_MOTOR_AUTO;
        gimbal_cmd_send.pitch_motor_mode = GIMBAL_MOTOR_AUTO;
    }
    else if (gimbal_cmd_send.gimbal_mode == GIMBAL_AUTO_XUNLUO)
    {
        gimbal_cmd_send.yaw_motor_mode = GIMBAL_MOTOR_AUTO_XUNLUO;
        gimbal_cmd_send.big_yaw_motor_mode = GIMBAL_MOTOR_AUTO_XUNLUO;
        gimbal_cmd_send.pitch_motor_mode = GIMBAL_MOTOR_AUTO_XUNLUO;
    }
}
```

---

## 四、自动模式 —— 学长对等实现

### 4.1 正常比赛自动模式 `AUTOKeySet`（← 新版 `AutoModeSet` 主对等）

```c
static void AUTOKeySet()
{
    static int8_t i = 0;
    // 各模式"上次值"继承(模式切换边沿用)
    chassis_cmd_send.last_chassis_mode = chassis_cmd_send.chassis_mode;
    gimbal_cmd_send.last_big_yaw_motor_mode = gimbal_cmd_send.big_yaw_motor_mode;
    gimbal_cmd_send.last_pitch_motor_mode = gimbal_cmd_send.pitch_motor_mode;
    gimbal_cmd_send.last_yaw_motor_mode = gimbal_cmd_send.yaw_motor_mode;
    shoot_cmd_send.last_lode_mode = shoot_cmd_send.load_mode;

    // ---- 云台: 始终自瞄 ----
    gimbal_cmd_send.gimbal_mode = GIMBAL_AUTO;
    gimbal_cmd_send.yaw   = usb_recv_data->yaw * TONGJI_YAW_AUTO_SEN;
    gimbal_cmd_send.pitch = -usb_recv_data->pitch * TONGJI_YAW_AUTO_SEN; // 注意:学长笔误用了YAW的增益,新版用TONGJI_PITCH_AUTO_SEN
    gimbal_behavior_to_motor();

    // ---- 底盘: 直接用导航速度(注意vx取负,vy取原值) ----
    chassis_cmd_send.vx = -micro_usb_data->linery;
    chassis_cmd_send.vy =  micro_usb_data->linearx;
    chassis_cmd_send.chassis_mode = CHASSIS_OPEN; // "正式比赛使用差速小陀螺"

    // ---- 发射: 恒开摩擦轮+连发 ----
    shoot_cmd_send.shoot_mode = SHOOT_ON;
    shoot_cmd_send.shoot_rate = 16.0f;
    shoot_cmd_send.bullet_speed = SMALL_AMU_25;
    shoot_cmd_send.friction_mode = FRICTION_ON;
    if (usb_recv_data->mode == 2)
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
    else
        shoot_cmd_send.load_mode = LOAD_STOP;
}
```

**差异**：
- 学长恒开摩擦轮、射频 16、无热量保护；新版 `mode==2 && 热量≤220` 才开火，更安全。
- 学长底盘 vx/vy **交换+取负**；新版直接用 `linearx/linery/angularz`。
- 学长**无巡逻**（巡逻在 `TestKeySet` 的 `auto_mode_flag>=500` 里）；新版 `mode==0` 直接巡逻。

### 4.2 巡逻触发（学长在 `TestKeySet` 中）

```c
// 上位机连续 500 帧不请求开火(mode!=2) → 巡逻
if (auto_mode_flag >= 500)
    gimbal_cmd_send.gimbal_mode = GIMBAL_AUTO_XUNLUO;
else
    gimbal_cmd_send.gimbal_mode = GIMBAL_AUTO;

// 比赛剩余时间 >= 390s 强制回自瞄(开局不巡逻)
if (chassis_fetch_data.game_state.stage_remain_time >= 390)
    gimbal_cmd_send.gimbal_mode = GIMBAL_AUTO;
// 剩余时间 >= 412s(比赛未开始) 云台归零
if (chassis_fetch_data.game_state.stage_remain_time >= 412)
{
    gimbal_cmd_send.yaw = 0;
    gimbal_cmd_send.pitch = 0;
}
```

**差异**：学长巡逻靠**帧数累计 + 时间硬编码**；新版靠 `mode==0` 直接判断。

### 4.3 自动发射热量保护（学长在 `TestKeySet`/`ApplyStartupPatrolControl` 中）

```c
shoot_cmd_send.shoot_mode = SHOOT_ON;
shoot_cmd_send.friction_mode = FRICTION_ON;
shoot_cmd_send.shoot_rate = 16.0f;
shoot_cmd_send.bullet_speed = SMALL_AMU_25;

if (chassis_fetch_data.game_state.game_progress != 4)   // 非战斗阶段
    shoot_cmd_send.load_mode = LOAD_STOP;
else if (usb_recv_data->mode == 2 && chassis_fetch_data.rest_heat <= 220)
    shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
else
    shoot_cmd_send.load_mode = LOAD_STOP;
```

**差异**：学长多了 `game_progress != 4`（非战斗期禁射）判断；新版用热量 220 阈值一致。

### 4.4 单发限位（学长特有，新版未实现）

```c
// 单发模式下,打一发后 shoot_flag 置0,反馈弹完再放开
if (shoot_fetch_data.feedback_shoot_flag == 2 && shoot_cmd_send.load_mode == LOAD_1_BULLET)
{
    shoot2_flag++;
    if (shoot2_flag >= 40) { shoot_cmd_send.shoot_flag = 0; shoot2_flag = 0; }
}
if (shoot_cmd_send.load_mode == LOAD_1_BULLET && shoot_cmd_send.shoot_flag == 0 && usb_recv_data->mode == 2)
    shoot_cmd_send.shoot_flag = 1;
if (shoot_cmd_send.load_mode == LOAD_1_BULLET && shoot_cmd_send.last_lode_mode != LOAD_1_BULLET && usb_recv_data->mode == 2)
    shoot_cmd_send.shoot_flag = 0; // 防模式切换后卡死
```

---

## 五、遥控器模式 —— 学长 `RemoteControlSet`（← 新版 `RemoteControlSet`）

```c
static void RemoteControlSet()
{
    static int16_t yaw_channel = 0, pitch_channel = 0, big_yaw_channel = 0;
    static int16_t vx_channel = 0, vy_channel = 0;
    static float vx_set_channel = 0, vy_set_channel = 0;
    static int16_t dadan = 1;
    static int16_t flag = 0, shoot2_flag = 0;

    // 模式值继承
    chassis_cmd_send.last_chassis_mode = chassis_cmd_send.chassis_mode;
    gimbal_cmd_send.last_big_yaw_motor_mode = gimbal_cmd_send.big_yaw_motor_mode;
    gimbal_cmd_send.last_pitch_motor_mode = gimbal_cmd_send.pitch_motor_mode;
    gimbal_cmd_send.last_yaw_motor_mode = gimbal_cmd_send.yaw_motor_mode;
    shoot_cmd_send.last_lode_mode = shoot_cmd_send.load_mode;
    shoot_cmd_send.shoot_flag = shoot_fetch_data.feedback_shoot_flag;
    dadan = shoot_fetch_data.dadan;

    // ---- 模式选择(右侧开关) ----
    if (switch_is_down(rc_data[TEMP].rc.switch_right))   // [下] 无力
    { chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE; gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE; }
    else if (switch_is_mid(rc_data[TEMP].rc.switch_right)) // [中] 底盘跟随云台
    { chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW; gimbal_cmd_send.gimbal_mode = GIMBAL_ABSOLUTE_ANGLE; }
    else if (switch_is_up(rc_data[TEMP].rc.switch_right)) // [上] 小陀螺
    { chassis_cmd_send.chassis_mode = CHASSIS_OPEN; gimbal_cmd_send.gimbal_mode = GIMBAL_MOTIONLESS; }
    gimbal_behavior_to_motor();

    // ---- 云台: 无力时清零 ----
    if (gimbal_cmd_send.gimbal_mode == GIMBAL_ZERO_FORCE)
    { gimbal_cmd_send.yaw = 0; gimbal_cmd_send.pitch = 0; gimbal_cmd_send.big_yaw = 0; }

    // ---- 云台: 底盘跟随云台(左摇杆控大yaw+pitch) ----
    if (gimbal_cmd_send.gimbal_mode == GIMBAL_ABSOLUTE_ANGLE)
    {
        rc_deadband_limit(rc_data[TEMP].rc.rocker_l_,  yaw_channel,   GIMBAL_RC_DEADBAND);
        rc_deadband_limit(rc_data[TEMP].rc.rocker_l1, pitch_channel, GIMBAL_RC_DEADBAND);
        gimbal_cmd_send.yaw = 0;
        gimbal_cmd_send.pitch = pitch_channel * PITCH_RC_SEN;
        gimbal_cmd_send.big_yaw = yaw_channel * BIG_YAW_RC_SEN;
    }
    // ---- 云台: 小陀螺/调试(左摇杆控小yaw+pitch) ----
    if (gimbal_cmd_send.gimbal_mode == GIMBAL_RELATIVE_ANGLE ||
        gimbal_cmd_send.gimbal_mode == GIMBAL_MOTIONLESS)
    {
        rc_deadband_limit(rc_data[TEMP].rc.rocker_l_,  yaw_channel,   GIMBAL_RC_DEADBAND);
        rc_deadband_limit(rc_data[TEMP].rc.rocker_l1, pitch_channel, GIMBAL_RC_DEADBAND);
        gimbal_cmd_send.big_yaw = 0;
        gimbal_cmd_send.pitch = pitch_channel * PITCH_RC_SEN;
        gimbal_cmd_send.yaw = yaw_channel * YAW_RC_SEN;
    }

    // ---- 底盘(右摇杆→vx/vy) ----
    rc_deadband_limit(rc_data[TEMP].rc.rocker_r_,  vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(rc_data[TEMP].rc.rocker_r1, vy_channel, CHASSIS_RC_DEADLINE);
    vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
    vy_set_channel = vy_channel * CHASSIS_VY_RC_SEN;
    chassis_cmd_send.vx = vx_set_channel;
    chassis_cmd_send.vy = vy_set_channel;
    // CHASSIS_NO_FOLLOW_YAW 时强制 no_follow_yaw_angle = 0

    // ---- 发射: 热量保护 ----
    if (chassis_fetch_data.rest_heat >= 220) shoot_cmd_send.shoot_mode = SHOOT_OFF;
    else shoot_cmd_send.shoot_mode = SHOOT_ON;

    // ---- 摩擦轮: 左侧开关边沿翻转 ----
    if ((switch_is_up(rc_data[TEMP].rc.switch_left)) && (!switch_is_up(rc_data[LAST].rc.switch_left)) &&
        (shoot_cmd_send.friction_mode == FRICTION_OFF))
        shoot_cmd_send.friction_mode = FRICTION_ON;
    else if (switch_is_up(rc_data[TEMP].rc.switch_left) && !switch_is_up(rc_data[LAST].rc.switch_left) &&
             shoot_cmd_send.friction_mode != FRICTION_OFF)
        shoot_cmd_send.friction_mode = FRICTION_OFF;

    // ---- 拨盘切发射模式(mode_flag 计数,带拨盘防抖) ----
    if (rc_data[TEMP].rc.dial < -500)
    {
        if (rc_offline_flag == 0 && last_rc_offline_flag == 1) mode_flag = 0;
        if ((mode_flag_time + dead_line_time >= DWT_GetTimeline_ms()) || line_flag == 0) line_flag++;
        else { mode_flag++; dead_line_time = 20; mode_flag_time = DWT_GetTimeline_ms(); }
    }
    if (shoot_fetch_data.mode_flag == 1) mode_flag = 0;
    if (mode_flag == 2) { mode_flag = 0; shoot_cmd_send.load_mode = LOAD_STOP; }
    shoot_cmd_send.load_mode = mode_flag;

    // ---- 单发限位(遥控版) ----
    if (shoot_cmd_send.shoot_flag == 2 && rc_offline_flag == 0)
    {
        shoot2_flag++;
        if (shoot2_flag >= 100) { shoot_cmd_send.shoot_flag = 1; shoot2_flag = 0; }
    }
    if (shoot_cmd_send.load_mode == LOAD_1_BULLET && shoot_cmd_send.shoot_flag == 0 && dadan == 0)
        shoot_cmd_send.shoot_flag = 1;
    if (shoot_cmd_send.load_mode == LOAD_1_BULLET && shoot_cmd_send.last_lode_mode != LOAD_1_BULLET && dadan == 0)
        shoot_cmd_send.shoot_flag = 1;

    if (switch_is_down(rc_data[TEMP].rc.switch_left))
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;

    shoot_cmd_send.shoot_rate = 13;
    shoot_cmd_send.bullet_speed = SMALL_AMU_25;
}
```

**差异**：
- 学长右侧开关 3 档：无力/跟随/小陀螺；新版 2 档：`ROTATE+GYRO`/`NO_FOLLOW+FREE`。
- 学长有拨盘 `mode_flag` 切发射模式、单发 `shoot_flag` 限位、`dadan` 反馈；新版简化用拨轮直接定模式。
- 学长摩擦轮用**左侧开关边沿**切换；新版用**拨轮**。

---

## 六、键鼠模式 —— 学长 `MouseKeySet`（与我们一致，仅摘录）

```c
static void MouseKeySet()
{
    chassis_cmd_send.vx = rc_data[TEMP].key[KEY_PRESS].w * 300 - rc_data[TEMP].key[KEY_PRESS].s * 300;
    chassis_cmd_send.vy = rc_data[TEMP].key[KEY_PRESS].s * 300 - rc_data[TEMP].key[KEY_PRESS].d * 300;
    gimbal_cmd_send.yaw   += (float)rc_data[TEMP].mouse.x / 660 * 10;
    gimbal_cmd_send.pitch += (float)rc_data[TEMP].mouse.y / 660 * 10;
    // Z键弹速: 15/18/30
    // E键发射模式: STOP/1发/连发
    // R键弹舱开合, F键摩擦轮开关
    // C键底盘速度档: 40/60/80/100
    // Shift: 预留超功率(空)
}
```

> 与新版 **完全一致**，无需修改。

---

## 七、急停 —— 学长 `EmergencyHandler`（整体注释）

```c
static void EmergencyHandler()
{
    // 拨轮向下拨超过一半进入急停
    // if (rc_data[TEMP].rc.dial < -500 || robot_state == ROBOT_STOP)
    // {
    //     robot_state = ROBOT_STOP;
    //     gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
    //     chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    //     shoot_cmd_send.shoot_mode = SHOOT_OFF;
    //     shoot_cmd_send.friction_mode = FRICTION_OFF;
    //     shoot_cmd_send.load_mode = LOAD_STOP;
    //     LOGERROR("[CMD] emergency stop!");
    // }
    // if (switch_is_up(rc_data[TEMP].rc.switch_right))
    // {
    //     robot_state = ROBOT_READY;
    //     shoot_cmd_send.shoot_mode = SHOOT_ON;
    //     LOGINFO("[CMD] reinstate, robot ready");
    // } // 暂时不需要
}
```

**差异**：学长完全禁用；新版启用（拨轮下拨>300 → 全停，右侧开关[上] → 恢复）。

---

## 八、偏角计算 —— 学长实际把角度计算写在 DM 驱动解码回调里

> ⚠️ 更正：学长并非"退化"，而是把角度计算**下沉到了 DM 电机驱动解码回调**（`modules/motor/DMmotor/dmmotor.c` 的 `DMMotorDecode`），`CalcOffsetAngle` 只是直接取驱动算好的现成值。

### 8.1 学长 `CalcOffsetAngle`（cmd 层，仅单行取值）

```c
static void CalcOffsetAngle()
{
    static float angle;
    static float total_angle;
    chassis_cmd_send.offset_angle = gimbal_fetch_data.yaw_motor_single_round_angle;
    // 这里我直接在回调函数里写完了  // 双yaw下这个函数要改
}
```

### 8.2 学长真正的角度计算（DM4310 解码回调 `DMMotorDecode`）

```c
static int flag = 0;   // ⚠️ static，所有 DM 电机实例共享！

// ---- 折叠归一化: DM4310 position 范围 ±12.5rad(约±2圈), 折叠到单圈[0,2π] ----
if (measure->position >= 6.283185 && measure->position <= 12.56637)
    measure->position -= 6.283185;   // [2π,4π] → [0,2π]
if (measure->position <= 0 && measure->position > -6.283185)
    measure->position += 6.283185;   // (-2π,0] → (0,2π]
if (measure->position <= -6.283185 && measure->position > -12.56637)
    measure->position += 12.56637;   // (-4π,-2π] → (0,2π]

// ---- 转成 8192 编码(模拟DJI 6020), ecd_sum 前100帧标定为零点 ----
motor->last_ecd = motor->ecd;            // ⚠️ 赋值后从未使用(死代码)
motor->ecd = measure->position * 8192 / 6.283185;
if (flag <= 100)
{
    motor->ecd_sum = motor->ecd;         // 上电后100帧的位置当零点(非机械零位!)
    flag++;
}
measure->angle_single_round = -rad_format(MOTOR4310_ECD_TO_RAD * (motor->ecd - motor->ecd_sum));
// 相对零点角度(rad), 限制[-π,π], 取负统一左转为负
```

### 8.3 消费链路

```
DMMotorDecode(驱动) 算 angle_single_round(相对上电零点的±π角度)
   ↓
GimbalTask: gimbal_feedback_data.yaw_motor_single_round_angle = big_yaw_motor->measure.angle_single_round
   ↓
CalcOffsetAngle: offset_angle = yaw_motor_single_round_angle  ← 直接用大yaw相对角
```

### 8.4 学长这套实现的问题

| # | 问题 | 说明 |
|---|---|---|
| 1 | `static int flag` 共享 | `DMMotorDecode` 里 static，**大yaw和pitch两个DM电机共用**，先跑的电机把 flag 累加过100后，后跑的电机 ecd_sum 就停标定（保持0/垃圾） |
| 2 | `last_ecd` 死代码 | 赋值后从未使用，学长自己也标注"可能有问题，垃圾值" |
| 3 | `total_round` 从未赋值 | 结构体声明了圈数，但**没有真正的多圈累计**，折叠只在±2圈内有效 |
| 4 | 零点依赖上电位置 | `ecd_sum` 是"上电后100帧的位置"，不是机械零位；offset_angle 实际是**大yaw相对上电位置的角**，非底盘-云台真实夹角 |
| 5 | 无连续多圈角度 | 转出±2圈后 ecd 在0/8192间跳变，angle_single_round 在±π间跳变，巡逻/连续转圈会失效 |
| 6 | CalcOffsetAngle 语义错位 | 学长把**大yaw相对角**当成 offset_angle，而基础框架的 offset_angle 应是**小yaw(底盘跟随)相对底盘**的夹角 |

### 8.5 新版对比（`modules/motor/DMmotor/dmmotor.c`）

```c
// 多圈角度累计: position量程±12.5rad(约2圈), 增量超过半程视为跨圈跳变, 解包成连续角度
{
    static float delta;
    delta = measure->position - measure->last_position;
    if (delta > (DM_P_MAX - DM_P_MIN) * 0.5f)
        delta -= (DM_P_MAX - DM_P_MIN);
    else if (delta < -(DM_P_MAX - DM_P_MIN) * 0.5f)
        delta += (DM_P_MAX - DM_P_MIN);
    measure->angle_total += delta; // 真正的多圈累计角度(rad)
}
measure->angle_single_round = measure->position; // 单圈角度保留原始±12.5
measure->total_round = (int32_t)(measure->angle_total / (2.0f * PI)); // 真圈数
```

新版 `GimbalTask` 反馈：
```c
gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round; // 小yaw单圈角
gimbal_feedback_data.big_yaw_motor_angle = big_yaw_motor->measure.angle_total;             // 大yaw多圈角
```

**关键差异**：
- 新版用**增量法**（`last_position`→`position` 增量+跨圈判断）做**真多圈累计** `angle_total`，支持360°无限旋转；学长靠折叠只在±2圈内，无连续多圈。
- 新版 `total_round` 有真实值；学长声明了但从未计算。
- 新版无共享 static flag 问题（每实例自己记 `last_position`）。
- 新版 `angle_single_round` 保留原始 position（±12.5rad），学长则折成 [0,2π] 再转8192编码。
- 新版反馈：小yaw单圈 + 大yaw多圈分字段；学长只给 `big_yaw` 的相对角填进 `yaw_motor_single_round_angle`。

---

## 九、主任务 —— 学长 `RobotCMDTask`（← 新版 `RobotCMDTask`）

```c
void RobotCMDTask()
{
#ifdef ONE_BOARD
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
#endif // GIMBAL_BOARD
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);

    CalcOffsetAngle();

    // ---- 自动/手动仲裁: 靠遥控器离线判定 ----
    if (rc_offline_flag == 1)
    {
        if (fighting_enable_flag)        AUTOKeySet();             // 正常比赛
        if (test_enable_flag)            TestKeySet();             // 日常测试
        if (dain_kong_gu_ding)           ApplyStartupPatrolControl(); // 写死模式(=1)
    }
    if (rc_offline_flag == 0)
    {
        auto_vision_filter_inited = 0;
        RemoteControlSet();
    }
    if (rc_offline_flag == 0 && last_rc_offline_flag == 1)
    {
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
    }

    // 把数据指针下传给各应用
    gimbal_cmd_send.usb_recv_data = usb_recv_data;                    // 视觉数据→云台
    chassis_cmd_send.IMU_data = &gimbal_fetch_data.gimbal_imu_data;   // IMU→底盘

    // EmergencyHandler(); // 已注释

    // ---- 双路回传上位机 ----
    DaohangVisionSetAltitude(0, chassis_fetch_data.real_vy, chassis_fetch_data.real_vx,
                             chassis_fetch_data.sentry_hp,
                             chassis_fetch_data.game_state.stage_remain_time,
                             chassis_fetch_data.remaining_gold,
                             chassis_fetch_data.shoot_count);
    UsbVsioionSetAltiitude(gimbal_fetch_data.small_gimbal_data->oula_data.yaw,
                           -gimbal_fetch_data.gimbal_data->Pitch_Data.pitch_absoulte_angle,
                           gimbal_fetch_data.small_gimbal_data->quar_data.q,
                           gimbal_fetch_data.small_gimbal_data->gyro_data.z_gyro,
                           gimbal_fetch_data.small_gimbal_data->gyro_data.x_gyro,
                           shoot_cmd_send.bullet_speed);

#ifdef ONE_BOARD
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif
#ifdef GIMBAL_BOARD
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
#endif
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
}
```

**差异**：
- 学长仲裁靠 `rc_offline_flag`（遥控器离线才自动）；新版靠右侧开关[下]显式进 `AutoModeSet`。
- 学长把 `usb_recv_data` / `IMU_data` **指针**下传给云台/底盘应用；新版通过 pub-sub 结构体传值。
- 学长**双路**回传（导航含血量/金币/时间/发弹量 + 自瞄含四元数）；新版单路自瞄回传（四元数暂空）。

---

## 十、学长特有、新版未实现的功能（记录备查）

以下学长有、新版暂未做（按你"裁判系统先不急"的规划）：

### 10.1 裁判系统交互段（`TestKeySet`/`ApplyStartupPatrolControl` 中）

```c
// 哨兵姿态申请: 血>150→进攻姿态,否则防御姿态,250ms重试
referee_sentry_posture_e target_posture =
    (current_hp > 150.0f) ? REFEREE_SENTRY_POSTURE_ATTACK : REFEREE_SENTRY_POSTURE_DEFENSE;
if (current_sentry_posture != (uint8_t)target_posture) {
    if (current_time_ms - posture_request_last_send_ms >= 250.0f) {
        RefereeSendSentryPosture(target_posture);
        posture_request_last_send_ms = current_time_ms;
    }
}

// 免费复活: 战斗阶段且剩余时间<=240s 时持续确认
if (can_confirm_free_revive != 0u &&
    current_time_ms - revive_request_last_send_ms >= 250.0f) {
    RefereeSendSentryConfirmRevive(1);
    revive_request_last_send_ms = current_time_ms;
}

// 远程血量交换: 掉线(is_disengaged)且血<80且金币>550且剩余<240且次数<15
if (!remote_hp_request_active && is_disengaged != 0u && current_hp < 80.0f &&
    chassis_fetch_data.remaining_gold > 550u && stage_remain_time < 240u &&
    current_remote_hp_count < 0xFu) {
    remote_hp_request_active = 1u;
    remote_hp_request_count = (uint8_t)(current_remote_hp_count + 1u);
    remote_hp_last_send_ms = -6000.0f;
}
// ... 每6s重发 RefereeSendSentryRemoteHPExchangeCount(count)

// 远程发弹量交换: 掉线且发弹<30且金币>550且剩余<240且次数<15
if (!remote_projectile_request_active && is_disengaged != 0u &&
    current_projectile_allowance_17mm < 30u && ...) {
    remote_projectile_request_active = 1u;
    remote_projectile_request_count = (uint8_t)(current_remote_projectile_count + 1u);
    ...
}
// ... 每6s重发 RefereeSendSentryRemoteProjectileExchangeCount(count)
```

### 10.2 被击躲避（`TestKeySet`/`ApplyStartupPatrolControl` 中）

```c
// 血量下降或 hit_flag 未结束 → 小陀螺躲避 8000 帧
if (chassis_fetch_data.game_state.game_progress == 4)
{
    if ((current_hp < last_hp) || hit_flag != 0) {
        chassis_cmd_send.vx = -micro_usb_data->linery;
        chassis_cmd_send.vy =  micro_usb_data->linearx;
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        if (hit_flag == 0) hit_flag = 8000;
    }
    if ((current_hp == last_hp) && hit_flag == 0) {
        chassis_cmd_send.vx = -micro_usb_data->linery;
        chassis_cmd_send.vy =  micro_usb_data->linearx;
        chassis_cmd_send.chassis_mode = CHASSIS_OPEN;
    }
}
else {
    chassis_cmd_send.vx = 0; chassis_cmd_send.vy = 0;
    chassis_cmd_send.chassis_mode = CHASSIS_OPEN;
}
last_hp = current_hp;
```

### 10.3 上电底盘巡逻 `RobotCMDStartupPatrolTask`

```c
// game_progress>=5 激活; <=3 不跑
// 阶段1: 右移(SENTRY_STARTUP_RIGHT_SPEED) 持续 SENTRY_STARTUP_RIGHT_TIME_MS
// 阶段2: 前进(SENTRY_STARTUP_FORWARD_SPEED) 持续 SENTRY_STARTUP_FORWARD_TIME_MS 后停止
switch (sentry_startup_patrol_stage) {
case SENTRY_STARTUP_STAGE_RIGHT:
    target_vx = 0; target_vy = SENTRY_STARTUP_RIGHT_SPEED;
    if (now_ms - start_ms >= SENTRY_STARTUP_RIGHT_TIME_MS) {
        stage = SENTRY_STARTUP_STAGE_FORWARD; start_ms = now_ms;
        target_vx = SENTRY_STARTUP_FORWARD_SPEED; target_vy = 0;
    }
    break;
case SENTRY_STARTUP_STAGE_FORWARD:
    target_vx = SENTRY_STARTUP_FORWARD_SPEED; target_vy = 0;
    if (now_ms - start_ms >= SENTRY_STARTUP_FORWARD_TIME_MS) { /* 停止并结束 */ }
    break;
}
```

---

## 十一、学长代码遗留问题清单（新版已规避）

| # | 问题 | 位置 |
|---|---|---|
| 1 | 写死开关 `dain_kong_gu_ding = 1`、`sentry_startup_patrol_active = 1` | 文件顶部 |
| 2 | `TestKeySet` 与 `ApplyStartupPatrolControl` 几乎全文重复 | 两大函数 |
| 3 | 绝对路径 include `"tongjimachine/message.h"`（clangd 报错） | 顶部 |
| 4 | 未用变量/参数 `count`、`current_usb_mode`、`posture_cycle_test_inited`、`current_time_ms` 等 | 多处 |
| 5 | 命名混乱：`CHASSIS_OPEN`、`CHASSIS_FOLLOW_GIMBAL_YAW` 重复 case | RemoteControlSet |
| 6 | 时间硬编码 `stage_remain_time >= 390/412`、`hit_flag = 8000` | TestKeySet |
| 7 | 自瞄 pitch 增益笔误：用了 `TONGJI_YAW_AUTO_SEN` 而非 `TONGJI_PITCH_AUTO_SEN` | AUTOKeySet |
| 8 | `CalcOffsetAngle` 退化为单行赋值（双 yaw 下失效） | CalcOffsetAngle |
| 9 | `EmergencyHandler` 整体注释禁用 | EmergencyHandler |
| 10 | 自动判定依赖遥控器离线 `rc_offline_flag`（比赛风险） | RobotCMDTask |

---

*整理时间：2026-08-12*
