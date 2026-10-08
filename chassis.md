# chassis

四全向轮（45°斜装）底盘应用。

> 从哨兵迁移而来，**运动学解算一行未动**——哨兵本来就已经是 45° 斜装全向轮形式。
> 本次只改了：CAN ID 改走 `robot_def.h` 宏、移除裁判系统/超级电容/UI、功率上限改为固定值。

@Todo 使用条件编译，选择全向轮/舵轮/平衡底盘

## 工作流程

首先进行初始化，`ChassisInit()` 会被 `RobotInit()` 调用，进行底盘 4 个 M3508 电机的初始化。如果为双板模式，则还会初始化 IMU，并且将消息订阅者和发布者的初始化改为 `CANComm` 的初始化。

操作系统启动后，工作顺序为：

1. 从 cmd 模块获取数据（如果双板则从 CANComm 获取）
2. 设定功率上限，判断当前控制数据的模式，如果为停止则停止所有电机
3. 根据控制模式计算底盘的旋转速度（跟随 / 不跟随 / 小陀螺）
4. 根据控制数据中的 `offset_angle`（云台指向与底盘正方向的夹角），将控制量映射到底盘坐标系下
5. 进行全向轮的运动学解算，得到每个电机的设定值
6. 通过功率控制对输出进行限幅并设定电机参考值
7. 由电机的反馈速度计算底盘当前的真实运动速度（逆解算）
8. 将反馈数据推送到消息中心（如果双板则通过 CANComm 发送）

## 运动学解算

采用**四全向轮 45° 斜装**：轮子滚动方向与车体轴线成 45°，因此平移项带 `sin45`/`cos45` 投影，旋转项直接乘旋转半径。

```c
vt_lf = -vx*sin45 - vy*cos45 - wz*LF_CENTER;
vt_rf = -vx*sin45 + vy*cos45 - wz*RF_CENTER;
vt_lb =  vx*sin45 - vy*cos45 - wz*LB_CENTER;
vt_rb =  vx*sin45 + vy*cos45 - wz*RB_CENTER;
```

旋转半径用 `CHASSIS_MOTOR_TO_CENTER`（**实测值**，含打滑修正，比理论值小约 13%），不是从轴距轮距推出来的理论值。

`EstimateSpeed()` 是上述公式的逆解算，用于里程计。**改某个轮子的方向时请改电机的 `motor_reverse_flag`，不要改解算公式**，否则逆解算会跟着失效。

## 功率限制（重要）

每 5ms 刷新一次功率上限，**优先用裁判系统实时值，拿不到时回退到固定宏**：

```c
if (referee_data != NULL && referee_data->GameRobotState.chassis_power_limit > 0)
    SetPowerLimit(referee_data->GameRobotState.chassis_power_limit); // 随比赛等级/增益变化
else
    SetPowerLimit(CHASSIS_MAX_POWER); // robot_def.h 中的固定宏, 默认 45.0f W
```

⚠️ **兜底不能省**：裁判系统未连接或尚未收到数据时 `chassis_power_limit` 是 **0**，
直接喂给 `SetPowerLimit()` 会让底盘被完全限功率、原地躺平。这是原实现的隐患。

⚠️ **`CHASSIS_MAX_POWER` 也要按实际功率等级填对**——它是裁判系统离线期间的唯一上限，填 0 一样会躺平。

回退时只会打一次 `LOGWARNING`（边沿触发），避免每 5ms 刷屏。

底盘 4 个 M3508 必须用 `PowerControlInit()` 而不是 `DJIMotorInit()`——只有前者会走 `PowerControl()` 的功率分配（按 M3508 功率模型缩放电流），后者直接吃 `speed_PID.MaxOut`。

## 裁判系统

`ChassisInit()` 里调用 `UITaskInit(&huart6, &ui_data)` 初始化裁判串口并持续解析数据
（血量/热量/功率上限/发弹量/机器人ID）。

**图传 UI 当前未启用**：`uiTask` 没有在 `robot_task.h` 中创建，所以 `UITask()`/`MyUIInit()`
不会运行，裁判串口不会被 UI 刷新占用。想开 UI 就把那个任务加回去，
但注意 `MyUIInit()` 里有 `while (robot_id == 0) osDelay(100);`——不接裁判系统时该任务会一直阻塞在那里。

底盘 4 个 M3508 必须用 `PowerControlInit()` 而不是 `DJIMotorInit()`——只有前者会走 `PowerControl()` 的功率分配（按 M3508 功率模型缩放电流），后者直接吃 `speed_PID.MaxOut`。

## 开关映射

详见 `application/CAN接线对照.md` 第七节。右开关三档：下=零力模式，中=不跟随（底盘不转，全向平移），上=小陀螺。

> 「跟随云台」(`CHASSIS_FOLLOW_GIMBAL_YAW`) 已被「零力模式」替换，不再由任何开关触发。枚举保留在 `robot_def.h`，`ChassisTask()` 里对应的 case 也保留（死代码），只是 `robot_cmd.c` 不再下发该模式。

`CHASSIS_FOLLOW_GIMBAL_YAW` 用的是非线性跟随律 `wz = -1.5 * offset * |offset|`（角度平方项），不是 PID。注意 `|offset|` 必须用 `fabsf`，用 `abs` 会把 float 截断成整数。

### 零力模式

右开关下档进入零力：底盘 `DJIMotorStop`（电流归零）+ 云台 `DMMotorStop`，整机泄力可自由转动。切回中/上档时 `robot_cmd.c` 会调用 `GimbalRefResync()` 重新对齐云台目标，避免用零力期间的旧目标闭环猛冲。

## 后续支持平衡底盘

新增一个 app balance_chassis
