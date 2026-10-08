/**
 * @file robot_def.h
 * @author NeoZeng neozng1@hnu.edu.cn
 * @author Even
 * @version 0.1
 * @date 2022-12-02
 *
 * @brief 机器人参数配置文件
 *        兵种: 四全向轮(45°斜装) 单yaw轴 单pitch轴 步兵
 *
 *        本文件是换车/改接线时唯一需要大改的地方,硬件相关的宏全部集中在此,
 *        应用层的 .c 文件不再散落 CAN id 和机械尺寸。
 *
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 *
 */
#pragma once // 可以用#pragma once代替#ifndef ROBOT_DEF_H(header guard)
#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include "ins_task.h"
#include "stdint.h"

/* 弹速档位。
 * 原先定义在 modules/master_machine/master_process.h, 上位机(视觉)模块已从构建中移除,
 * 但发射应用仍然需要它来切换摩擦轮转速, 因此收拢到这里。 */
typedef enum
{
    BULLET_SPEED_NONE = 0,
    BIG_AMU_10 = 10,
    SMALL_AMU_15 = 15,
    BIG_AMU_16 = 16,
    SMALL_AMU_18 = 18,
    SMALL_AMU_30 = 30,
} Bullet_Speed_e;

/* 开发板类型定义,烧录时注意不要弄错对应功能;修改定义后需要重新编译,只能存在一个定义! */
#define ONE_BOARD // 单板控制整车
// #define CHASSIS_BOARD //底盘板
// #define GIMBAL_BOARD  //云台板

/* ==========================================================================================
 * 硬件配置区
 * 换车、改接线、改机械尺寸时主要修改这一段。带 [实车核对] 的项上电前必须逐项确认。
 * ========================================================================================== */

/* ---- CAN 总线分配 ---- */
/* 每个设备单独一个宏,便于把某个设备挪到另一条总线上而不影响其他设备 */
#define CAN_CHASSIS  hcan1 // 底盘 4× M3508
#define CAN_LOADER   hcan1 // 拨盘 M2006, 与底盘同总线
#define CAN_YAW      hcan1 // 云台 yaw DM4310
#define CAN_FRICTION hcan2 // 摩擦轮 2× M3508
#define CAN_PITCH    hcan2 // 云台 pitch DM4310

/* ---- 电机 ID ---- */
// 底盘四轮按**物理位置**命名, 这里填的是该位置电机实际烧录的 ID
#define ID_CHASSIS_LF 1 // 底盘左前
#define ID_CHASSIS_RF 4 // 底盘右前  [实车已确认]
#define ID_CHASSIS_LB 3 // 底盘左后
#define ID_CHASSIS_RB 2 // 底盘右后  [实车已确认]

#define ID_LOADER   5 // 拨盘 M2006, tx_id=5 使其落在 0x1FF 控制组,与底盘 1-4(0x200组)分开
#define ID_FRICTION_L 1
#define ID_FRICTION_R 2

/* 达妙 DM4310 的 tx/rx id(实车已确认)
 * 换算规则(MIT 模式): tx_id(控制帧) = 达妙软件的 canid; rx_id(反馈帧) = 达妙软件的 master_id
 * yaw  在 CAN1: canid=4, master_id=6
 * pitch 在 CAN2: canid=0x003, master_id=0x006
 * 注意两个电机的 master_id 都是 6(反馈帧ID都是0x006), 因分属不同总线故不冲突。 */
#define ID_YAW_TX   0x004 // yaw 控制帧 = canid 4
#define ID_YAW_RX   0x006 // yaw 反馈帧 = master_id 6
#define ID_PITCH_TX 0x003 // pitch 控制帧 = canid 0x003
#define ID_PITCH_RX 0x006 // pitch 反馈帧 = master_id 0x006

/* ---- 底盘机械参数 ---- */
#define WHEEL_BASE 400 // 纵向轴距(前进后退方向),单位 mm, [实车核对]
#define TRACK_WIDTH 400 // 横向轮距(左右平移方向),单位 mm, [实车核对]
// 底盘中心到轮速旋转等效作用点的距离,单位 m
//
// 该参数**只出现在运动学解算的旋转项**, 平移项(vx/vy)只用 sin45/cos45 系数, 完全不受它影响。
// 因此它填错不会影响全向平移, 只会让实际旋转速度与指令值差一个固定比例:
//     实际转速 = 指令转速 × (本参数 / 真实值)
// 它也**不会导致失稳**(小陀螺是开环给定; 跟随环的系数 -1.5 本来就要实测整定, 误差会被吸收)。
//
// 理论几何值 = sqrt((WHEEL_BASE/2)^2 + (TRACK_WIDTH/2)^2) / 1000
//            = sqrt(200^2 + 200^2) / 1000 = 0.283 m   (轴距轮距均为 400mm 时)
// 哨兵实测 0.2465, 约为理论值的 87%, 差值来自轮子打滑。
//
// 若新车底盘尺寸与哨兵相同, 直接沿用 0.2465 即可, 不需要重新精确测量。
// 若要校准, 用"数圈法"比拿尺子量更准(打滑量无法测量, 只能实测):
//     让底盘以已知 wz 自旋, 数 N 圈用时 T 秒 → 实际转速 = 360*N/T
//     新值 = 旧值 × (指令转速 / 实际转速)
#define CHASSIS_MOTOR_TO_CENTER 0.2465f
#define CENTER_GIMBAL_OFFSET_X 0 // 云台旋转中心距底盘几何中心的前后偏移,云台在正中心时设为0
#define CENTER_GIMBAL_OFFSET_Y 0 // 云台旋转中心距底盘几何中心的左右偏移,云台在正中心时设为0
#define RADIUS_WHEEL 60            // 轮子半径,单位 mm, [实车核对]
#define REDUCTION_RATIO_WHEEL 19.0f // 电机减速比,编码器量测的是转子速度而非输出轴速度,故需换算

/* ---- 功率限制 ---- */
// 裁判系统已移除,底盘功率上限改为固定值。
// [实车核对] 按实际功率等级填写,单位 W。填错会导致底盘躺平(填0)或超功率(填过大)。
#define CHASSIS_MAX_POWER 45.0f

/* ---- 云台参数 ---- */
// 云台与底盘对齐(云台指向 == 底盘正前方)时的 **yaw 电机角**, 单位 度。
// 注意这不是 IMU 角度: IMU 装在 yaw 轴上, 给出的是云台的世界系绝对指向, 不能用来算
// 云台相对底盘的夹角。相对夹角只能从 yaw 电机编码器取。
//
// ⚠️ DMMotorInit 每次上电都会执行 DMMotorCaliEncoder, 把上电瞬间的姿态设为电机零点,
//    所以 yaw 电机角是**相对上电姿态**的。因此操作约定是:
//    **上电时把云台摆到与底盘正前方一致**, 此时电机角为 0, 保持本宏为 0.0f 即可。
//    若上电时云台是歪的, 跟随模式和高速平移都会带上一个固定的角度偏差。
// [实车核对]
#define YAW_CHASSIS_ALIGN_ANGLE 0.0f
// yaw 软件限位, 单位 rad, 相对对齐零位(即上电零位); 0 表示禁用限位(有滑环可无限旋转的机构)
// 限位在**电机角**空间判断——机械限位约束的本来就是云台相对底盘的夹角。
// [实车核对] 取决于新车 yaw 是否有机械限位
#define YAW_MAX_ANGLE 0.0f
#define YAW_MIN_ANGLE 0.0f
// pitch 软件限位,单位 rad,相对电机零位(上电姿态即为零位);0 表示禁用
// [实车核对]
#define PITCH_MAX_ANGLE 0.0f
#define PITCH_MIN_ANGLE 0.0f

/* ---- 底盘小陀螺 ---- */
// 小陀螺自旋转速,单位 °/s,与 Chassis_Ctrl_Cmd_s.wz 量纲一致
// 按需求保持与哨兵一致(2000)。注意实际转速还受 CHASSIS_MOTOR_TO_CENTER 影响, 见下方说明。
#define CHASSIS_SPIN_BASE_WZ 2000.0f

/* ---- 发射机构总开关 ---- */
// 取消注释即启用摩擦轮/拨盘。
// 未定义时 robot.c 不会调用 ShootInit()/ShootTask(), 发射电机完全不注册:
// 既不会有 CAN 控制帧发到总线上, 也不会产生 daemon 离线报警,
// 底盘和云台可以不受发射机构没接线的影响独立调试。
// robot_cmd 侧的 shoot_cmd/shoot_feed 话题仍然完整(它在自己的 Init 里注册),
// 只是"发了没人收、订阅的队列始终为空", 消息中心对这两种情况都有安全处理。
// #define SHOOT_ENABLE

/* ---- 发射参数 ---- */
#define ONE_BULLET_DELTA_ANGLE 36    // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出, [实车核对]
#define REDUCTION_RATIO_LOADER 36.0f // 2006拨盘电机的减速比,英雄需要修改为3508的19.0f
#define NUM_PER_CIRCLE 10            // 拨盘一圈的装载量, [实车核对]
// 发射卡弹检测参数(需实测)
#define BLOCK_TIME 50.0f          // 拨盘卡弹检测时间(ms): 连发下实际转速低于阈值持续该时间判定卡弹
#define BLOCK_TRIGGER_SPEED 300   // 卡弹判定速度阈值(°/s,转子转速),正常连发上万,堵转时接近0
#define REVERSE_TIMEOUT 1000.0f   // 角度环反转清除卡弹超时兜底(ms): 机械卡死角度环走不到位时强制恢复,防止连发永久中断
#define LOADER_REVERSE_SPEED 10368 // 手动反转(LOAD_REVERSE)的转子转速(°/s,速度环); 卡弹反转已改角度环,仅手动模式使用

// 检查是否出现主控板定义冲突,只允许一个开发板定义存在,否则编译会自动报错
#if (defined(ONE_BOARD) && defined(CHASSIS_BOARD)) || \
    (defined(ONE_BOARD) && defined(GIMBAL_BOARD)) ||  \
    (defined(CHASSIS_BOARD) && defined(GIMBAL_BOARD))
#error Conflict board definition! You can only define one board type.
#endif

#pragma pack(1) // 压缩结构体,取消字节对齐,下面的数据都可能被传输
/* -------------------------基本控制模式和数据类型定义-------------------------*/
/**
 * @brief 这些枚举类型和结构体会作为CMD控制数据和各应用的反馈数据的一部分
 *
 */
// 机器人状态
typedef enum
{
    ROBOT_STOP = 0,
    ROBOT_READY,
} Robot_Status_e;

// 应用状态
typedef enum
{
    APP_OFFLINE = 0,
    APP_ONLINE,
    APP_ERROR,
} App_Status_e;

// 底盘模式设置
typedef enum
{
    CHASSIS_ZERO_FORCE = 0,    // 电流零输入/急停
    CHASSIS_ROTATE,            // 小陀螺自旋
    CHASSIS_NO_FOLLOW,         // 不跟随云台,允许全向平移,底盘不旋转
    CHASSIS_FOLLOW_GIMBAL_YAW, // 跟随云台,底盘叠加角度环控制
} chassis_mode_e;

// 云台模式设置
typedef enum
{
    GIMBAL_ZERO_FORCE = 0, // 电流零输入
    GIMBAL_GYRO_MODE,      // 陀螺仪反馈模式: yaw 锁 IMU 绝对角,反馈值为 IMU YawTotalAngle/Gyro
    GIMBAL_AUTO,           // 自瞄模式。速度前馈通路已就绪,当前无触发源(上位机已移除),留给后续加视觉
} gimbal_mode_e;

// 发射模式设置
typedef enum
{
    SHOOT_OFF = 0,
    SHOOT_ON,
} shoot_mode_e;
typedef enum
{
    FRICTION_OFF = 0, // 摩擦轮关闭
    FRICTION_ON,      // 摩擦轮开启
} friction_mode_e;

typedef enum
{
    LID_OPEN = 0, // 弹舱盖打开
    LID_CLOSE,    // 弹舱盖关闭
} lid_mode_e;

typedef enum
{
    LOAD_STOP = 0,  // 停止发射
    LOAD_REVERSE,   // 反转
    LOAD_1_BULLET,  // 单发
    LOAD_3_BULLET,  // 三发
    LOAD_BURSTFIRE, // 连发
} loader_mode_e;

/* ----------------CMD应用发布的控制数据,应当由gimbal/chassis/shoot订阅---------------- */
// cmd发布的底盘控制数据,由chassis订阅
typedef struct
{
    // 控制部分
    float vx;           // 前进方向速度
    float vy;           // 横移方向速度
    float wz;           // 旋转速度,单位 °/s
    float offset_angle; // 云台指向与底盘正方向的夹角,单位 度,范围-180~180
    chassis_mode_e chassis_mode;
    int chassis_speed_buff;
    // UI部分
    //  ...

} Chassis_Ctrl_Cmd_s;

// cmd发布的云台控制数据,由gimbal订阅
typedef struct
{ // 云台角度控制
    float yaw;            // yaw目标角度, 单位: rad, 与 IMU YawTotalAngle(弧度制)一致
    float pitch;          // pitch目标角度, 单位: rad, 与 DM4310 motor angle_total 一致
    float yaw_ff_speed;   // yaw角速度前馈, 单位: rad/s
    float yaw_ff_acc;     // yaw角加速度前馈, 单位: rad/s^2
    float pitch_ff_speed; // pitch角速度前馈, 单位: rad/s
    float pitch_ff_acc;   // pitch角加速度前馈, 单位: rad/s^2

    gimbal_mode_e gimbal_mode;
} Gimbal_Ctrl_Cmd_s;

// cmd发布的发射控制数据,由shoot订阅
typedef struct
{
    shoot_mode_e shoot_mode;
    loader_mode_e load_mode;
    loader_mode_e last_lode_mode; // 上一次发射模式(边沿检测用,单发触发判断)
    lid_mode_e lid_mode;
    friction_mode_e friction_mode;
    Bullet_Speed_e bullet_speed; // 弹速枚举
    float shoot_rate; // 连续发射的射频,unit per s,发/秒
    int16_t shoot_flag; // 单发限位标志位: 1=触发, 2=发射中/到位保持, 0=可再发
} Shoot_Ctrl_Cmd_s;

/* ----------------gimbal/shoot/chassis发布的反馈数据----------------*/
/**
 * @brief 由cmd订阅,其他应用也可以根据需要获取.
 *
 */

typedef struct
{
    float real_vx; // 底盘真实前进速度(mm/s),由EstimateSpeed()逆解算得到
    float real_vy; // 底盘真实横移速度(mm/s)
    float real_wz; // 底盘真实旋转角速度(°/s),与Chassis_Ctrl_Cmd_s.wz量纲一致
} Chassis_Upload_Data_s;


typedef struct
{
    attitude_t gimbal_imu_data; // 云台姿态(IMU装在yaw轴上,随云台转动,故Yaw是云台的世界系绝对指向)
    float yaw_motor_angle;      // yaw电机多圈角(rad), 即**云台相对底盘的夹角**。
                                // robot_cmd 的 offset_angle 必须用它算, 不能用 IMU 的 YawTotalAngle:
                                // 后者是世界系绝对指向, 底盘转动时它不变, 拿去做跟随环反馈会导致
                                // 增益为0(底盘一直转不收敛), 拿去做平移坐标系映射也会串方向。
    float pitch_motor_angle;    // pitch电机多圈角(rad), 与 Gimbal_Ctrl_Cmd_s.pitch 同坐标
                                // 供 robot_cmd 在急停恢复/切模式时把目标对齐到当前姿态,避免猛冲
} Gimbal_Upload_Data_s;

typedef struct
{
    int16_t feedback_shoot_flag; // 单发发射状态反馈: 2=发射中, 0=完成
    int16_t dadan;               // 单发限位(有无弹): 0=有弹可发, 1=无弹; 无传感器时保持0
} Shoot_Upload_Data_s;

#pragma pack() // 开启字节对齐,结束前面的#pragma pack(1)

#endif // !ROBOT_DEF_H
