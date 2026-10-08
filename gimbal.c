/**
 * @file gimbal.c
 * @brief 云台应用: 单 yaw(DM4310) + 单 pitch(DM4310)
 *
 *        单位约定(重要): yaw 与 pitch 的目标/反馈**全部使用弧度制**。
 *        - yaw   : 角度反馈来自 IMU YawTotalAngle(原生为度, 每周期转成 rad),
 *                  速度反馈直接取 IMU Gyro[Z](原生 rad/s, 无需换算)
 *        - pitch : 角度/速度反馈均来自 DM4310 电机本体(angle_total / velocity, 原生 rad)
 *        两轴单位统一, PID 增益可以直接对照, 不会出现度/弧度混用导致增益差 57.3 倍的坑。
 *
 *        控制策略: yaw 始终锁 IMU 绝对角(世界坐标系), 底盘模式(跟随/不跟随/小陀螺)
 *        由 robot_cmd 决定, 云台这边不区分。
 */

#include "gimbal.h"
#include "robot_def.h"
#include "dmmotor.h"
#include "ins_task.h"
#include "message_center.h"
#include "daemon.h"
#include "general_def.h"
#include "bsp_log.h"

static attitude_t *gimba_IMU_data;     // 云台IMU数据
static DMMotorInstance *yaw_motor;     // yaw: DM4310, IMU绝对角反馈
static DMMotorInstance *pitch_motor;   // pitch: DM4310, 电机多圈角反馈

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

/* 每周期从 IMU 刷新的弧度制反馈量, 供 DMMotor 以 OTHER_FEED 解引用。
 * 注意: DMMotor 对 other_angle_feedback_ptr / other_speed_feedback_ptr **没有判空保护**
 * (见 dmmotor.c 的角度环/速度环), 所以这两个变量的地址必须在 DMMotorInit 之前就有效。
 * 它们具有静态存储期, 天然满足这个条件; 但**绝不能**改成局部变量或运行时才分配的指针。 */
static float yaw_angle_rad = 0.0f;   // IMU YawTotalAngle 转弧度
static float yaw_speed_rad_s = 0.0f; // IMU Gyro[Z], 原生 rad/s

static float yaw_target = 0.0f;       // yaw目标角(rad), 绝对角(世界坐标系)
static float pitch_target = 0.0f;     // pitch目标角(rad), 电机多圈角
static float yaw_speed_ff = 0.0f;     // yaw速度前馈(rad/s), 自瞄用
static float pitch_speed_ff = 0.0f;   // pitch速度前馈(rad/s), 自瞄用
static uint8_t dm_yaw_offline = 0;   // yaw 离线标志,边沿触发日志用
static uint8_t dm_pitch_offline = 0; // pitch 离线标志,边沿触发日志用


static float GimbalLimitPitch(float ref)
{
    if (PITCH_MAX_ANGLE != 0.0f && ref > PITCH_MAX_ANGLE)
        return PITCH_MAX_ANGLE;
    if (PITCH_MIN_ANGLE != 0.0f && ref < PITCH_MIN_ANGLE)
        return PITCH_MIN_ANGLE;
    return ref;
}



static float GimbalLimitYaw(float ref)
{
    if (YAW_MAX_ANGLE == 0.0f && YAW_MIN_ANGLE == 0.0f)
        return ref; // 未启用限位

    float align = YAW_CHASSIS_ALIGN_ANGLE * DEGREE_2_RAD;
    float motor_rel = yaw_motor->measure.angle_total - align; // 当前相对角(相对对齐零位)
    float rel_target = motor_rel + (ref - yaw_angle_rad);     // 目标相对角

    if (YAW_MAX_ANGLE != 0.0f && rel_target > YAW_MAX_ANGLE)
        rel_target = YAW_MAX_ANGLE;
    if (YAW_MIN_ANGLE != 0.0f && rel_target < YAW_MIN_ANGLE)
        rel_target = YAW_MIN_ANGLE;

    return yaw_angle_rad + (rel_target - motor_rel);
}

void GimbalInit()
{
    gimba_IMU_data = INS_Init(); // IMU先初始化,姿态数据供yaw以OTHER_FEED闭环

    // ---- yaw: DM4310, 角度/速度均取自IMU(OTHER_FEED) ----
    
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &CAN_YAW,
            .tx_id = ID_YAW_TX,
            .rx_id = ID_YAW_RX,
        },
        .controller_param_init_config = {
            .angle_PID = { // 角度环: 输入rad(误差), 输出rad/s(速度参考)
                .Kp = 5.0f,
                .Ki = 0,
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 10.0f,
                .MaxOut = 3.0f, // rad/s, 即最大回转速度约172°/s
            },
            .speed_PID = { // 速度环: 输入rad/s, 输出N·m(力矩)
                .Kp = 1.0f,
                .Ki = 0,
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 10.0f,
                .MaxOut = 10.0f, // N·m
            },
            .other_angle_feedback_ptr = &yaw_angle_rad,   
            .other_speed_feedback_ptr = &yaw_speed_rad_s, 
            .speed_feedforward_ptr = &yaw_speed_ff,       // 自瞄速度前馈
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .feedforward_flag = SPEED_FEEDFORWARD,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
    };
    yaw_motor = DMMotorInit(&yaw_config);

    // ---- pitch: DM4310, 角度/速度均取自电机本体(MOTOR_FEED) ----
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &CAN_PITCH,
            .tx_id = ID_PITCH_TX,
            .rx_id = ID_PITCH_RX,
        },
        .controller_param_init_config = {
            .angle_PID = { // 角度环: 输入rad, 输出rad/s
                .Kp = 1.0f,
                .Ki = 0,
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 10.0f,
                .MaxOut = 3.0f, // rad/s
            },
            .speed_PID = { // 速度环: 输入rad/s, 输出N·m
                .Kp = 0.5f,
                .Ki = 0,
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 10.0f,
                .MaxOut = 10.0f, // N·m
            },
            .speed_feedforward_ptr = &pitch_speed_ff, // 自瞄速度前馈
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .feedforward_flag = SPEED_FEEDFORWARD,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
    };
    pitch_motor = DMMotorInit(&pitch_config);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

/* 机器人云台控制核心任务: 两电机(yaw DM4310 + pitch DM4310) */
void GimbalTask()
{
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

    // 每周期从IMU刷新弧度制反馈量, 供DM以OTHER_FEED读取(解引用的是这里的静态变量)
    // 注意取负号: IMU 的 yaw 正方向 与 yaw 电机的正转方向相反(装反了), 不取负会导致
    // 角度环/速度环正反馈, 电机一上电就疯转。方向确认后如需反转再改回。
    if (gimba_IMU_data != NULL)
    {
        yaw_angle_rad = -gimba_IMU_data->YawTotalAngle * DEGREE_2_RAD;
        yaw_speed_rad_s = -gimba_IMU_data->Gyro[2];
    }

    // 速度前馈: 仅自瞄模式消费, 其余模式清零避免残留旧值
    if (gimbal_cmd_recv.gimbal_mode == GIMBAL_AUTO)
    {
        yaw_speed_ff = gimbal_cmd_recv.yaw_ff_speed;     // rad/s
        pitch_speed_ff = gimbal_cmd_recv.pitch_ff_speed; // rad/s
    }
    else
    {
        yaw_speed_ff = 0.0f;
        pitch_speed_ff = 0.0f;
    }

    /* DM电机离线保护。
     * DMMotor 模块自身没有任何掉线处理(DMMotorLostCallback 是空函数, lost_cnt 从未被读写),
     * 掉线后 DMMotorTask 仍会每2ms把最后一次的力矩指令重发出去, 云台不会自己停。
     * 这里用 daemon 兜底: DM 收到一帧 CAN 就喂狗, reload_count=10 @100Hz ≈ 100ms 无反馈判定离线。
     *
     * 策略: **只停掉离线的那一个**, 在线的照常控制。这样单个电机调试时(比如 pitch ID
     * 还没确认), yaw 仍可单独用遥控器操控, 不用被离线的另一轴拖累。 */
    uint8_t yaw_online = DaemonIsOnline(yaw_motor->motor_daemon);
    uint8_t pitch_online = DaemonIsOnline(pitch_motor->motor_daemon);

    // 边沿触发日志 + 离线电机立即停(本函数 200Hz, 不边沿会刷屏)
    if (!yaw_online)
    {
        if (!dm_yaw_offline) { dm_yaw_offline = 1; LOGERROR("[gimbal] yaw DM offline"); }
        DMMotorStop(yaw_motor);
    }
    else if (dm_yaw_offline) { dm_yaw_offline = 0; LOGINFO("[gimbal] yaw DM back online"); }

    if (!pitch_online)
    {
        if (!dm_pitch_offline) { dm_pitch_offline = 1; LOGERROR("[gimbal] pitch DM offline"); }
        DMMotorStop(pitch_motor);
    }
    else if (dm_pitch_offline) { dm_pitch_offline = 0; LOGINFO("[gimbal] pitch DM back online"); }

    /* 注意: 这里不再做"切模式时把参考值同步到当前姿态"的动作。
     * 原因是本应用的目标值每周期都从 cmd 取(见下方 switch), 在这里同步会被立刻覆盖, 属于死代码。
     * 真正需要的对齐发生在 robot_cmd 侧: 急停期间不读取摇杆/鼠标, 恢复时由 GimbalRefResync()
     * 把 yaw/pitch 目标重新对齐到当前姿态, 避免用停机前的旧目标闭环导致猛冲。 */

    switch (gimbal_cmd_recv.gimbal_mode)
    {
    // 停止
    case GIMBAL_ZERO_FORCE:
        DMMotorStop(yaw_motor);
        DMMotorStop(pitch_motor);
        break;

    // 陀螺仪模式 / 自瞄模式: 两轴都用目标角度闭环, yaw锁IMU绝对角
    case GIMBAL_GYRO_MODE:
    case GIMBAL_AUTO:
        yaw_target = GimbalLimitYaw(gimbal_cmd_recv.yaw);     // rad
        pitch_target = GimbalLimitPitch(gimbal_cmd_recv.pitch); // rad

        // 在线的电机才控制, 离线的保持 stop 状态
        if (yaw_online)
        {
            DMMotorEnable(yaw_motor);
            DMMotorOuterLoop(yaw_motor, ANGLE_LOOP);
            DMMotorSetRef(yaw_motor, yaw_target);
        }
        if (pitch_online)
        {
            DMMotorEnable(pitch_motor);
            DMMotorOuterLoop(pitch_motor, ANGLE_LOOP);
            DMMotorSetRef(pitch_motor, pitch_target);
        }
        break;

    default:
        break;
    }

    // 反馈: 整份IMU姿态(含YawTotalAngle/Pitch/Gyro)
    if (gimba_IMU_data != NULL)
        gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
    // 两个电机角: yaw电机角是"云台相对底盘的夹角", robot_cmd 的 offset_angle 靠它;
    // pitch电机角供 robot_cmd 在急停恢复时对齐目标
    gimbal_feedback_data.yaw_motor_angle = yaw_motor->measure.angle_total;
    gimbal_feedback_data.pitch_motor_angle = pitch_motor->measure.angle_total;

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}
