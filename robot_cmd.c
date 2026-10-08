/**
 * @file robot_cmd.c
 * @brief 机器人核心控制应用(整车"大脑")
 *
 *        输入来源: 遥控器(DBUS) / 键鼠。上位机(自瞄/导航)已移除。
 *        输出: 把控制输入翻译成定量的控制目标, 通过消息中心发布给 gimbal/chassis/shoot。
 *
 *        单位约定: 发给云台的 yaw/pitch 目标均为**弧度制**, 与 gimbal 应用内部一致。
 *                  offset_angle 为**度**, 与 chassis 的三角函数解算一致。
 */

// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

#include <math.h> // fmodf

/* cmd应用包含的模块实例指针和交互信息存储*/
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
#include "can_comm.h"
static CANCommInstance *cmd_can_comm; // 双板通信
#endif
#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
#endif                                 // ONE_BOARD

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息

static RC_ctrl_t *rc_data; // 遥控器数据,初始化时返回

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static Robot_Status_e robot_state;  // 机器人整体工作状态
static uint8_t remote_online_flag = 0; // 遥控器在线状态,用于online/offline日志的边沿触发
static uint8_t zero_force_flag = 0; // 零力模式(右开关下档)边沿标志,切回正常档时需对齐云台目标

void RobotCMDInit()
{
    rc_data = RemoteControlInit(&huart3); // 遥控器(DBUS),当前整车唯一的对外通信接口

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

#ifdef ONE_BOARD // 双板兼容
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x312,
            .rx_id = 0x311,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    };
    cmd_can_comm = CANCommInit(&comm_conf);
#endif // GIMBAL_BOARD

    gimbal_cmd_send.yaw = 0;
    gimbal_cmd_send.pitch = 0;
    // 弹速档位必须给初值: 上位机移除后没有任何来源会自动设定它,
    // 保持0的话 shoot.c 的 switch 会落到 default 分支, 摩擦轮永远拿不到转速。
    shoot_cmd_send.bullet_speed = SMALL_AMU_18;

    robot_state = ROBOT_READY; // 启动时机器人进入工作模式
}

/**
 * @brief 计算云台指向相对底盘正方向的夹角 offset_angle。
 *        单位角度制, 范围-180~180, 用于底盘跟随云台以及把平移指令从云台系映射到底盘系。
 *
 *        ⚠️ 必须用 **yaw 电机角**, 不能用 IMU 的 YawTotalAngle。
 *        IMU 装在 yaw 轴上, 它给的是云台的**世界系绝对指向**: 底盘转动时这个值不变,
 *        拿去做跟随环的反馈会让误差恒为常数(增益为0, 底盘一直转不收敛),
 *        拿去做平移的坐标系映射会让前进方向不跟随云台。
 *        而 yaw 电机角天生就是"云台相对底盘转了多少", 才是这里需要的量。
 */
static void CalcOffsetAngle()
{
    // 电机角是多圈无界值, 用 fmodf 环绕而不是单次 if
    float rel = gimbal_fetch_data.yaw_motor_angle * RAD_2_DEGREE - YAW_CHASSIS_ALIGN_ANGLE;
    rel = fmodf(rel, 360.0f); // 先收到 (-360, 360)
    if (rel > 180.0f)
        rel -= 360.0f;
    else if (rel < -180.0f)
        rel += 360.0f;

    chassis_cmd_send.offset_angle = rel;
}

/**
 * @brief 把云台的参考目标重新对齐到当前真实姿态。
 *        急停恢复时必须调用: 急停期间摇杆/鼠标仍在被读取, 若不重新对齐,
 *        恢复瞬间会用停机期间积累的旧目标闭环, 云台会猛地甩过去。
 *        yaw 从 IMU 读, pitch 从 gimbal 回传的电机角读。
 */
static void GimbalRefResync()
{
    gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle * DEGREE_2_RAD;
    gimbal_cmd_send.pitch = gimbal_fetch_data.pitch_motor_angle;
}

/**
 * @brief 遥控器右开关三档决定底盘与云台的运行模式。遥控器和键鼠共用。
 *        [下] 零力模式: 底盘+云台一起泄力(电机电流归零, 可自由转动), 用于调试/收车
 *        [中] 云台锁 IMU 绝对角(GIMBAL_GYRO_MODE), 底盘不转, 保持全向平移
 *        [上] 云台锁 IMU 绝对角, 底盘小陀螺自旋
 *
 *        零力模式替换掉了原来的"底盘跟随云台"(CHASSIS_FOLLOW_GIMBAL_YAW),
 *        该枚举仍保留在 robot_def.h 中, 只是不再由任何开关触发。
 *        零力期间摇杆/鼠标增量仍在累积, 切回中/上档时必须先 GimbalRefResync()
 *        把云台目标对齐到当前姿态, 否则会用零力期间的旧目标闭环导致猛冲。
 */
static void ChassisGimbalModeSet()
{
    if (switch_is_down(rc_data[TEMP].rc.switch_right)) // [下] 零力模式: 底盘+云台一起泄力
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        zero_force_flag = 1; // 记录进入零力, 切回时需对齐云台目标
    }
    else // [中] 不跟随 / [上] 小陀螺, 云台行为一致
    {
        // 刚从零力切回: 先对齐云台目标再闭环, 避免猛冲(见函数注释)
        if (zero_force_flag)
        {
            zero_force_flag = 0;
            GimbalRefResync();
        }
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE; // 两档下云台行为一致

        if (switch_is_mid(rc_data[TEMP].rc.switch_right)) // [中] 底盘不转,保持全向平移
            chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        else if (switch_is_up(rc_data[TEMP].rc.switch_right)) // [上] 小陀螺自旋
            chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        else // 开关值异常(离线时已被 EmergencyHandler 拦下, 这里纯属兜底)
            chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW; // 兜底不落到自旋
    }
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet()
{
    // 保存上一帧发射模式(边沿检测用,需在load_mode被覆盖前)
    shoot_cmd_send.last_lode_mode = shoot_cmd_send.load_mode;

    // 云台: 摇杆量为增量, yaw 按 rad 叠加(0.005°/单位量转成弧度), pitch 本身即为 rad
    gimbal_cmd_send.yaw += 0.005f * (float)rc_data[TEMP].rc.rocker_l_ * DEGREE_2_RAD;
    gimbal_cmd_send.pitch += 0.001f * (float)rc_data[TEMP].rc.rocker_l1;

    // 底盘: 左摇杆全向平移
    chassis_cmd_send.vx = 10.0f * (float)rc_data[TEMP].rc.rocker_r_;  // 水平方向
    chassis_cmd_send.vy = 10.0f * (float)rc_data[TEMP].rc.rocker_r1;  // 竖直方向

    // 摩擦轮控制: 拨轮向上打为负, 向下为正
    if (rc_data[TEMP].rc.dial < -100) // 向上超过100,打开摩擦轮
        shoot_cmd_send.friction_mode = FRICTION_ON;
    else
        shoot_cmd_send.friction_mode = FRICTION_OFF;
    // 拨弹控制: 遥控器固定为连发模式
    if (rc_data[TEMP].rc.dial < -500)
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
    else
        shoot_cmd_send.load_mode = LOAD_STOP;

    // 弹速档位: 遥控器通道没有对应的调节方式, 固定18m/s档(按实际弹道标定)
    shoot_cmd_send.bullet_speed = SMALL_AMU_18;

    // ---- 单发 shoot_flag 状态机: 回读发射反馈, 无弹时置1触发 ----
    shoot_cmd_send.shoot_flag = shoot_fetch_data.feedback_shoot_flag; // 回读单发状态
    if (shoot_cmd_send.load_mode == LOAD_1_BULLET &&
        shoot_cmd_send.shoot_flag == 0 && shoot_fetch_data.dadan == 0)
    {
        shoot_cmd_send.shoot_flag = 1; // 触发单发
    }
    if (shoot_cmd_send.load_mode == LOAD_1_BULLET &&
        shoot_cmd_send.last_lode_mode != LOAD_1_BULLET && shoot_fetch_data.dadan == 0)
    {
        shoot_cmd_send.shoot_flag = 1; // 刚切到单发也触发
    }

    shoot_cmd_send.shoot_rate = 8; // 连发射频, 发/秒
}

/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet()
{
    // 保存上一帧发射模式(边沿检测用,需在load_mode被覆盖前)
    shoot_cmd_send.last_lode_mode = shoot_cmd_send.load_mode;

    chassis_cmd_send.vx = rc_data[TEMP].key[KEY_PRESS].w * 300 - rc_data[TEMP].key[KEY_PRESS].s * 300;
    chassis_cmd_send.vy = rc_data[TEMP].key[KEY_PRESS].a * 300 - rc_data[TEMP].key[KEY_PRESS].d * 300;

    // 云台: 鼠标位移为增量, 统一按 rad 叠加
    gimbal_cmd_send.yaw += (float)rc_data[TEMP].mouse.x / 660.0f * 10.0f * DEGREE_2_RAD;
    gimbal_cmd_send.pitch += (float)rc_data[TEMP].mouse.y / 660.0f * 10.0f * DEGREE_2_RAD;

    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_Z] % 3) // Z键设置弹速
    {
    case 0:
        shoot_cmd_send.bullet_speed = SMALL_AMU_15;
        break;
    case 1:
        shoot_cmd_send.bullet_speed = SMALL_AMU_18;
        break;
    default:
        shoot_cmd_send.bullet_speed = SMALL_AMU_30;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_E] % 4) // E键设置发射模式
    {
    case 0:
        shoot_cmd_send.load_mode = LOAD_STOP;
        break;
    case 1:
        shoot_cmd_send.load_mode = LOAD_1_BULLET;
        break;
    case 2:
        shoot_cmd_send.load_mode = LOAD_3_BULLET;
        break;
    default:
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        break;
    }

    // ---- 单发 shoot_flag 状态机: 回读发射反馈, 无弹时置1触发 ----
    shoot_cmd_send.shoot_flag = shoot_fetch_data.feedback_shoot_flag; // 回读单发状态
    if (shoot_cmd_send.load_mode == LOAD_1_BULLET &&
        shoot_cmd_send.shoot_flag == 0 && shoot_fetch_data.dadan == 0)
    {
        shoot_cmd_send.shoot_flag = 1; // 触发单发
    }
    if (shoot_cmd_send.load_mode == LOAD_1_BULLET &&
        shoot_cmd_send.last_lode_mode != LOAD_1_BULLET && shoot_fetch_data.dadan == 0)
    {
        shoot_cmd_send.shoot_flag = 1; // 刚切到单发也触发
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_R] % 2) // R键开关弹舱
    {
    case 0:
        shoot_cmd_send.lid_mode = LID_OPEN;
        break;
    default:
        shoot_cmd_send.lid_mode = LID_CLOSE;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_F] % 2) // F键开关摩擦轮
    {
    case 0:
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        break;
    default:
        shoot_cmd_send.friction_mode = FRICTION_ON;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_C] % 4) // C键设置底盘速度档位
    {
    case 0:
        chassis_cmd_send.chassis_speed_buff = 40;
        break;
    case 1:
        chassis_cmd_send.chassis_speed_buff = 60;
        break;
    case 2:
        chassis_cmd_send.chassis_speed_buff = 80;
        break;
    default:
        chassis_cmd_send.chassis_speed_buff = 100;
        break;
    }
    shoot_cmd_send.shoot_rate = 8; // 连发射频, 发/秒
}

/**
 * @brief 把所有应用切到停止状态
 */
static void SetAllStop()
{
    gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    shoot_cmd_send.shoot_mode = SHOOT_OFF;
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    shoot_cmd_send.load_mode = LOAD_STOP;
}

/**
 * @brief  紧急停止与恢复。
 *         急停触发: 遥控器离线, 或遥控器拨轮向下打满(dial > 300)
 *         恢复: 上述条件都解除后**自动恢复**
 */
static void EmergencyHandler()
{
    /* 遥控器离线必须优先判断, 且必须停车。
     * 原因: RCLostCallback(remote_control.c) 掉线时会把 rc_ctrl 整体清零, 而开关值为 0 时
     * switch_is_down/mid/up 全部为假, 会落进 ChassisGimbalModeSet() 的 else 分支(小陀螺),
     * 结果是"遥控器一掉线, 底盘就自己转起来"。忘开遥控器、换遥控器没对上码、拔接收机都会触发。
     *
     * 在线/离线都做成边沿触发打日志(本函数 200Hz 运行, 不这样会刷屏)。
     * 这两条日志是排查"遥控器没反应"最快的入口:
     *   只看到 online 之后一切正常       -> DBUS 通, 问题在别处
     *   online 紧接着 offline, 反复横跳  -> DBUS 帧收不到/收不全, 查接收机接线与对码
     *   从上电起就是 offline             -> 同上, 且底盘会被这条保护锁死(这是预期行为) */
    if (!RemoteControlIsOnline())
    {
        if (remote_online_flag) // 只在"刚刚掉线"时打一次
        {
            remote_online_flag = 0;
            LOGERROR("[CMD] remote offline, emergency stop!");
        }
        robot_state = ROBOT_STOP;
        SetAllStop();
        return;
    }
    else if (!remote_online_flag) // 刚刚恢复在线
    {
        remote_online_flag = 1;
        LOGINFO("[CMD] remote online, DBUS OK");
    }

    if (rc_data[TEMP].rc.dial > 300) // 拨轮向下打满, 进入急停
    {
        if (robot_state != ROBOT_STOP)
            LOGERROR("[CMD] emergency stop!");

        robot_state = ROBOT_STOP;
        SetAllStop();
    }
    else if (robot_state == ROBOT_STOP) // 拨轮已回中 / 遥控器恢复在线, 自动恢复
    {
        robot_state = ROBOT_READY;
        shoot_cmd_send.shoot_mode = SHOOT_ON;
        GimbalRefResync(); // 关键: 把云台目标对齐到当前姿态, 否则恢复瞬间会猛冲
        LOGINFO("[CMD] reinstate, robot ready");
    }
}

/* 机器人核心控制任务,200Hz频率运行 */
void RobotCMDTask()
{
    // 从其他应用获取回传数据
#ifdef ONE_BOARD
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
#endif // GIMBAL_BOARD
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);

    // 根据gimbal的反馈值计算云台和底盘正方向的夹角
    CalcOffsetAngle();

    // 先判定急停。急停期间不再读取摇杆/鼠标, 避免停机期间参考值继续累积。
    EmergencyHandler();

    if (robot_state != ROBOT_STOP)
    {
        // 模式由右开关决定(遥控器与键鼠共用)
        ChassisGimbalModeSet();

        // 键鼠 / 遥控器二选一, 由左开关决定; 中位按遥控器处理, 避免停留在无输入状态
        if (switch_is_up(rc_data[TEMP].rc.switch_left))
            MouseKeySet();
        else
            RemoteControlSet();
    }

    // 推送消息
#ifdef ONE_BOARD
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
#endif // GIMBAL_BOARD
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
}
