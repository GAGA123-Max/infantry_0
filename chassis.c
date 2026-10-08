/**
 * @file chassis.c
 * @brief 底盘应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
 *        注意底盘采取右手系,对于平面视图,底盘纵向运动的正前方为x正方向;横向运动的右侧为y正方向
 *
 *        底盘形式: 四全向轮 45°斜装(轮子滚动方向与车体轴线成45°), 因此平移项带 sin45/cos45 投影。
 *        裁判系统已移除, 底盘功率上限改用 robot_def.h 中的 CHASSIS_MAX_POWER 固定值。
 */

#include "chassis.h"
#include "robot_def.h"
#include "power_control.h"
#include "message_center.h"
#include "referee_task.h"

#include "general_def.h"
#include "arm_math.h"
#include "bsp_log.h"

#include <math.h> // fabsf

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
#ifdef CHASSIS_BOARD // 如果是底盘板,使用板载IMU获取底盘转动角速度
#include "can_comm.h"
#include "ins_task.h"
static CANCommInstance *chasiss_can_comm; // 双板通信CAN comm
attitude_t *Chassis_IMU_data;
#endif // CHASSIS_BOARD
#ifdef ONE_BOARD
static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令
#endif                                              // !ONE_BOARD
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据

static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back

static referee_info_t *referee_data;       // 裁判系统数据(功率上限/热量/发弹量/机器人ID)
static Referee_Interactive_info_t ui_data; // UI 交互数据; uiTask 当前未创建, 这里只是 UITaskInit 的入参

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy;     // 将云台系的速度投影到底盘
static float vt_lf, vt_rf, vt_lb, vt_rb; // 底盘速度解算后的临时输出,待进行限幅

void ChassisInit()
{
    // 四个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle = &CAN_CHASSIS,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 4.5,
                .Ki = 0,
                .Kd = 0,
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 15000,
                .Output_LPF_RC = 0.3,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP, // 设置为开环，电机设定值由下面的功率控制设定，不走普通的pid
            .close_loop_type = SPEED_LOOP,
        },
        .motor_type = M3508,
    };
    //  @todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
    //使用功率控制的电机需要使用PowerControlInit()函数初始化,因为电机的控制方式不同
    chassis_motor_config.can_init_config.tx_id = ID_CHASSIS_LF;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lf = PowerControlInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = ID_CHASSIS_RF;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rf = PowerControlInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = ID_CHASSIS_LB;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lb = PowerControlInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = ID_CHASSIS_RB;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rb = PowerControlInit(&chassis_motor_config);

    // 裁判系统初始化(huart6)。只取数据, 不建 UI 任务 —— 图传 UI 已停用。
    referee_data = UITaskInit(&huart6, &ui_data);

    // 发布订阅初始化,如果为双板,则需要can comm来传递消息
#ifdef CHASSIS_BOARD
    Chassis_IMU_data = INS_Init(); // 底盘IMU初始化

    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x311,
            .rx_id = 0x312,
        },
        .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .send_data_len = sizeof(Chassis_Upload_Data_s),
    };
    chasiss_can_comm = CANCommInit(&comm_conf); // can comm初始化
#endif                                          // CHASSIS_BOARD

#ifdef ONE_BOARD // 单板控制整车,则通过pubsub来传递消息
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
}

// 全向轮(Omni wheel, 45°斜装)解算参数
// 旋转半径: 使用学长实测值 CHASSIS_MOTOR_TO_CENTER(m), 转mm后换算, 含打滑修正(比理论小13%)
#define OMNI_WHEEL_SIN45 0.70710678f // sin45°: 全向轮斜装时vx在轮子滚动方向的投影系数
#define OMNI_WHEEL_COS45 0.70710678f // cos45°: 全向轮斜装时vy在轮子滚动方向的投影系数
#define LF_CENTER ((CHASSIS_MOTOR_TO_CENTER * 1000.0f + CENTER_GIMBAL_OFFSET_X - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RF_CENTER ((CHASSIS_MOTOR_TO_CENTER * 1000.0f - CENTER_GIMBAL_OFFSET_X - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define LB_CENTER ((CHASSIS_MOTOR_TO_CENTER * 1000.0f + CENTER_GIMBAL_OFFSET_X + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RB_CENTER ((CHASSIS_MOTOR_TO_CENTER * 1000.0f - CENTER_GIMBAL_OFFSET_X + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
/**
 * @brief 计算每个轮毂电机的输出,正运动学解算(全向轮, 45°斜装)
 *        与学长实现一致: 平移项带sin45/cos45投影, 旋转项直接乘旋转半径
 *        用宏进行预替换减小开销,运动解算具体过程参考教程
 */
static void MecanumCalculate()
{
    vt_lf = -chassis_vx * OMNI_WHEEL_SIN45 - chassis_vy * OMNI_WHEEL_COS45 - chassis_cmd_recv.wz * LF_CENTER;
    vt_rf = -chassis_vx * OMNI_WHEEL_SIN45 + chassis_vy * OMNI_WHEEL_COS45 - chassis_cmd_recv.wz * RF_CENTER;
    vt_lb = chassis_vx * OMNI_WHEEL_SIN45 - chassis_vy * OMNI_WHEEL_COS45 - chassis_cmd_recv.wz * LB_CENTER;
    vt_rb = chassis_vx * OMNI_WHEEL_SIN45 + chassis_vy * OMNI_WHEEL_COS45 - chassis_cmd_recv.wz * RB_CENTER;
}

/**
 * @brief 根据裁判系统和电容剩余容量对输出进行限制并设置电机参考值
 *
 */
static void LimitChassisOutput()
{
    // 功率限制由 power_control 模块的 PowerControl() 统一完成(见 SetPowerLimit)
    // 完成功率限制后进行电机参考输入设定
    DJIMotorSetRef(motor_lf, vt_lf);
    DJIMotorSetRef(motor_rf, vt_rf);
    DJIMotorSetRef(motor_lb, vt_lb);
    DJIMotorSetRef(motor_rb, vt_rb);
}

/**
 * @brief 根据每个轮子的速度反馈,计算底盘的实际运动速度,逆运动解算
 *        对于双板的情况,考虑增加来自底盘板IMU的数据
 *
 */
static void EstimateSpeed()
{
    // 由电机反馈角速度计算轮子线速度(mm/s):
    //  输出轴角速度 = 转子角速度(speed_aps,°/s) / 减速比
    //  线速度 = 输出轴角速度(rad/s) * 轮半径
    static float wheel_k = RADIUS_WHEEL * DEGREE_2_RAD / REDUCTION_RATIO_WHEEL;
    static float v_lf, v_rf, v_lb, v_rb;
    v_lf = motor_lf->measure.speed_aps * wheel_k;
    v_rf = motor_rf->measure.speed_aps * wheel_k;
    v_lb = motor_lb->measure.speed_aps * wheel_k;
    v_rb = motor_rb->measure.speed_aps * wheel_k;

    // 逆运动学解算(对称近似),符号约定与MecanumCalculate()保持一致(全向轮,含sin45投影)
    chassis_feedback_data.real_vx = (v_lb + v_rb - v_lf - v_rf) / (4.0f * OMNI_WHEEL_SIN45);
    chassis_feedback_data.real_vy = (v_rf + v_rb - v_lf - v_lb) / (4.0f * OMNI_WHEEL_COS45);
    // 旋转角速度,量纲与Chassis_Ctrl_Cmd_s.wz一致;分母为四个轮距半径之和(4倍均值)
    chassis_feedback_data.real_wz = -(v_lf + v_rf + v_lb + v_rb) / (LF_CENTER + RF_CENTER + LB_CENTER + RB_CENTER);
}

/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    // 后续增加没收到消息的处理(双板的情况)
    // 获取新的控制信息
#ifdef ONE_BOARD
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
#endif
#ifdef CHASSIS_BOARD
    chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chasiss_can_comm);
#endif // CHASSIS_BOARD

    /* 底盘功率上限: 优先用裁判系统实时值(随比赛等级/增益变化), 拿不到时退回固定宏。
     * ⚠️ 裁判系统未连接或尚未收到数据时 chassis_power_limit 为 0, 直接喂给 SetPowerLimit()
     *    会让底盘被完全限功率(原地躺平)。这是原实现的隐患, 这里必须兜底。
     * 日志只在首次回退时打一次, 避免每 5ms 刷屏。 */
    static uint8_t power_fallback_logged = 0;
    if (referee_data != NULL && referee_data->GameRobotState.chassis_power_limit > 0)
        SetPowerLimit(referee_data->GameRobotState.chassis_power_limit);
    else
    {
        SetPowerLimit(CHASSIS_MAX_POWER);
        if (!power_fallback_logged)
        {
            power_fallback_logged = 1;
            LOGWARNING("[chassis] referee offline or no data, fall back to CHASSIS_MAX_POWER");
        }
    }

    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE)
    { // 如果出现重要模块离线或遥控器设置为急停,让电机停止
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
    }
    else
    { // 正常工作
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);
    }

    // 根据控制模式设定旋转速度
    switch (chassis_cmd_recv.chassis_mode)
    {
    case CHASSIS_NO_FOLLOW: // 底盘不旋转,但维持全向机动,一般用于调整云台姿态
        chassis_cmd_recv.wz = 0;
        break;
    case CHASSIS_FOLLOW_GIMBAL_YAW: // 跟随云台,不单独设置pid,以误差角度平方为速度输出
        // 注意用 fabsf 而不是 abs: abs 是整数取绝对值, 会把 float 的 offset_angle 截断成整数
        chassis_cmd_recv.wz = -1.5f * chassis_cmd_recv.offset_angle * fabsf(chassis_cmd_recv.offset_angle);
        break;
    case CHASSIS_ROTATE: // 小陀螺自旋,保持全向机动
        chassis_cmd_recv.wz = CHASSIS_SPIN_BASE_WZ;
        break;
    default:
        break;
    }

    // 根据云台和底盘的角度offset将控制量映射到底盘坐标系上
    // 底盘逆时针旋转为角度正方向;云台命令的方向以云台指向的方向为x,采用右手系(x指向正北时y在正东)
    static float sin_theta, cos_theta;
    cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
    chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;

    // 根据控制模式进行正运动学解算,计算底盘输出
    MecanumCalculate();

    // 根据功率限制对输出限幅并设定闭环参考值
    LimitChassisOutput();

    // 根据电机的反馈速度和IMU(如果有)计算真实速度
    EstimateSpeed();

    // 推送反馈消息
#ifdef ONE_BOARD
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
#ifdef CHASSIS_BOARD
    CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
#endif // CHASSIS_BOARD
}
