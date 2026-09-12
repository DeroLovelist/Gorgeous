/**
 * @file    MissionControl.c
 * @brief   比赛任务主状态机(适配本 STM32F407 工程)
 *
 * 硬件/模块映射:
 *   - K230 视觉/扫码   : UART4(Serial.c)  -> g_k230_rx_line / g_k230_new_data_flag
 *   - 机械臂           : 飞特舵机(ServoArm.c)
 *   - 底盘             : Chassis.c(麦轮, 位置闭环 + 航向闭环)
 *   - 激光             : LASER.c (PC13)
 *   - 日志             : EasyLogger(USART3 蓝牙)
 *
 * 与 K230 的串口协议(需与 K230 端一致):
 *   MCU -> K230 : "scan_qr"         请求扫码
 *               : "run_task:<n>"    执行任务 n (1=排爆 2=打靶)
 *               : "start_align"     进入精对准(回传 D:<x>,<y>)
 *               : "qr_code:<data>"  回传二维码数据
 *               : "reset:0"         复位
 *   K230 -> MCU : "qr:<data>"       扫码结果
 *               : "C"/"L"/"R"       目标路径
 *               : "OK"              已对准
 *               : "D:<x>,<y>"       像素误差(x横向 / y纵向)
 */

#include "MissionControl.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "Serial.h"
#include "Chassis.h"
#include "ServoArm.h"
#include "LASER.h"
#include "LED.h"
#include "elog.h"

/* 日志快捷方式: 输出到 USART3(蓝牙) */
#define MLOG(...)   elog_i("MISSION", __VA_ARGS__)

/* ================= 全局变量定义 ================= */
volatile MissionState_t g_mission_state = MISSION_STATE_IDLE;   // 任务状态机当前状态
volatile uint8_t g_vision_task_in_progress = 0;                 // 视觉子状态机任务号 (0=无任务, 1=排爆, 2=打靶, 3=救援)
char g_qr_code_string[8];                                       // 扫码结果字符串(最多 7 字节 + '\0')

#if !MISSION_TEST_NO_ARM
/* 机械臂动作节拍 (非专业人员可先不改)
 * move_time: 每次动作让舵机到达目标位置的时间(ms)。
 *   取值建议 1500~3000, 默认 2500。太短→舵机甩得快易抖动/过冲;
 *   太长→整场任务变慢。
 * hold_time: 相邻两个动作之间的停顿(ms), 让动作完成后车/臂稳定。
 *   取值建议 300~1000, 默认 500。 */
static uint16_t move_time = 2500;   /* 机械臂动作时间(ms) */
static uint16_t hold_time = 500;    /* 机械臂动作间停顿(ms) */
#endif

/* =====================================================================
 * ⭐ 任务调参总区 (非专业人员主要改这里, 下面每个宏都已在下文被使用)
 * 说明: 距离宏单位为 mm; 角度宏单位为 度, 符号遵循 Chassis_Rotate 约定
 *       (负值 = 顺时针/右转, 正值 = 逆时针/左转)。
 * 改动原则: 每次只改一段, 用蓝牙日志里 “State: X -> Y” 定位是哪一段。
 * ===================================================================== */

/* ================= 超时参数 (ms) ================= */
#define BLIND_MOVE_TIMEOUT_MS   3000   /* 盲走(无视觉)一段移动的超时上限:
                                        * 到点判定失效时最长等多久就强行进入下一步。
                                        * 建议 2000~5000。太短→长距离没走完就被打断;
                                        * 太长→某段卡住时整场等很久 */
#define ALIGN_TIMEOUT_MS        30000  /* 整个视觉精对准环节的超时上限(ms):
                                        * 超时则放弃对准直接执行任务。
                                        * 建议 15000~40000 */

/* ================= 视觉精对准过程时序 (ms) ================= */
#define FINE_TUNE_COOLDOWN_MS   800    /* 每次修正动作后的冷却时间:
                                        * 期间忽略 K230 新帧, 避免指令叠车。
                                        * 建议 500~1200。太小→上一条没走完又来一条;
                                        * 太大→对准变慢 */
#define FINE_TUNE_SETTLE_MS     500    /* 判“已对准/停稳”后额外停车等待时间,
                                        * 等惯性晃动结束再执行(抓/打)。建议 300~800 */
#define FORCE_GRAB_AFTER_MS     15000  /* 精对准开始后超过该时长(ms)仍未对准:
                                        * 放弃继续对准, 直接执行任务(防超时卡死) */
#define QR_SIM_WAIT_MS          1000   /* 测试模式: 到扫码点后停车等待多久才模拟扫到码 */

/* ================= 视觉对准参数 (像素误差 -> 修正距离) ================= */
#define K_GAIN              0.5f        /* 像素误差 → 移动距离 的比例 (mm/像素)。
                                         * 建议 0.3~1.0。太大→每帧都冲过头来回振荡;
                                         * 太小→老修不到位。默认 0.5 */
#define MIN_MOVE_MM         35          /* 单次修正的最小距离(mm): 暴力起步用,
                                         * 太小的修正推不动车(静摩擦), 强制到 35mm。
                                         * 建议 20~50 */
#define MAX_MOVE_MM         120         /* 单次修正的最大距离(mm), 防一下冲太远。
                                         * 建议 80~200 */
#define ALIGN_TOLERANCE     50          /* 判定“已对准”的像素误差容差。
                                         * 建议 30~80。越小对准越准但越难达成(可能超时) */

/* ================= 排爆任务参数 ================= */
/* 任务1: 路径 C(正前方) 接近炸弹的基准距离(mm) */
#define BOMB_C1_APPROACH_DISTANCE   15.0f

//下面的参数在视觉进行排爆项目中起作用，K230传回L/R/C，表示机器人要走左边、右边还是中间的路线，下面的参数就是根据不同路线来设置机器人走的距离和角度
/* 路径 L/R 的进入与回程补偿距离(mm), 视场地布局实测微调: */
#define BOMB_L_INIT_BACK_MM         120   /* 路径 L: 进入炸弹区的初始后退补偿 */
#define BOMB_R_INIT_FWD_MM          360   /* 路径 R: 进入炸弹区的初始前进补偿 */
#define BOMB_L_RETURN_FWD_MM        360   /* 路径 L: 抓取后返回放置区的前进距离 */
#define BOMB_R_RETURN_BACK_MM       360   /* 路径 R: 抓取后返回放置区的后退距离 */

/* ================= 任务2 打靶参数 ================= */
#define TARGET_ADJUST_DISTANCE      50.0f  /* 靶标在左/右侧时横移多少(mm)去对准。建议 30~100 */
#define LASER_FIRE_DURATION_MS      2000   /* 激光点亮持续时间(ms), 即“开枪”时长。
                                            * 建议 1000~3000。光敏传感器需要足够的点亮时间 */

/* =====================================================================
 * 路线距离/角度宏 (单位: 距离 mm, 角度 度)
 * —— 想改“某一段走多远/转多少”, 改这里即可, 无需翻下面状态机。
 * ===================================================================== */

/* ---------- 阶段一: 扫码区走位 ---------- */
#define ROUTE_1_TO_QR_MM            600    /* 起点 → 二维码扫描点(直行) */
#define ROUTE_3_LEFT_A_MM           600    /* 扫码后左移 A 段 */
#define ROUTE_4_DIAG_FWD_MM         70     /* 左上斜跑: 前进分量(≈45°斜走) */
#define ROUTE_4_DIAG_LEFT_MM        70     /* 左上斜跑: 左移分量(≈45°斜走) */

/* ---------- 过斜坡段 ---------- */
#define ROUTE_5_TO_RAMP_MM          930    /* 斜坡前直行距离 */
#define ROUTE_7_LEFT_B_MM           450    /* 左移 B 段 */
#define ROUTE_7_LEFT_C_MM           440    /* 左移 C 段 */

/* ---------- 排爆区走位 ---------- */
#define ROUTE_8_TO_BOMB_AREA_MM     990    /* 直行进入排爆区 */
#define ROUTE_8B_RIGHT_MM           700    /* 右移微调 */
#define ROUTE_9_TURN_DEG            (0.1)  /* 转向排爆点(相对角度, 负=右转) */
#define ROUTE_9A_LEFT_MM            4      /* 转向后左移微调 */
#define ROUTE_10_APPROACH_MM        5     /* 接近炸弹最后一段直行 */

/* ---------- 打靶路线 (分段走位, 段间有航向校正) ---------- */
#define ROUTE_12_P1_A_MM            400//300    /* 打靶 Part1 第 1 段 */
#define ROUTE_12_P1_B_MM            400//300    /* 打靶 Part1 第 2 段 */
#define ROUTE_12_P1_C_MM            400//300    /* 打靶 Part1 第 3 段 */
#define ROUTE_12_TURN_A_DEG         300  
#define ROUTE_12_P2_A_MM            300    /* 打靶 Part2 第 1 段 */
#define ROUTE_12_P2_B_MM            200    /* 打靶 Part2 第 2 段 */
#define ROUTE_12_P2_C_MM            200    /* 打靶 Part2 第 3 段 */

/* ---------- 救援(掉头) ---------- */
#define ROUTE_14_TO_HOSTAGE_MM      600    /* 救援前前进距离 */
#define ROUTE_15_TURN_180_DEG       (-91) /* 原地掉头 180° */

/* ---------- 救援接近(新阶段四): 右移距离占位宏 (默认100, 实测后只改这里) ----------
 * 航向校正使用 Turn_Angle_Compat(0.1f) = 转到绝对 0° 的微调占位。 */
#define ROUTE_16_RIGHT_A_MM         200    /* 右移 A */
#define ROUTE_17_RIGHT_B_MM         300    /* 右移 B */
#define ROUTE_18_RIGHT_C_MM         200    /* 右移 C */
#define ROUTE_19_RIGHT_D_MM         300    /* 右移 D */
#define ROUTE_21_RIGHT_E_MM         200    /* 右移 E(视觉营救完成后) */
#define ROUTE_22_RIGHT_F_MM         300    /* 右移 F(收尾) */

/* =====================================================================
 * 机械臂动作位置 (占位值, 实际角度务必重新标定)
 * 每个动作是一组 5 个数, 依次对应 5 个舵机的位置(0~4095 ≈ 0°~360°):
 *   [0]=ID1 底座(左右转)  [1]=ID2 大臂(抬/降)
 *   [2]=ID3 辅助(姿态)   [3]=ID4 辅助(姿态)  [4]=ID5 夹爪(开/合)
 * 标定提示:
 *   - 数值 0~4095 是舵机角度码: 0=0°, 4095≈360°(飞特总线舵机)。
 *   - 只调一个动作、一个舵机时, 只改该数组里对应下标的值, 其余别动。
 *   - ⚠️ 夹爪“闭合”(CLOSE/LIFT/PLACE_* 里的第 5 个)别一次调太大,
 *     数值超出夹爪物理行程会堵转发热; 从占位值附近小幅试。
 *   - 当前为 MISSION_TEST_NO_ARM=1(测试) 时整段不参与编译。
 * ===================================================================== */
#if !MISSION_TEST_NO_ARM
static const uint16_t ARM_GRAB_OPEN[5]   = {640, 2360, 1260, 2420, 800};   /* 夹爪张开 */
static const uint16_t ARM_GRAB_LOWER[5]  = {640, 1320, 1260, 2420, 800};   /* 大臂下放 */
static const uint16_t ARM_GRAB_CLOSE[5]  = {640, 1320, 1260, 2420, 1640};  /* 夹爪闭合 */
static const uint16_t ARM_GRAB_LIFT[5]   = {640, 2360, 1260, 2420, 1640};  /* 抬臂 */
static const uint16_t ARM_PLACE_TURN[5]  = {3660, 2360, 1260, 2420, 1640}; /* 底座转向放置区 */
static const uint16_t ARM_PLACE_LOWER[5] = {3660, 1320, 1260, 2420, 1640}; /* 下放 */
static const uint16_t ARM_PLACE_OPEN[5]  = {3660, 1320, 1260, 2420, 800};  /* 松开夹爪 */
static const uint16_t ARM_PLACE_LIFT[5]  = {3660, 2360, 1260, 2420, 800};  /* 抬臂复位 */
static const uint16_t ARM_HOME[5]        = {640, 2360, 1260, 3400, 800};   /* 初始姿态 */
#endif

/**
 * @brief  读取一条新的 K230 消息到本地缓冲
 * @retval 1=有新消息, 0=无
 */
static uint8_t Mission_GetNewLine(char *dst, uint16_t maxlen)
{
    if (!g_k230_new_data_flag) {
        return 0;
    }
    g_k230_new_data_flag = 0;
    memcpy(dst, (const void *)g_k230_rx_line, maxlen - 1);
    dst[maxlen - 1] = '\0';
    return 1;
}

/**
 * @brief  兼容参考工程的转向接口
 * @note   本工程 Chassis_Rotate 为【相对旋转】; 参考工程 Turn_Angle 为绝对角度。
 *         角度符号/绝对值需按实际场地标定, 微小角度(|a|<1)按"航向校正占位"处理。
 */
static void Turn_Angle_Compat(float angle)
{
    if (angle > -1.0f && angle < 1.0f) {
        /* 分段后的航向校正: 旋转到绝对 0°(车头方向), 用 JY61P yaw 闭环 */
        Chassis_Rotate_To(0.0f);
    } else {
        Chassis_Rotate(angle);
    }
}

/* ================= 机械臂动作序列 ================= */

void Arm_Start_Bomb_Grab(void)
{
#if MISSION_TEST_NO_ARM
    MLOG("Arm: Bomb Grab (disabled)");
#else
    MLOG("Arm: Bomb Grab");
    Servos_SetPositions((uint16_t *)ARM_GRAB_OPEN, 1500);
    HAL_Delay(move_time + hold_time);
    Servos_SetPositions((uint16_t *)ARM_GRAB_LOWER, 1500);
    HAL_Delay(move_time + hold_time);
    Servos_SetPositions((uint16_t *)ARM_GRAB_CLOSE, 1500);
    HAL_Delay(move_time + hold_time);
    Servos_SetPositions((uint16_t *)ARM_GRAB_LIFT, 1500);
    HAL_Delay(move_time + hold_time);
#endif
}

void Arm_Start_Bomb_Place(void)
{
#if MISSION_TEST_NO_ARM
    MLOG("Arm: Bomb Place (disabled)");
#else
    MLOG("Arm: Bomb Place");
    Servos_SetPositions((uint16_t *)ARM_PLACE_TURN, move_time);
    HAL_Delay(move_time + hold_time);
    Servos_SetPositions((uint16_t *)ARM_PLACE_LOWER, move_time);
    HAL_Delay(move_time + hold_time);
    Servos_SetPositions((uint16_t *)ARM_PLACE_OPEN, move_time);
    HAL_Delay(move_time + hold_time);
    Servos_SetPositions((uint16_t *)ARM_PLACE_LIFT, move_time);
    HAL_Delay(move_time + hold_time);
#endif
}

void Arm_Start_Rescue_Grab(void)   { MLOG("Arm: Rescue Grab (TODO)"); }
void Arm_Start_Rescue_Retract(void){ MLOG("Arm: Rescue Retract (TODO)"); }

/**
 * @brief  任务初始化: 恢复到待机状态
 */
void Mission_Init(void)
{
    g_mission_state = MISSION_STATE_IDLE;
    g_vision_task_in_progress = 0;
    g_qr_code_string[0] = '\0';
    Chassis_Stop();
    Laser_Off();
    LED_OFF();
#if MISSION_TEST_NO_ARM
    MLOG("Arm disabled (test): skip home");
#else
    Servos_SetPositions((uint16_t *)ARM_HOME, 1500);
    HAL_Delay(1500);
#endif
}

/**
 * @brief  启动任务
 */
void Mission_Start(void)
{
    if (g_mission_state == MISSION_STATE_IDLE || g_mission_state == MISSION_STATE_COMPLETE) {
        MLOG("--- MISSION START ---");
        g_mission_state = STATE_1_MOVING_TO_QR_SCAN;
    }
}

/**
 * @brief  像素误差 -> 修正距离
 */
static int32_t Calculate_Move_Distance(int pixel_error)
{
    if (abs(pixel_error) < ALIGN_TOLERANCE) return 0;
    int32_t dist = (int32_t)(abs(pixel_error) * K_GAIN);
    if (dist < MIN_MOVE_MM) dist = MIN_MOVE_MM;
    if (dist > MAX_MOVE_MM) dist = MAX_MOVE_MM;
    return dist;
}

/**
 * @brief  视觉辅助任务(任务1 排爆 / 任务2 打靶 共用)
 */
static void Handle_Vision_Alignment(uint8_t expected_task_number)
{
    typedef enum {
        BOMB_IDLE,
        BOMB_WAIT_INITIAL_MOVE,
        BOMB_REQUEST_FINE_TUNE,
        BOMB_WAIT_FINE_ALIGN,
        BOMB_SETTLE,
        BOMB_PERFORM_GRAB,
        BOMB_ADJUST_FOR_PLACE,
        BOMB_PERFORM_PLACE,
        BOMB_COMPLETE
    } BombSubState_t;

    typedef enum {
        TARGET_IDLE,
        TARGET_WAIT_ADJUST,
        TARGET_FIRING,
        TARGET_COMPLETE
    } TargetSubState_t;

    /* 任务3: 救援(复用"停下给视觉→对准→执行"的结构, K230 任务号=3) */
    typedef enum {
        RESCUE_IDLE,
        RESCUE_WAIT_ADJUST,     /* 视觉给 L/R: 先横移占位 */
        RESCUE_WAIT_ALIGN,      /* 视觉精对准(D:/OK 脉冲) */
        RESCUE_SETTLE,          /* 对准后停车稳定 */
        RESCUE_PERFORM,         /* 执行营救动作 */
        RESCUE_COMPLETE
    } RescueSubState_t;

    static BombSubState_t bomb_sub_state = BOMB_IDLE;
    static TargetSubState_t target_sub_state = TARGET_IDLE;
    static RescueSubState_t rescue_sub_state = RESCUE_IDLE;
    static char bomb_path_taken = ' ';
#if !MISSION_TEST_NO_VISION
    static char target_path_taken = ' ';
    static char rescue_path_taken = ' ';
#endif
    static uint32_t state_start_tick = 0;
    static uint32_t align_start_time = 0;
    static uint32_t laser_start_time = 0;
    static uint32_t cooldown_until = 0;   /* 脉冲驱动冷却截止时刻(ms) */
    static uint32_t settle_until = 0;     /* 停车稳定等待截止时刻(ms) */

    char line[K230_LINE_MAX];

    /* ================= 任务1: 排爆 ================= */
    if (expected_task_number == 1) {
        switch (bomb_sub_state) {
            case BOMB_IDLE:
#if MISSION_TEST_NO_VISION
                /* 测试: 无 K230, 模拟路径 C(直接靠近) */
                bomb_path_taken = 'C';
                MLOG("Task1 Dir(sim): C");
                Chassis_Move_Forward((int32_t)BOMB_C1_APPROACH_DISTANCE);
                state_start_tick = HAL_GetTick();
                bomb_sub_state = BOMB_WAIT_INITIAL_MOVE;
#else
                if (!g_vision_task_in_progress) {
                    Chassis_Stop();
                    MLOG("TASK1: Vision Start");
                    K230_Run_Specific_Task(1);
                    g_vision_task_in_progress = 1;
                } else if (Mission_GetNewLine(line, sizeof(line))) {
                    bomb_path_taken = line[0];
                    MLOG("Task1 Dir: %c", bomb_path_taken);
                    if (bomb_path_taken == 'C') {
                        Chassis_Move_Forward((int32_t)BOMB_C1_APPROACH_DISTANCE);
                    } else if (bomb_path_taken == 'L') {
                        Chassis_Move_Backward(BOMB_L_INIT_BACK_MM);
                    } else if (bomb_path_taken == 'R') {
                        Chassis_Move_Forward((int32_t)BOMB_C1_APPROACH_DISTANCE + BOMB_R_INIT_FWD_MM);
                    }
                    state_start_tick = HAL_GetTick();
                    bomb_sub_state = BOMB_WAIT_INITIAL_MOVE;
                }
#endif
                break;

            case BOMB_WAIT_INITIAL_MOVE:
                if (Chassis_Task_Is_Complete() || (HAL_GetTick() - state_start_tick > BLIND_MOVE_TIMEOUT_MS)) {
                    bomb_sub_state = BOMB_REQUEST_FINE_TUNE;
                }
                break;

            case BOMB_REQUEST_FINE_TUNE:
#if MISSION_TEST_NO_VISION
                /* 测试: 跳过精对准, 直接进入稳定等待 */
                MLOG("Fine Align(sim): OK");
                Chassis_Stop();
                settle_until = HAL_GetTick() + FINE_TUNE_SETTLE_MS;
                bomb_sub_state = BOMB_SETTLE;
#else
                MLOG("Request Fine Align");
                K230_Start_Align();
                Serial_FlushRx();
                state_start_tick = HAL_GetTick();
                align_start_time = HAL_GetTick();
                cooldown_until = 0;
                Chassis_Stop();
                bomb_sub_state = BOMB_WAIT_FINE_ALIGN;
#endif
                break;

            case BOMB_WAIT_FINE_ALIGN:
                if (HAL_GetTick() - state_start_tick > ALIGN_TIMEOUT_MS) {
                    MLOG("Timeout! Force Grab");
                    Chassis_Stop();
                    bomb_sub_state = BOMB_PERFORM_GRAB;
                    break;
                }
                if (HAL_GetTick() - align_start_time > FORCE_GRAB_AFTER_MS && g_k230_new_data_flag) {
                    MLOG("15s Limit! Force Grab");
                    Chassis_Stop();
                    bomb_sub_state = BOMB_PERFORM_GRAB;
                    break;
                }
                if (Mission_GetNewLine(line, sizeof(line))) {
                    /* 脉冲驱动: 每次修正后冷却 0.8s, 期间忽略新帧(不叠指令) */
                    if (HAL_GetTick() < cooldown_until) {
                        break;
                    }
                    if (strncmp(line, "OK", 2) == 0) {
                        MLOG("Vision OK -> Settle 0.5s");
                        Chassis_Stop();
                        settle_until = HAL_GetTick() + FINE_TUNE_SETTLE_MS;   /* 停车惯性晃动, 等稳再抓 */
                        bomb_sub_state = BOMB_SETTLE;
                    } else {
                        char *pD = strstr(line, "D:");
                        if (pD != NULL) {
                            int err_x = 0, err_y = 0;
                            char *pComma = strchr(pD, ',');
                            if (pComma) {
                                *pComma = '\0';
                                err_x = atoi(pD + 2);
                                err_y = atoi(pComma + 1);
                                MLOG("Err: %d, %d", err_x, err_y);

                                if (abs(err_x) < ALIGN_TOLERANCE && abs(err_y) < ALIGN_TOLERANCE) {
                                    MLOG("Close Enough -> Settle 0.5s");
                                    Chassis_Stop();
                                    settle_until = HAL_GetTick() + FINE_TUNE_SETTLE_MS;
                                    bomb_sub_state = BOMB_SETTLE;
                                    break;
                                }

                                /* 串行修正: 先 X 后 Y, 绝不同时修 */
                                if (abs(err_x) >= ALIGN_TOLERANCE) {
                                    int32_t d = Calculate_Move_Distance(err_x);
                                    if (d > 0) {
                                        if (err_x < 0) Chassis_Move_Right(d);
                                        else           Chassis_Move_Left(d);
                                        cooldown_until = HAL_GetTick() + FINE_TUNE_COOLDOWN_MS;
                                    }
                                } else if (abs(err_y) >= ALIGN_TOLERANCE) {
                                    int32_t d = Calculate_Move_Distance(err_y);
                                    if (d > 0) {
                                        if (err_y < 0) Chassis_Move_Backward(d);
                                        else           Chassis_Move_Forward(d);
                                        cooldown_until = HAL_GetTick() + FINE_TUNE_COOLDOWN_MS;
                                    }
                                }
                            }
                        }
                    }
                }
                break;

            case BOMB_SETTLE:
                if (HAL_GetTick() >= settle_until) {
                    bomb_sub_state = BOMB_PERFORM_GRAB;
                }
                break;

            case BOMB_PERFORM_GRAB:
                MLOG("State: GRAB");
                Arm_Start_Bomb_Grab();
                if (bomb_path_taken == 'L') {
                    Chassis_Move_Forward(BOMB_L_RETURN_FWD_MM);
                    bomb_sub_state = BOMB_ADJUST_FOR_PLACE;
                } else if (bomb_path_taken == 'R') {
                    Chassis_Move_Backward(BOMB_R_RETURN_BACK_MM);
                    bomb_sub_state = BOMB_ADJUST_FOR_PLACE;
                } else {
                    bomb_sub_state = BOMB_PERFORM_PLACE;
                }
                break;

            case BOMB_ADJUST_FOR_PLACE:
                if (Chassis_Task_Is_Complete()) bomb_sub_state = BOMB_PERFORM_PLACE;
                break;

            case BOMB_PERFORM_PLACE:
                MLOG("State: PLACE");
                Arm_Start_Bomb_Place();
                bomb_sub_state = BOMB_COMPLETE;
                break;

            case BOMB_COMPLETE:
                MLOG("Task 1 Done");
                bomb_sub_state = BOMB_IDLE;
                g_mission_state++;
                break;

            default: break;
        }
    }
    /* ================= 任务2: 打靶 ================= */
    else if (expected_task_number == 2) {
        switch (target_sub_state) {
            case TARGET_IDLE:
#if MISSION_TEST_NO_VISION
                /* 测试: 模拟靶标在正中间(C) */
                MLOG("Task2 Dir(sim): C");
                target_sub_state = TARGET_FIRING;
                Laser_On();
                laser_start_time = HAL_GetTick();
#else
                if (!g_vision_task_in_progress) {
                    Chassis_Stop();
                    MLOG("TASK2: Vision Start");
                    K230_Run_Specific_Task(2);
                    g_vision_task_in_progress = 1;
                } else if (Mission_GetNewLine(line, sizeof(line))) {
                    g_vision_task_in_progress = 0;
                    target_path_taken = line[0];
                    MLOG("Task2 Dir: %c", target_path_taken);

                    if (target_path_taken == 'C') {
                        target_sub_state = TARGET_FIRING;
                        Laser_On();
                        laser_start_time = HAL_GetTick();
                    } else if (target_path_taken == 'L') {
                        Chassis_Move_Left((int32_t)TARGET_ADJUST_DISTANCE);
                        target_sub_state = TARGET_WAIT_ADJUST;
                    } else if (target_path_taken == 'R') {
                        Chassis_Move_Right((int32_t)TARGET_ADJUST_DISTANCE);
                        target_sub_state = TARGET_WAIT_ADJUST;
                    }
                }
#endif
                break;

            case TARGET_WAIT_ADJUST:
                if (Chassis_Task_Is_Complete()) {
                    target_sub_state = TARGET_FIRING;
                    Laser_On();
                    laser_start_time = HAL_GetTick();
                }
                break;

            case TARGET_FIRING:
                if (HAL_GetTick() - laser_start_time > LASER_FIRE_DURATION_MS) {
                    Laser_Off();
                    target_sub_state = TARGET_COMPLETE;
                }
                break;

            case TARGET_COMPLETE:
                MLOG("Task 2 Done");
                target_sub_state = TARGET_IDLE;
                g_mission_state++;
                break;

            default: break;
        }
    }
    /* ================= 任务3: 救援 (视觉对准; 复用排爆同款流程, K230 任务号=3) ================= */
    else if (expected_task_number == 3) {
        switch (rescue_sub_state) {
            case RESCUE_IDLE:
#if MISSION_TEST_NO_VISION
                /* 测试: 无 K230, 模拟"已对准"→停稳→执行营救 */
                MLOG("Task3 Dir(sim): C");
                Chassis_Stop();
                settle_until = HAL_GetTick() + FINE_TUNE_SETTLE_MS;
                rescue_sub_state = RESCUE_SETTLE;
#else
                if (!g_vision_task_in_progress) {
                    Chassis_Stop();
                    MLOG("TASK3(Rescue): Vision Start");
                    K230_Run_Specific_Task(3);
                    g_vision_task_in_progress = 1;
                } else if (Mission_GetNewLine(line, sizeof(line))) {
                    g_vision_task_in_progress = 0;
                    rescue_path_taken = line[0];
                    MLOG("Task3 Dir: %c", rescue_path_taken);
                    if (rescue_path_taken == 'C') {
                        /* 正对: 直接进入精对准 */
                        K230_Start_Align();
                        Serial_FlushRx();
                        state_start_tick = HAL_GetTick();
                        align_start_time = HAL_GetTick();
                        cooldown_until = 0;
                        Chassis_Stop();
                        rescue_sub_state = RESCUE_WAIT_ALIGN;
                    } else if (rescue_path_taken == 'L') {
                        Chassis_Move_Left((int32_t)TARGET_ADJUST_DISTANCE);
                        rescue_sub_state = RESCUE_WAIT_ADJUST;
                    } else if (rescue_path_taken == 'R') {
                        Chassis_Move_Right((int32_t)TARGET_ADJUST_DISTANCE);
                        rescue_sub_state = RESCUE_WAIT_ADJUST;
                    }
                }
#endif
                break;

            case RESCUE_WAIT_ADJUST:      /* 仅联调模式: 先横移再开始精对准 */
                if (Chassis_Task_Is_Complete()) {
                    K230_Start_Align();
                    Serial_FlushRx();
                    state_start_tick = HAL_GetTick();
                    align_start_time = HAL_GetTick();
                    cooldown_until = 0;
                    Chassis_Stop();
                    rescue_sub_state = RESCUE_WAIT_ALIGN;
                }
                break;

            case RESCUE_WAIT_ALIGN:
                if (HAL_GetTick() - state_start_tick > ALIGN_TIMEOUT_MS) {
                    MLOG("Rescue Align Timeout!");
                    Chassis_Stop();
                    settle_until = HAL_GetTick();
                    rescue_sub_state = RESCUE_SETTLE;
                    break;
                }
#if MISSION_TEST_NO_VISION
                /* 测试模式不会走到这里(已在 RESCUE_IDLE 模拟完成) */
                rescue_sub_state = RESCUE_PERFORM;
#else
                if (Mission_GetNewLine(line, sizeof(line))) {
                    if (HAL_GetTick() < cooldown_until) break;   /* 冷却中, 忽略新帧 */
                    if (strncmp(line, "OK", 2) == 0) {
                        MLOG("Rescue Align OK -> Settle");
                        Chassis_Stop();
                        settle_until = HAL_GetTick() + FINE_TUNE_SETTLE_MS;
                        rescue_sub_state = RESCUE_SETTLE;
                    } else {
                        char *pD = strstr(line, "D:");
                        if (pD != NULL) {
                            char *pComma = strchr(pD, ',');
                            if (pComma) {
                                *pComma = '\0';
                                int err_x = atoi(pD + 2);
                                int err_y = atoi(pComma + 1);
                                if (abs(err_x) < ALIGN_TOLERANCE && abs(err_y) < ALIGN_TOLERANCE) {
                                    MLOG("Rescue Close Enough");
                                    Chassis_Stop();
                                    settle_until = HAL_GetTick() + FINE_TUNE_SETTLE_MS;
                                    rescue_sub_state = RESCUE_SETTLE;
                                    break;
                                }
                                /* 串行修正: 先 X 后 Y */
                                if (abs(err_x) >= ALIGN_TOLERANCE) {
                                    int32_t d = Calculate_Move_Distance(err_x);
                                    if (d > 0) {
                                        if (err_x < 0) Chassis_Move_Right(d);
                                        else           Chassis_Move_Left(d);
                                        cooldown_until = HAL_GetTick() + FINE_TUNE_COOLDOWN_MS;
                                    }
                                } else if (abs(err_y) >= ALIGN_TOLERANCE) {
                                    int32_t d = Calculate_Move_Distance(err_y);
                                    if (d > 0) {
                                        if (err_y < 0) Chassis_Move_Backward(d);
                                        else           Chassis_Move_Forward(d);
                                        cooldown_until = HAL_GetTick() + FINE_TUNE_COOLDOWN_MS;
                                    }
                                }
                            }
                        }
                    }
                }
#endif
                break;

            case RESCUE_SETTLE:
                if (HAL_GetTick() >= settle_until) {
                    rescue_sub_state = RESCUE_PERFORM;
                }
                break;

            case RESCUE_PERFORM:
                MLOG("State: RESCUE PERFORM");
                Arm_Start_Rescue_Grab();
                Arm_Start_Rescue_Retract();
                rescue_sub_state = RESCUE_COMPLETE;
                break;

            case RESCUE_COMPLETE:
                MLOG("Task 3 (Rescue) Done");
                rescue_sub_state = RESCUE_IDLE;
                g_mission_state++;
                break;

            default: break;
        }
    }
}

/**
 * @brief  主状态机更新(由主循环周期性调用)
 *
 * 本函数是整场比赛流程的调度核心, 采用经典的两段式有限状态机(FSM)写法:
 *
 *   【第一段】状态进入动作 (Entry Action)
 *     只有当 g_mission_state 发生变化(与 last_state 不同)时才执行一次。
 *     作用: 在"刚切入新状态"的瞬间, 发起一次性指令(底盘运动 / 停止 / 校正),
 *     之后每个主循环周期不再重复执行, 避免重复下发运动指令。
 *
 *   【第二段】状态转移检查 (Transition Check)
 *     每个主循环周期都会执行, 根据当前状态判断是否满足切换条件:
 *       - 移动类状态: Chassis_Task_Is_Complete() 为真 -> g_mission_state++ (顺序推进)
 *       - 扫码状态(STATE_2): 拿到二维码(或测试模拟) -> 推进
 *       - 视觉任务状态(STATE_11/STATE_13): 交给 Handle_Vision_Alignment 的内部子状态机,
 *         由其内部完成后 g_mission_state++
 *
 *   【调用位置】
 *     main.c 主循环: if(demoRun){...} else { Mission_Update(); }
 *     即非演示模式下, 每个 while(1) 循环调用一次。
 */
void Mission_Update(void)
{
    /* 上一次的状态号; 初始置为 -1(无效值), 保证首次调用必然执行一次"进入动作" */
    static MissionState_t last_state = (MissionState_t)-1;
    /* 测试模式下扫码状态的停车计时起点 */
    static uint32_t qr_scan_start = 0;

    /* ============================================================
     * 第一段: 状态进入动作 (仅在状态切换瞬间执行一次)
     * ============================================================ */
    if (g_mission_state != last_state) {
        MLOG("State: %d -> %d", (int)last_state, (int)g_mission_state);
        last_state = g_mission_state;   /* 记录新状态, 防止同一状态重复触发 */

        switch (g_mission_state) {
            /* ---------- 阶段一: 扫码并前往排爆区 ---------- */
            case STATE_1_MOVING_TO_QR_SCAN:   Chassis_Move_Forward(ROUTE_1_TO_QR_MM); break;  /* 起点→扫码点(直行) */
            case STATE_2_PERFORMING_QR_SCAN:  Chassis_Stop(); break;                          /* 停下准备扫码 */

            case STATE_3_MOVE_LEFT_A:         Chassis_Move_Left(ROUTE_3_LEFT_A_MM); break;    /* 扫码后左移 A 段 */
            case STATE_3A_HEADING_CORRECT:    Turn_Angle_Compat(0.1f); break;                 /* 左移后航向校正到 0° */
            case STATE_4_MOVE_TOPLEFT:        Chassis_Move_Diagonal(ROUTE_4_DIAG_FWD_MM, ROUTE_4_DIAG_LEFT_MM); break;  /* 左上斜跑(前/左分量见宏) */
            case STATE_4A_HEADING_CORRECT:    Turn_Angle_Compat(0.1f); break;                 /* 斜跑后航向校正到 0° */
            case STATE_5_STOP_BEFORE_RAMP:    Chassis_Move_Forward(ROUTE_5_TO_RAMP_MM); break;/* 斜坡前直行 */
            case STATE_6_CROSSING_RAMP_A:     Turn_Angle_Compat(0.1f); break;                 /* 过斜坡段: 航向校正 */

            /* ---------- STATE_7 分段: 左移B -> 中间停 -> 左移C -> 航向校正 ---------- */
            case STATE_7_MOVE_LEFT_B:         Chassis_Move_Left(ROUTE_7_LEFT_B_MM); break;    /* 左移阶段B */
            case STATE_7A_INTERMEDIATE_STOP:  Turn_Angle_Compat(0.1f); break;                 /* 中间停顿 + 航向校正 */
            case STATE_7B_MOVE_LEFT_C:        Chassis_Move_Left(ROUTE_7_LEFT_C_MM); break;    /* 左移阶段C */
            case STATE_7A_HEADING_CORRECTION: Turn_Angle_Compat(0.1f); break;                 /* 航向校正 */

            /* ---------- 排爆区走位 ---------- */
            case STATE_8_MOVE_FORWARD_A:      Chassis_Move_Forward(ROUTE_8_TO_BOMB_AREA_MM); break;  /* 直线前进进入排爆区 */
            case STATE_8A_HEADING_CORRECT:    Turn_Angle_Compat(0.1f); break;                 /* 前进后航向校正 */
            case STATE_8B_ADJUST_RIGHT:       Chassis_Move_Right(ROUTE_8B_RIGHT_MM); break;    /* 右移微调 */
            case STATE_8C_HEADING_CORRECT:    Turn_Angle_Compat(0.1f); break;                 /* 右移后航向校正 */
            case STATE_9_TURN_FOR_BOMB:       Chassis_Rotate(ROUTE_9_TURN_DEG); break;        /* 转向排爆点(角度见宏) */
            case STATE_9A_ADJUST_LEFT:        Chassis_Move_Left(ROUTE_9A_LEFT_MM); break;     /* 左移微调 */
            case STATE_10_APPROACH_BOMB:      Chassis_Move_Forward(ROUTE_10_APPROACH_MM); break; /* 接近炸弹 */
            case STATE_11_PERFORMING_BOMB_DISPOSAL: Chassis_Stop(); break;                    /* 停下, 交给视觉子状态机 */

            /* ---------- 阶段二: 打靶 (分段 + 陀螺仪校正) ---------- */
            case STATE_12_PART1_MOVE_A:       Chassis_Move_Right(ROUTE_12_P1_A_MM); break;  /* Part1 第1段 */
            case STATE_12_PART1_CORRECT_A:    Turn_Angle_Compat(0.1f); break;                 /* 陀螺仪航向校正 */
            case STATE_12_PART1_MOVE_B:       Chassis_Move_Right(ROUTE_12_P1_B_MM); break;  /* Part1 第2段 */
            case STATE_12_PART1_CORRECT_B:    Turn_Angle_Compat(0.1f); break;                 /* 陀螺仪航向校正 */
            case STATE_12_PART1_MOVE_C:       Chassis_Move_Right(ROUTE_12_P1_C_MM); break;  /* Part1 第3段 */

            /* 大角度转弯拆成两步, 减小单次转向的超调 */
            case STATE_12_TURN_A:             Chassis_Move_Right(ROUTE_12_TURN_A_DEG); break;  
            case STATE_12_TURN_B:             Turn_Angle_Compat(0.1f); break;    

            /* Part2: 段间各做一次航向校正, 各段距离见 ROUTE_12_P2_*_MM 宏 */
            case STATE_12_PART2_MOVE_A:       Chassis_Move_Right(ROUTE_12_P2_A_MM); break;  /* Part2 第1段 */
            case STATE_12_PART2_CORRECT_A:    Turn_Angle_Compat(0.1f); break;                 /* 航向校正 */
            case STATE_12_PART2_MOVE_B:       Chassis_Move_Right(ROUTE_12_P2_B_MM); break;  /* Part2 第2段 */
            case STATE_12_PART2_CORRECT_B:    Turn_Angle_Compat(0.1f); break;                 /* 航向校正 */
            case STATE_12_PART2_MOVE_C:       Chassis_Move_Right(ROUTE_12_P2_C_MM); break;  /* Part2 第3段 */

            case STATE_13_PERFORMING_TARGETING: Chassis_Stop(); break;                        /* 停下, 交给视觉子状态机打靶 */

            /* ---------- 阶段三: 救援(掉头后接新阶段四走位) ---------- */
            case STATE_14_MOVE_FORWARD_B:     Chassis_Move_Forward(ROUTE_14_TO_HOSTAGE_MM); break;  /* 救援前前进 */
            case STATE_15_TURN_FOR_HOSTAGE:   Chassis_Rotate(ROUTE_15_TURN_180_DEG); break;   /* 原地掉头 180° */

            /* ---------- 阶段四(新增): 营救接近: 右移+航向校正 x4 -> 视觉营救 -> 右移收尾 ---------- */
            case STATE_16_RESCUE_RIGHT_A:         Chassis_Move_Right(ROUTE_16_RIGHT_A_MM); break;   /* 右移A */
            case STATE_16A_RESCUE_HEADING_CORRECT: Turn_Angle_Compat(0.1f); break;                  /* 航向校正 */
            case STATE_17_RESCUE_RIGHT_B:         Chassis_Move_Right(ROUTE_17_RIGHT_B_MM); break;   /* 右移B */
            case STATE_17A_RESCUE_HEADING_CORRECT: Turn_Angle_Compat(0.1f); break;                  /* 航向校正 */
            case STATE_18_RESCUE_RIGHT_C:         Chassis_Move_Right(ROUTE_18_RIGHT_C_MM); break;   /* 右移C */
            case STATE_18A_RESCUE_HEADING_CORRECT: Turn_Angle_Compat(0.1f); break;                  /* 航向校正 */
            case STATE_19_RESCUE_RIGHT_D:         Chassis_Move_Right(ROUTE_19_RIGHT_D_MM); break;   /* 右移D */
            case STATE_19A_RESCUE_HEADING_CORRECT: Turn_Angle_Compat(0.1f); break;                  /* 航向校正 */
            case STATE_20_PERFORMING_HOSTAGE_RESCUE: Chassis_Stop(); break;                         /* 停下, 交给视觉(3=救援) */
            case STATE_21_RESCUE_RIGHT_E:         Chassis_Move_Right(ROUTE_21_RIGHT_E_MM); break;   /* 右移E */
            case STATE_21A_RESCUE_HEADING_CORRECT: Turn_Angle_Compat(0.1f); break;                  /* 航向校正 */
            case STATE_22_RESCUE_RIGHT_F:         Chassis_Move_Right(ROUTE_22_RIGHT_F_MM); break;   /* 右移F(收尾) */

            default: break;   /* IDLE / COMPLETE 等状态无进入动作 */
        }
    }

    /* ============================================================
     * 第二段: 状态转移检查 (每个主循环周期执行)
     * 大多数移动状态: 底盘到位(Chassis_Task_Is_Complete)即 g_mission_state++
     * 视觉状态: 交给 Handle_Vision_Alignment 的内部子状态机处理
     * ============================================================ */
    switch (g_mission_state) {
        /* 前进 600mm 完成后 -> STATE_2 扫码 */
        case STATE_1_MOVING_TO_QR_SCAN:
            if (Chassis_Task_Is_Complete()) g_mission_state++;
            break;

        /* 扫码状态:
         *   - 测试模式(NO_VISION): 停车 1s 后模拟二维码 "123" 并推进
         *   - 联调模式: 向 K230 发 scan_qr, 收到回传后解析并推进 */
        case STATE_2_PERFORMING_QR_SCAN:
#if MISSION_TEST_NO_VISION
            /* 测试: 停车 1s 再模拟扫码推进, 否则进入状态后立即跳到 STATE_3, 看不到停车 */
            if (!g_vision_task_in_progress) {
                qr_scan_start = HAL_GetTick();
                g_vision_task_in_progress = 1;
            } 
            else if (HAL_GetTick() - qr_scan_start >= QR_SIM_WAIT_MS) {
                strcpy(g_qr_code_string, "123");
                MLOG("QR(sim): %s", g_qr_code_string);
                g_vision_task_in_progress = 0;
                g_mission_state++;
            }
#else
            if (!g_vision_task_in_progress) {
                K230_Request_QRScan();   /* 请求 K230 扫码 */
                Serial_FlushRx();        /* 清空旧数据, 避免误读 */
                g_vision_task_in_progress = 1;
            } else {
                char line[K230_LINE_MAX];
                if (Mission_GetNewLine(line, sizeof(line))) {
                    g_vision_task_in_progress = 0;
                    const char *p = line;
                    if (strncmp(p, "qr:", 3) == 0) p += 3;   /* 兼容 "qr:xxx" 前缀 */
                    strncpy(g_qr_code_string, p, sizeof(g_qr_code_string) - 1);
                    g_qr_code_string[sizeof(g_qr_code_string) - 1] = '\0';
                    MLOG("QR: %s", g_qr_code_string);
                    K230_Send_QRCode_Data(g_qr_code_string);   /* 回传二维码给 K230 */
                    g_mission_state++;
                }
            }
#endif
            break;

        /* 以下移动状态: 底盘到位即顺序推进到下一状态 */
        case STATE_3_MOVE_LEFT_A:         if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        case STATE_3A_HEADING_CORRECT:    if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        case STATE_4_MOVE_TOPLEFT:        if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        case STATE_4A_HEADING_CORRECT:    if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        case STATE_5_STOP_BEFORE_RAMP:    if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        case STATE_6_CROSSING_RAMP_A:     if (Chassis_Task_Is_Complete()) g_mission_state++; break;

        case STATE_7_MOVE_LEFT_B:         if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        case STATE_7A_INTERMEDIATE_STOP:  if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        /* 左移C完成后不是 ++, 而是跳回航向校正状态 */
        case STATE_7B_MOVE_LEFT_C:        if (Chassis_Task_Is_Complete()) g_mission_state = STATE_7A_HEADING_CORRECTION; break;
        case STATE_7A_HEADING_CORRECTION: if (Chassis_Task_Is_Complete()) g_mission_state++; break;

        case STATE_8_MOVE_FORWARD_A:      if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        case STATE_8A_HEADING_CORRECT:    if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        case STATE_8B_ADJUST_RIGHT:       if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        case STATE_8C_HEADING_CORRECT:    if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        case STATE_9_TURN_FOR_BOMB:       if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        case STATE_9A_ADJUST_LEFT:        if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        case STATE_10_APPROACH_BOMB:      if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        /* 排爆: 进入视觉辅助子状态机(内部处理完会自己 g_mission_state++) */
        case STATE_11_PERFORMING_BOMB_DISPOSAL: Handle_Vision_Alignment(1); break;

        case STATE_12_PART1_MOVE_A:       if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        case STATE_12_PART1_CORRECT_A:    if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        case STATE_12_PART1_MOVE_B:       if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        case STATE_12_PART1_CORRECT_B:    if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        case STATE_12_PART1_MOVE_C:       if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        case STATE_12_TURN_A:             if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        case STATE_12_TURN_B:             if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        case STATE_12_PART2_MOVE_A:       if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        case STATE_12_PART2_CORRECT_A:    if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        case STATE_12_PART2_MOVE_B:       if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        case STATE_12_PART2_CORRECT_B:    if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        /* Part2 最后一段走完后, 跳入打靶视觉状态 */
        case STATE_12_PART2_MOVE_C:       if (Chassis_Task_Is_Complete()) g_mission_state = STATE_13_PERFORMING_TARGETING; break;
        /* 打靶: 进入视觉辅助子状态机(内部处理完会自己 g_mission_state++) */
        case STATE_13_PERFORMING_TARGETING: Handle_Vision_Alignment(2); break;

        case STATE_14_MOVE_FORWARD_B:     if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        /* 掉头完成后 -> 进入救援接近(新阶段四) */
        case STATE_15_TURN_FOR_HOSTAGE:
            if (Chassis_Task_Is_Complete()) {
                g_mission_state = STATE_16_RESCUE_RIGHT_A;
            }
            break;

        /* ---------- 阶段四(救援): 右移/航向校正 顺序推进 ---------- */
        case STATE_16_RESCUE_RIGHT_A:         if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        case STATE_16A_RESCUE_HEADING_CORRECT: if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        case STATE_17_RESCUE_RIGHT_B:         if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        case STATE_17A_RESCUE_HEADING_CORRECT: if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        case STATE_18_RESCUE_RIGHT_C:         if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        case STATE_18A_RESCUE_HEADING_CORRECT: if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        case STATE_19_RESCUE_RIGHT_D:         if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        case STATE_19A_RESCUE_HEADING_CORRECT: if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        /* 视觉营救: 交给视觉子状态机(任务3, 内部完成会自己 g_mission_state++) */
        case STATE_20_PERFORMING_HOSTAGE_RESCUE: Handle_Vision_Alignment(3); break;
        case STATE_21_RESCUE_RIGHT_E:         if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        case STATE_21A_RESCUE_HEADING_CORRECT: if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        /* 最后一段右移完成后 -> 结束整个任务 */
        case STATE_22_RESCUE_RIGHT_F:
            if (Chassis_Task_Is_Complete()) {
                MLOG("All Done.");
                Chassis_Stop();
                g_mission_state = MISSION_STATE_COMPLETE;
            }
            break;

        default: break;   /* IDLE / COMPLETE 等状态无转移条件 */
    }
}
