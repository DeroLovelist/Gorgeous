/**
 * @file    MissionControl.h
 * @brief   比赛任务主状态机(适配本 STM32F407 工程: K230 视觉 + 飞特舵机机械臂 + 麦轮底盘)
 *
 * 与参考工程 mission_control 的对应关系:
 *   - 树莓派视觉  -> K230(经 UART4/Serial 通信, 见 Serial.h 的 g_k230_rx_line)
 *   - GM65 扫码   -> K230 扫码(经同一串口回传二维码数据)
 *   - 总线舵机    -> 飞特舵机(见 ServoArm.h)
 *   - Motor_Control -> Chassis.h(麦轮底盘)
 */

#ifndef __MISSIONCONTROL_H
#define __MISSIONCONTROL_H

#include <stdint.h>
#include <stdbool.h>

/* =====================================================================
 * ⭐ 测试开关 (调参时最常改的两个开关)
 *   取值只可为 0 或 1。
 *   1 = 跳过该部分(用模拟数据/不驱动硬件); 0 = 真实启用该部分。
 *   联调/比赛前记得把要启用的部分设回 0。
 * ---------------------------------------------------------------------
 * MISSION_TEST_NO_ARM     =1: 不初始化/不驱动机械臂舵机(防误动伤到人/舵机堵转)
 *                           =0: 正常驱动飞特舵机机械臂(需先标定 ARM_* 位置)
 * MISSION_TEST_NO_VISION  =1: K230 视觉不参与, 状态机用模拟数据自行推进(方便纯调底盘/路线)
 *                           =0: 等待 K230 经 UART4 回传真实二维码/路径/对准数据
 * ---------------------------------------------------------------------
 * 当前: 两者都为 1 = “纯底盘测试模式”: 只跑路线距离, 不抓不看不扫码。
 * 调好路线距离后: 先把 MISSION_TEST_NO_VISION 置 0 联调视觉, 最后再把
 * MISSION_TEST_NO_ARM 置 0 联调机械臂。
 * ===================================================================== */
#define MISSION_TEST_NO_ARM      0   /* 1=不初始化/不驱动机械臂 */
#define MISSION_TEST_NO_VISION   0   /* 1=K230 不参与, 用模拟数据推进状态机 */

/* ⭐ 视觉单独调试开关: 0=正常整场任务(联合调试);
 *   1=球(抓取前对准) 2=靶(打靶) 3=桶(放置前对准) 4=形状(救援)。
 *   非 0 时: 上电按 KEY1 直接进入对应视觉目标对准(不跑路线、不动机械臂),
 *   对准完成后停车并打印结果。改完记得重载+构建。
 *   5 = 视觉串口链路监控(纯收发打印, 车/臂完全不动):
 *     上电后 KEY1=发 scan_qr, KEY1 长按=发 run_task:1, KEY2=发 reset:0,
 *     收到 K230 任何一行打印 "RX: ..." */
#define MISSION_DEBUG_VISION_TASK  0

/* ⭐ 机械臂动作单独调试开关(底盘完全不动, 不跑视觉也不跑路线):
 *   0 = 关;
 *   1 = 排爆序列(回初始位 → 抓取 → 停顿 → 放置 → 回初始位)。
 *   非 0 时: 上电按 KEY1 跑一遍该序列(需 MISSION_TEST_NO_ARM=0)。
 *   用途: 标定完 ARM_* 位置后, 小车静止时验证机械臂动作是否正确/会不会撞。 */
#define MISSION_DEBUG_ARM_SEQ  0

/* ⭐ 编码器静态标定开关 (标定 CH_WHEEL_SCALE_*):
 *   0 = 关;
 *   1 = 上电进入标定模式(底盘闭环挂起, 电机不输出, 小车不会自己跑)。
 *   流程: ① KEY1 清零 → ② 沿车头方向用卷尺量 ENC_CALIB_DIST_MM 推过去
 *         → ③ KEY2 蓝牙打印 I/CALIB 建议系数 → ④ 反向再推一次对比
 *         → ⑤ KEY1 长按结束标定(恢复正常闭环)。
 *   详细说明/怎么判断该不该用静态标定 → 见 Chassis.h 的 Chassis_CalibStart 注释。 */
#define CHASSIS_ENC_CALIB        0
#define ENC_CALIB_DIST_MM        1000   /* 标定时人工推车的距离(mm), 自己用卷尺量准 */

#if MISSION_DEBUG_ARM_SEQ && MISSION_TEST_NO_ARM
#error "MISSION_DEBUG_ARM_SEQ 需要 MISSION_TEST_NO_ARM=0 (机械臂必须启用)"
#endif

#if MISSION_DEBUG_ARM_SEQ && CHASSIS_ENC_CALIB
#error "MISSION_DEBUG_ARM_SEQ 与 CHASSIS_ENC_CALIB 只能开一个"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------- 任务状态枚举 ---------------- */
typedef enum {
    MISSION_STATE_IDLE = 0,                 // 待机状态
    MISSION_STATE_COMPLETE,                 // 任务完成

    /* 阶段一: 扫码并前往排爆区 */
    STATE_1_MOVING_TO_QR_SCAN,              // 前往二维码扫描点
    STATE_2_PERFORMING_QR_SCAN,             // 扫描二维码
    STATE_3_MOVE_LEFT_A,                    // 左移并前往斜坡
    STATE_3A_HEADING_CORRECT,               // 左移后航向校正(消除左移甩尾, 保证斜跑角度准)
    STATE_4_MOVE_TOPLEFT,                   // 斜坡前微调
    STATE_4A_HEADING_CORRECT,               // 斜跑后航向校正(保证上坡直行角度准)
    STATE_5_STOP_BEFORE_RAMP,               // 停在斜坡前
    STATE_6_CROSSING_RAMP_A,                // 上斜坡

    /* STATE_7 分段 */
    STATE_7_MOVE_LEFT_B,                    // 左移阶段B
    STATE_7A_INTERMEDIATE_STOP,             // 中间停顿
    STATE_7B_MOVE_LEFT_C,                   // 左移阶段C
    STATE_7A_HEADING_CORRECTION,            // 航向校正

    STATE_8_MOVE_FORWARD_A,                 // 直线前进进入排爆区
    STATE_8A_HEADING_CORRECT,               // [新增] 前进980后航向校正, 保证右移前车头正
    STATE_8B_ADJUST_RIGHT,                  // 右移微调 (原 STATE_8A)
    STATE_8C_HEADING_CORRECT,               // [新增] 右移600后航向校正, 保证 -99° 相对转向从 0° 起算

    /* 阶段二: 排爆 */
    STATE_9_TURN_FOR_BOMB,                  // 转向排爆点
    STATE_9A_ADJUST_LEFT,                   // 左移微调
    STATE_10_APPROACH_BOMB,                 // 接近炸弹
    STATE_11_PERFORMING_BOMB_DISPOSAL,      // 视觉对准并抓取/放置炸弹

    /* 阶段三: 打靶 (2026-10-05 定稿)
     *   走位链: MOVE_A(右移850) → CORRECT_A(航向校正) → MOVE_B(右移850)
     *           → CORRECT_B(停稳 TARGET_STOP_SETTLE_MS) → MOVE_C(摆 TARGET_LOOK)
     *           → STATE_13 视觉
     *   视觉对准(任务2): 底盘完全不动, 根据 K230 回的 C/L/R 原地小步转底座 ID1;
     *           收到 C 后依次摆 FIRE → LIFT → SCAN_RESET。
     *   收尾: 出了 STATE_13 后走 PART2_MOVE_A(右移) → PART2_CORRECT_A(航向校正)
     *         → 直接进入 STATE_14 救援。
     * ⚠️ TURN_A/TURN_B 与 PART2_MOVE_B/CORRECT_B/MOVE_C 不在流程里(不可达),
     *    PART1_MOVE_B/CORRECT_B/MOVE_C 是【在用】的, 别当废弃删掉。 */
    STATE_12_PART1_MOVE_A,
    STATE_12_PART1_CORRECT_A,
    STATE_12_PART1_MOVE_B,
    STATE_12_PART1_CORRECT_B,
    STATE_12_PART1_MOVE_C,
    STATE_12_TURN_A,
    STATE_12_TURN_B,
    STATE_12_PART2_MOVE_A,
    STATE_12_PART2_CORRECT_A,
    STATE_12_PART2_MOVE_B,
    STATE_12_PART2_CORRECT_B,
    STATE_12_PART2_MOVE_C,
    STATE_13_PERFORMING_TARGETING,          // 视觉: 原地转底座 ID1 对准靶子, 然后摆 FIRE/LIFT/SCAN_RESET

    /* 阶段四: 救援 (2026-10-05 重新定义)
     * 流程: ①后退 → ②航向校准 → ③停下等 3s → ④摆 HOSTAGE_LOOK
     *      → ⑤视觉对准 + 抓取 → ⑥后退 600 → ⑦航向校准
     *      → ⑧后退 600 → ⑨航向校准 → ⑩停下(任务完成) */
    STATE_14_MOVE_FORWARD_B,                 // ① 后退 ROUTE_14_TO_HOSTAGE_MM
    STATE_15_TURN_FOR_HOSTAGE,               // ② 航向校准
    STATE_15A_RESCUE_STOP_WAIT,              // ③ 原地停等 RESCUE_STOP_WAIT_MS(3000ms)
    STATE_16_RESCUE_RIGHT_A,                 // ④ 摆 ARM_POSE_HOSTAGE_LOOK(看人质)
    STATE_16A_RESCUE_HEADING_CORRECT,        // (未使用) 备用航向校正
    STATE_17_RESCUE_RIGHT_B,                 // ⑥ 抓完后第 1 段后退 ROUTE_17_RIGHT_B_MM
    STATE_17A_RESCUE_HEADING_CORRECT,        // ⑦ 航向校准
    STATE_18_RESCUE_RIGHT_C,                 // ⑧ 抓完后第 2 段后退 ROUTE_18_RIGHT_C_MM
    STATE_18A_RESCUE_HEADING_CORRECT,        // ⑨ 航向校正
    STATE_19_RESCUE_RIGHT_D,                 // ⑩ 停下 → MISSION_STATE_COMPLETE
    STATE_19A_RESCUE_HEADING_CORRECT,        // (未使用)
    STATE_20_PERFORMING_HOSTAGE_RESCUE,      // ⑤ 底盘不动, 交给视觉子状态(任务 3=救援)
    STATE_21_RESCUE_RIGHT_E,                 // (未使用)
    STATE_21A_RESCUE_HEADING_CORRECT,        // (未使用)
    STATE_22_RESCUE_RIGHT_F,                 // (未使用)

    /* 注: 原占位 STATE_16_APPROACH_HOSTAGE / STATE_17_ / STATE_18_RETURNING 未使用,
     *     已并入上方序列; 后续如要“返回起点”阶段再追加即可。 */
} MissionState_t;

/* ---------------- 全局状态 ---------------- */
extern volatile MissionState_t g_mission_state;
extern volatile uint8_t g_vision_task_in_progress;
extern char g_qr_code_string[8];

/* ---------------- 机械臂姿态表(示教标定/动作编排用) ----------------
 * 每个姿态 = 5 个舵机的位置(0~4095), 顺序固定:
 *   [0]=ID1 底座  [1]=ID2 大臂  [2]=ID3 副关节  [3]=ID4 腕部
 *   [4]=ID5 夹爪(数值小=张开, 数值大=闭合)
 * 数值本体在 MissionControl.c 的 s_arm_pose_table 里(实测标定), 这里只列下标。
 * ⚠️ 相邻两姿态的差值不要超过 2048(半圈): 飞特舵机按“最短路径”转,
 *    超过 2048 会朝反方向甩近一整圈。 */
#define ARM_POSE_COUNT  20
typedef enum {
    ARM_POSE_HOME = 0,      /* 复位/初始姿态(取自 ServoArm.c 的 SERVO_POS_HOME) */
    ARM_POSE_SCAN,          /* 扫码: 车停稳后伸臂给摄像头扫码 */
    ARM_POSE_SCAN_RESET,    /* 扫码之后复位: 扫到码后把机械臂收回 */
    ARM_POSE_BALL_LOOK,     /* 看球: 摄像头对准小球(抓取前视觉对准) */
    ARM_POSE_BALL_PRE,      /* 抓夹移动到小球前 */
    ARM_POSE_BALL_CLOSE,    /* 夹爪夹紧小球 */
    ARM_POSE_BALL_LIFT,     /* 抓到小球后大臂抬起 */
    ARM_POSE_BUCKET_CARRY,  /* 携带姿态: 端着球, 底盘移动到另一侧 */
    ARM_POSE_BUCKET_LOOK,   /* 看桶: 摄像头对准球桶(放置前视觉对准) */
    ARM_POSE_PLACE_PRE,     /* 机械臂移动到放置小球的位置 */
    ARM_POSE_PLACE_OPEN,    /* 夹爪松开(放球) */
    ARM_POSE_PLACE_LIFT,    /* 放置完之后大臂抬起 */
    ARM_POSE_TARGET_READY,  /* 转动到准备识别靶子的位置 */
    ARM_POSE_TARGET_LOOK,   /* 识别靶子: 摄像头对准靶子 */
    ARM_POSE_TARGET_FIRE,   /* 激光发射位 */
    ARM_POSE_TARGET_LIFT,   /* 发射完激光后大臂抬起 */
    ARM_POSE_HOSTAGE_LOOK,  /* 识别人质: 摄像头对准人质 */
    ARM_POSE_HOSTAGE_PRE,   /* 机械臂准备抱人质 */
    ARM_POSE_HOSTAGE_CLOSE, /* 抱紧人质 */
    ARM_POSE_HOSTAGE_LIFT   /* 抱起人质后大臂抬起 */
} ArmPose_t;

/* ---- 姿态表访问/执行接口 ---- */
void ArmAction_SetPositions(uint8_t pose_idx, const uint16_t pos[5]);
const char *ArmAction_GetName(uint8_t pose_idx);
/* 摆到指定姿态(阻塞: 等舵机走完 pose 自己的运动时间 + hold_time 再返回) */
void Arm_GotoPose(uint8_t pose_idx);
/* 分两步摆到指定姿态(阻塞): 先动 first_mask 里的舵机(SERVO_MASK_*),
 * 等它们到位停稳, 再动剩下的。用于实测“一步摆到位会剐蹭”的动作,
 * 例如 PLACE_LIFT: Arm_GotoPoseSplit(ARM_POSE_PLACE_LIFT, SERVO_MASK_ARM_BODY) */
void Arm_GotoPoseSplit(uint8_t pose_idx, uint8_t first_mask);

/* ---- 示教标定模式(联调期) ----
 * KEY2 长按 = 进入/退出示教模式;
 * 示教中: KEY2 = 切换动作数组, KEY1 = 读取当前位置并写入当前动作 */
bool ArmTeach_IsActive(void);
void ArmTeach_Enter(void);
void ArmTeach_Exit(void);
void ArmTeach_NextAction(void);
void ArmTeach_WriteCurrent(void);
uint8_t ArmTeach_GetActionIdx(void);
const char *ArmTeach_GetActionName(void);

/* ---------------- 接口 ---------------- */
void Mission_Init(void);
void Mission_Start(void);
void Mission_Update(void);

/* ---------------- 机械臂阻塞等待的支撑(重要) ----------------
 * 陀螺仪 yaw 只在 main.c 主循环里每 20ms 刷新一次; 而摆臂一次要阻塞好几秒,
 * 期间 TIM9 里的航向/转向闭环会一直读到冻结的角度 → 原地转向的
 * s_turn_remaining 永远减不下去 → 车会一直自转。
 * Mission_Coop_Wait() 就是替代 HAL_Delay 的“协作式等待”:
 *   等待期间自己刷 yaw, 并把 K230 收到的行存进内部小队列(不丢帧)。
 * Mission_SetYawPollHook() 由 main.c 初始化时注入“刷新 yaw”的函数。 */
void Mission_Coop_Wait(uint32_t ms);
void Mission_SetYawPollHook(void (*fn)(void));

/* ---- 视觉单独调试接口(仅 MISSION_DEBUG_VISION_TASK != 0 时使用) ---- */
void Mission_DebugVisionStart(void);
void Mission_DebugVisionUpdate(void);
/* 链路监控(值 5)发送: which 0=reset:0, 1=scan_qr, 2=run_task:1 */
void Mission_DebugVisionLinkSend(uint8_t which);

/* ---- 机械臂单独调试接口(仅 MISSION_DEBUG_ARM_SEQ != 0 时使用) ---- */
void Mission_DebugArmStart(void);
void Mission_DebugArmUpdate(void);
/* KEY2 切换当前调试步骤(序列执行中才生效), 见 MissionControl.c */
void Mission_ChangeStep(void);
/* 1=调试序列尚未启动(等 KEY1); 0=序列执行中。供 main.c 区分 KEY1 是“启动”还是“切步” */
uint8_t Mission_DebugArmIsIdle(void);

#ifdef __cplusplus
}
#endif

#endif /* __MISSIONCONTROL_H */
