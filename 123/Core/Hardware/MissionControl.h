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
#define MISSION_TEST_NO_ARM      1   /* 1=不初始化/不驱动机械臂 */
#define MISSION_TEST_NO_VISION   1   /* 1=K230 不参与, 用模拟数据推进状态机 */

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

    /* 阶段三: 打靶 */
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
    STATE_13_PERFORMING_TARGETING,          // 视觉对准并打靶

    /* 阶段四: 救援 */
    STATE_14_MOVE_FORWARD_B,                 // (原有) 救援前前进(掉头准备)
    STATE_15_TURN_FOR_HOSTAGE,               // (原有) 掉头朝救援方向

    /* 阶段四后半(新增): 营救接近走位: 右移+航向校正 x4 -> 视觉营救 -> 右移收尾 */
    STATE_16_RESCUE_RIGHT_A,                 // 右移 A
    STATE_16A_RESCUE_HEADING_CORRECT,        // 航向校正
    STATE_17_RESCUE_RIGHT_B,                 // 右移 B
    STATE_17A_RESCUE_HEADING_CORRECT,        // 航向校正
    STATE_18_RESCUE_RIGHT_C,                 // 右移 C
    STATE_18A_RESCUE_HEADING_CORRECT,        // 航向校正
    STATE_19_RESCUE_RIGHT_D,                 // 右移 D
    STATE_19A_RESCUE_HEADING_CORRECT,        // 航向校正
    STATE_20_PERFORMING_HOSTAGE_RESCUE,      // 停下, 交给视觉子状态(任务 3=救援)
    STATE_21_RESCUE_RIGHT_E,                 // 右移 E(营救完成后)
    STATE_21A_RESCUE_HEADING_CORRECT,        // 航向校正
    STATE_22_RESCUE_RIGHT_F,                 // 右移 F(收尾, 结束后任务完成)

    /* 注: 原占位 STATE_16_APPROACH_HOSTAGE / STATE_17_ / STATE_18_RETURNING 未使用,
     *     已并入上方序列; 后续如要“返回起点”阶段再追加即可。 */
} MissionState_t;

/* ---------------- 全局状态 ---------------- */
extern volatile MissionState_t g_mission_state;
extern volatile uint8_t g_vision_task_in_progress;
extern char g_qr_code_string[8];

/* ---------------- 接口 ---------------- */
void Mission_Init(void);
void Mission_Start(void);
void Mission_Update(void);

#ifdef __cplusplus
}
#endif

#endif /* __MISSIONCONTROL_H */
