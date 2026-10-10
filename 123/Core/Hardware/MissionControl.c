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
 * 与 K230 的串口协议(与最新的 K230 端 main.py + yolo_main.py 对齐):
 *   MCU -> K230 : "run_task:<n>"    1=球 2=靶 3=桶 4=形状(救援)
 *               : "start_align"     进入精对准(K230 回传 D:<x>,<y> / OK)
 *               : "reset:"         复位 K230 回到 WAIT_CMD
 *   K230 -> MCU : "SCAN_OK"         扫码完成(main.py 扫到 3 位目标号后回传)
 *               : "C"/"L"/"R"       接近阶段: 目标在画面 中/左/右
 *               : "D:<x>,<y>"       精对准: 像素误差(x = 画面中 - 目标x,
 *                                    y = 目标y - 画面中)
 *                                    x>0 → 目标偏画面左; y>0 → 目标偏画面下
 *                                    (K230 每周期只发其中一个轴, 另一轴为 0)
 *               : "OK"              已对准
 *               : "FIRE"            任务2 专用: K230 已点亮激光(仅通知)
 *   ⚠️ 新 K230 【不再支持 "scan_qr"】: 它在 main.py 阶段用自己的摄像头扫码,
 *      扫到即回一行 "SCAN_OK" 并重启进入 yolo_main; 所以 STM32 侧不再请求扫码,
 *      只在 STATE_2 等 "SCAN_OK"(见那里的注释与 QR_WAIT_TIMEOUT_MS)。
 *   ⚠️ 激光由 K230 自己控制(K230 的 FIRE_PIN_NUM): 打靶时 STM32 必须发
 *      start_align 让 K230 进入 ALIGN 状态, 它对好之后才会 "FIRE"+"OK"。
 *      见 TARGET_USE_FINE_ALIGN。
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

/* ⭐⭐ 方案②(陀螺仪零偏)的【起锚范围】开关 --------------------------------------
 * 与 MissionControl.h 里那两个 SCOPE 开关同族, 但只有本文件用到, 所以定义在这里。
 *   SCOPE_TAIL   (在 .h) = 1 : 【抓取人质后 → 终点】, 在 STATE_17 起锚 / STATE_19 停锚
 *   SCOPE_TARGET (在 .h) = 0 : 【打靶走位段】, 在 STATE_11A 起锚 / STATE_14 停锚
 *   SCOPE_RESCUE (本文件) = 0: 【进救援区右移 → 对准人质 → 抓取人质】这一段,
 *                              在 STATE_15C 停稳时起锚 / STATE_19 停锚
 *   ⭐ 三者互相独立, 不许混用: SCOPE_RESCUE = 1 时【不再】在 STATE_17 重复起锚
 *      (重复起锚会把 corrected_yaw 重新对齐当前 yaw, 顺手清掉刚建好的前馈链),
 *      统一由 STATE_15C 一路用到 STATE_19 —— 这正是原版 f8106e7 的做法。
 *   ⚠️ 当前取值: TAIL = 1 / TARGET = 0 / RESCUE = 0
 *      ⇒ 只有【抓取人质后 → 终点】用方案②, 其余全程走方案①(原始 yaw)。
 *   ⚠️ 原来 STATE_15C 那个起锚错挂在 GYRO_BIAS_TGT_ON 下, 一旦把 SCOPE_TARGET
 *      打开就会把这一段一起激活 ⇒ 已改用本开关, 彻底解耦。
 */
#define GYRO_BIAS_SCOPE_RESCUE      0     /* 1 = 救援区右移→对准→抓取 这一段也用方案② */
#if (GYRO_BIAS_SCHEME && GYRO_BIAS_SCOPE_RESCUE)
#define GYRO_BIAS_RESCUE_ON         1
#else
#define GYRO_BIAS_RESCUE_ON         0     /* 方案① 或 本段不启用: 一律不起锚 */
#endif

/* ================= 全局变量定义 ================= */
volatile MissionState_t g_mission_state = MISSION_STATE_IDLE;   // 任务状态机当前状态
volatile uint8_t g_vision_task_in_progress = 0;                 // 视觉子状态机任务号 (0=无任务, 1=排爆, 2=打靶, 3=救援)
char g_qr_code_string[8];                                       // 扫码结果字符串(最多 7 字节 + '\0')

/* ⭐⭐ 2026-10-11(用户要求): 机械臂"保持(稳定)时间"分两档 ------------------------
 * 原来只有全局一个 hold_time = 500ms, 所有姿态都一样。现在:
 *   【打靶】那几步 → ARM_HOLD_MS_TARGET(200ms)
 *     打靶时底盘不动, 只是底座/腕部在靶方向上微调, 舵机到位就稳了 ⇒ 多等纯属浪费;
 *     而且打靶那几帧要跟 K230 的识别节奏对齐, 等太久反而错过好帧。
 *   【其它】(排爆抓球/放球、扫码、救援抓取/回程…) → ARM_HOLD_MS_OTHER(450ms)
 *     这些多带夹爪开合, 且常常是"车刚停稳就伸臂/收臂", 多给一点时间消抖
 *     (原来是 500, 现在 450, 只少 50ms, 手感基本不变)。
 * 【怎么判定哪一档】按【姿态名】: 见 Arm_IsTargetPose() —— 名字属
 *   ARM_POSE_TARGET_*(READY/LOOK/FIRE/LIFT) 的算"打靶"。一轮动作里每个姿态
 *   各取自己那一档, 所以同一个阶段里可以混(例如打靶收尾那步 SCAN_RESET 由调用处
 *   显式指定成打靶档)。
 * ⚠️ 想整体恢复"统一 500ms": 把两个宏都改成 500。
 * ⚠️ 保持时间【太短】的典型症状: 下一帧视觉还没稳定/舵机还在回间隙就动了下一动作
 *    (看起来"动作被吃掉一步"或"对准后又偏了") —— 那就把对应那一档加大。 */
#define ARM_HOLD_MS_TARGET          200    /* 打靶那几步的保持时间(ms) */
#define ARM_HOLD_MS_OTHER           450    /* 其它所有姿态的保持时间(ms) */

/* "打靶准备位"(ARM_POSE_TARGET_READY)算哪一档?
 * ⚠️ 它的执行时机比较特殊: 它在【排爆的最后】被摆出来(放完球 → 摆 TARGET_READY
 *    → 车才开始打靶走位), 但姿态名属 TARGET_* 家族、用途也是给打靶做准备。
 *    1 = 算打靶档(200ms, 与其余 TARGET_* 一致);
 *    0 = 算其它档(450ms, 若实测这一步太快、进打靶时臂还晃, 就改 0)。 */
#define ARM_HOLD_TARGET_READY_IS_TARGET   1

/* ⭐⭐ 2026-10-11(用户要求): 排爆【抓球】那三步的时间再给长一点 ----------------------
 * 抓球 = BALL_PRE(伸到球前) → BALL_CLOSE(合夹爪) → BALL_LIFT(抓着抬大臂), 见
 *   Arm_Start_Bomb_Grab()。
 * 原来: 运动时间 2500/2000/2500 + 每步保持 450(ARM_HOLD_MS_OTHER) = 共 8350ms;
 * 现在: 3200/2600/3200 + 每步保持 ARM_HOLD_MS_GRAB(600) = 共 10800ms(+2.45s)。
 * 为什么要加长: 这三步都是"贴着小球"的小动作, 快了下发还没走完就发下一条 → 夹爪顶飞球、
 *   或被球桶边缘挂到; 慢一点球更不容易滑掉、抬起瞬间也更稳。
 * ⚠️ 只影响这三个姿态(BALL_PRE/CLOSE/LIFT), 表里其它行和其它阶段都不受影响。
 * ⚠️ 嫌总时间太长 → 按比例调这三个宏(例如 2900/2300/2900 + hold 500)。
 * ⚠️ 想完全回退到改之前: 三个宏改回 2500/2000/2500, ARM_HOLD_MS_GRAB 改回 450。 */
#define ARM_GRAB_PRE_MS             3200   /* BALL_PRE   抓夹伸到小球前(原 2500) */
#define ARM_GRAB_CLOSE_MS           2600   /* BALL_CLOSE 合夹爪夹紧(原 2000) */
#define ARM_GRAB_LIFT_MS            3200   /* BALL_LIFT  抓着球抬大臂(原 2500) */
#define ARM_HOLD_MS_GRAB            600    /* 抓球这三步各自的保持时间(原来走 ARM_HOLD_MS_OTHER=450) */

#if !MISSION_TEST_NO_ARM
/**
 * @brief  该姿态是否属于"打靶"那一档(决定保持时间用 200 还是 450)
 * @param  pose_idx ARM_POSE_*
 * @retval 1 = 打靶档; 0 = 其它档
 */
static uint8_t Arm_IsTargetPose(uint8_t pose_idx)
{
    if (pose_idx == ARM_POSE_TARGET_READY) {
        return (uint8_t)ARM_HOLD_TARGET_READY_IS_TARGET;
    }
    return (pose_idx == ARM_POSE_TARGET_LOOK ||
            pose_idx == ARM_POSE_TARGET_FIRE ||
            pose_idx == ARM_POSE_TARGET_LIFT) ? 1u : 0u;
}

/** @brief 该姿态是否属于排爆【抓球】那三步(BALL_PRE/CLOSE/LIFT), 见 ARM_GRAB_*_MS */
static uint8_t Arm_IsGrabPose(uint8_t pose_idx)
{
    return (pose_idx == ARM_POSE_BALL_PRE ||
            pose_idx == ARM_POSE_BALL_CLOSE ||
            pose_idx == ARM_POSE_BALL_LIFT) ? 1u : 0u;
}

/** @brief 按姿态号取保持时间(ms): 抓球 600 / 打靶 200 / 其它 450, 见各 ARM_HOLD_MS_* 宏 */
static uint16_t Arm_HoldMs(uint8_t pose_idx)
{
    if (Arm_IsGrabPose(pose_idx)) {
        return (uint16_t)ARM_HOLD_MS_GRAB;
    }
    return Arm_IsTargetPose(pose_idx) ? (uint16_t)ARM_HOLD_MS_TARGET
                                      : (uint16_t)ARM_HOLD_MS_OTHER;
}
#endif /* !MISSION_TEST_NO_ARM */

/* ⭐ 分步动作(Arm_GotoPoseSplit)第 2 步(腕/夹爪)的最短运动时间(ms)。
 * 第 2 步默认取该姿态运动时间的一半, 但不短于这个值, 免得小动作等得没必要 */
#define ARM_SPLIT_2ND_MIN_MS   1000

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
#define VISION_CMD_RESEND_MS    1000   /* scan_qr / run_task 指令定时重发间隔(ms):
                                        * K230 扫码后会重启并重新加载模型(需数秒),
                                        * 期间 STM32 发的指令会丢; 定时重发直到收到回应。
                                        * 建议 800~1500 */
#define VISION_MODE_SETTLE_MS   500    /* ⭐ 换任务“静默期”(ms): 发新的 run_task 后
                                        * 这么久内收到的行【全部丢弃】。
                                        * 原因: K230 是连续输出的(比如每 33ms 一帧),
                                        * 发 run_task 那一刻它还在按【上一个任务】
                                        * 的模式往外发 C/L/R/OK, 那些帧已经在路上
                                        * 或者刚进 UART, K230_FlushAll() 清不掉。
                                        * 不丢的话会被当成新任务的结果立刻误判:
                                        * 实测打靶刚发完 run_task:2 就读到桶阶段的
                                        * 'C' → ID1 一步没转就去摆发射位了。
                                        * 建议 300~800。太短→可能清不干净旧帧;
                                        * 太长→白等(不影响正确性, K230 会持续发新帧) */
#define VISION_RESPONSE_TIMEOUT_MS 8000 /* 视觉指令发出后仍无回应(如 K230 该任务未实现)
                                        * 的最大等待(ms), 超时跳过视觉盲走执行(防卡死) */
#define QR_WAIT_TIMEOUT_MS      20000  /* ⭐ 扫码等待超时(ms): 新 K230 在 main.py 阶段
                                        * 【自己】扫码, 扫到才回 "SCAN_OK" 并重启进入
                                        * 识别模式; STM32 既不能也不该主动请求扫码,
                                        * 只能在 STATE_2 干等。
                                        * 超时仍没等到的话(例如 K230 里残留了旧的
                                        * /sdcard/target.txt, 它直接进了识别模式)
                                        * 就继续往下走, 免得整场卡死在扫码点。
                                        * 建议 15000~40000 */

/* ---- ⭐ 扫码“多角度扫描” (2026-10-06 新增) ----------------------------
 * 目的: 只用一个固定姿态扫码时, 遇到二维码反光 / 角度偏 / K230 视野边缘
 *       就扫不到。用腕部 ID4 微调出 3 个角度轮着扫, 命中率高很多。
 * 做法: 进入 STATE_2 先摆 ARM_POSE_SCAN(其余 4 个舵机保持不动), 之后每隔
 *       SCAN_MOVE_MS + SCAN_HOLD_MS 用掩码【只改 ID4】换一个角度:
 *           ID4 = SCAN基准 + 0  →  SCAN基准 - SCAN_ID4_DELTA
 *                                →  SCAN基准 + SCAN_ID4_DELTA
 *       每个角度停 SCAN_HOLD_MS 给 K230 拍照识别; 期间它若回了 SCAN_OK
 *       就立刻结束本状态(不会白等三个角度走完)。
 * 调参: 还扫不到 → 加大 SCAN_ID4_DELTA(如 30~40) 或 SCAN_HOLD_MS(如 1200);
 *       嫌慢     → 减 SCAN_HOLD_MS, 或把 SCAN_MOVE_MS 调到 300。
 * 总耗时 ≈ 3 × (SCAN_MOVE_MS + SCAN_HOLD_MS) ≈ 3.9s, 远小于
 *         QR_WAIT_TIMEOUT_MS(20s), 不会把整个扫码环节拖到超时。 */
#define SCAN_ID4_DELTA          60     /* 每个角度相对 SCAN 姿态偏移多少码(4096=360°) */
#define SCAN_MOVE_MS            500    /* 换角度时 ID4 的转动时间(ms) */
#define SCAN_HOLD_MS            800    /* 每个角度到位后停留多久(ms), 给 K230 识别 */

/* ================= 视觉精对准过程时序 (ms) ================= */
#define FINE_TUNE_COOLDOWN_MS   750    /* 每次修正动作后的冷却时间:
                                        * 期间忽略 K230 新帧, 避免指令叠车。
                                        * 建议 500~1200。太小→上一条没走完又来一条;
                                        * 太大→对准变慢 */
#define FINE_TUNE_SETTLE_MS     600    /* 判“已对准/停稳”后额外停车等待时间,
                                        * 等惯性晃动结束再执行(抓/打)。建议 300~800 */
#define FORCE_GRAB_AFTER_MS     15000  /* 精对准开始后超过该时长(ms)仍未对准:
                                        * 放弃继续对准, 直接执行任务(防超时卡死) */
#define QR_SIM_WAIT_MS          1000   /* 测试模式: 到扫码点后停车等待多久才模拟扫到码 */

/* ================= 视觉对准参数 (像素误差 -> 修正距离) ================= */
/* ⚠️ 先看两个硬约束(不知道会白调):
 *   ① 底盘有个“最小有效步长”: Chassis.h 的 CH_POS_THRESHOLD_COUNT = 40 计数
 *      ≈ 6mm(CH_COUNTS_PER_MM = 1560/(75π) ≈ 6.62), 即单轮残差 < 6mm 就判“到位”。
 *      ⇒ MIN_MOVE_MM 必须【明显大于 6mm】, 否则指令下去车根本不动。
 *   ② 但步长也不能大: 每次修正都冲过头 → 左右来回横移、永远进不了 ALIGN_TOLERANCE。
 *      “一直接近不了”通常就是 K_GAIN + MIN_MOVE_MM 偏大造成的。 */
#define K_GAIN              0.09/*0.25f*/       /* 像素误差 → 移动距离 的比例 (mm/像素)。
                                         * 太大→每帧都冲过头来回振荡(就是“一直左右横移
                                         * 接近不了”的典型原因); 太小→老修不到位。
                                         * ⭐ 标定: 人为把目标挪开已知距离 D(mm), 读 K230
                                         *    回的 |err_x|(px) → 实际比例 a = D/|err_x|
                                         *    (mm/px), 然后取 K_GAIN ≈ 0.5×a(留一半余量)。
                                         * 默认 0.25(原来是 0.5, 偏大) */
#define MIN_MOVE_MM         5/*10*/          /* 单次修正的最小距离(mm): “暴力起步”用。
                                         * 必须 > 底盘到位死区(≈6mm)才真能动起来;
                                         * 建议 10~20。原来 35 → 只能左右荡 */
#define MAX_MOVE_MM         60          /* 单次修正的最大距离(mm), 防止一开始误差很大时一下
                                         * 冲太远。建议 40~100。原来 120 */
#define ALIGN_TOLERANCE     25          /* 【底盘阶段(球/桶/救援)】判定“已对准”的像素容差。
                                         * ⚠️⚠️ 两个方向都有硬约束, 必须同时满足:
                                         * 【上限】≤ K230(yolo_main.py)那边本任务的
                                         *   对准窗口(ALIGN_TOL_PIX)。本机是靠“|误差| ≥
                                         *   容差就再动一步”推进的, 本机容差比 K230 大 ⇒
                                         *   误差落在两者之间时本机认为“够准了、不再动”,
                                         *   K230 却认为没对准、继续发 D 帧
                                         *   ⇒ 车停在半路就执行(放球/抓取)。
                                         *   ⭐ 2026-10-07 桶“放不准”就是这个。
                                         * 【下限】≥ 底盘的【物理分辨率】, 否则永远达不到、
                                         *   只会一路修到超时兜底。实测(2026-10-08):
                                         *   到位死区 CH_POS_THRESHOLD_COUNT(30 计数)
                                         *   ≈ 4.5mm ≈ 25px(a≈0.161~0.187 mm/px)
                                         *   ⇒ 窗口比 25px 还小(如 14px≈2.3mm)时底盘
                                         *     根本停不到那里, 只能等
                                         *     FORCE_GRAB_AFTER_MS/ALIGN_TIMEOUT_MS
                                         *     “放弃对准直接执行”。
                                         * ⭐ 2026-10-08 定值 25: 上下都满足
                                         *   (K230 那边同步改成 25, 死区 ≈25px) —— 能真收敛,
                                         *   精度 ≈ ±4mm。
                                         * ⭐⭐ 2026-10-11(用户要求): **25 → 15** ——
                                         *   用户已把 K230 那边(抓球/放桶)的对准窗口改成 15px,
                                         *   本机跟它保持一致(避免"本机认为够准、K230 还在发 D 帧")。
                                         * ⚠️ 风险提示(改之前已在工程注释里写明): 15px 已【低于】
                                         *   底盘单步的粒度 —— 最小步 8mm ≈ 42~89px(a≈0.18~0.09),
                                         *   一步就跨出 15px 的带子 ⇒ 本机很难"一步落进容差",
                                         *   实际收敛主要靠 ① K230 发 OK, 或 ② 某一帧恰好进带后
                                         *   连续 2 帧(ALIGN_DONE_FRAMES)确认。
                                         * ⚠️ 若实测发现"对准变慢 / 老是走到 15 秒强制放弃",
                                         *   按这个顺序处理: ① 改回 25(和 K230 一起改回);
                                         *   ② 或者做配套: ALIGN_STEP_MIN_MM 8→3 + 给球也开
                                         *      "对准小死区"(CH_ALIGN_FINE_*), 让步长能小于容差。
                                         * ⚠️ 打靶不走这里: 它走舵机(不是底盘), 用
                                         *    TARGET_ALIGN_TOLERANCE;
                                         *    每个阶段实际生效的容差放在 AlignAxisCfg_t::tol_px */

/* ⭐⭐ 2026-10-08 新增: “已进容差”要连续确认几帧才算真的对准。
 * 【为什么加】原来收到【一帧】|误差| < tol_px 就立刻判定“已对准 → 停车执行
 *   (放球/抓取)”。K230 每帧都在发 D, 单帧误检、或目标正好在容差边缘抖一下,
 *   就会让车还没摆正就开干 —— 现场表现就是“没正对桶, 机械臂直接放下去了”。
 * 取 1 = 旧行为(单帧即判定); 建议 2~3。
 * ⚠️ 别设太大: 目标卡在容差边缘来回跳时可能一直凑不齐, 最后只能走超时兜底。 */
#define ALIGN_DONE_FRAMES       2

/* =====================================================================
 * ⭐⭐ 底盘精对准: 自适应步长 (2026-10-07 新增)
 * ---------------------------------------------------------------------
 * 【为什么要自适应】实测反馈“排爆阶段有时候对齐不准”。这里有两个硬约束:
 *   ① 底盘有【死区】: Chassis.h 的 CH_POS_THRESHOLD_COUNT(40 计数)
 *      ⇒ CH_DEADZONE_MM ≈ 6.0mm(40 ÷ 6.62 计数/mm)。
 *      单步指令比它小 ⇒ 车基本不动(指令被死区吃掉), 误差一直不降,
 *      看起来就是“对不准”; 而且最终精度也不可能优于这个死区。
 *   ② 像素↔毫米比例 K_GAIN 是估的: 偏大 ⇒ 每步都冲过头来回摆;
 *      偏小 ⇒ 修得慢, 而且小误差时算出的步长一旦小于死区就彻底不动了。
 * 【做法】(和打靶 ID1 那套“分档 + 实测标定”同一个思路)
 *   ① 分档: |误差| > ALIGN_STEP_BIG_PX(250px) → 直接走上限快赶过去;
 *      否则 步长 = |误差| × a × ALIGN_STEP_DAMP_PCT%(默认 70%, 留余量防过冲)
 *   ② 下限 ALIGN_STEP_MIN_MM(12mm) 必须 > 底盘死区(CH_DEADZONE_MM≈6mm),
 *      否则指令下去车不动;
 *   ③ ⭐【自学习】a: 用上一步实测的“走了多少 mm / 误差变了多少 px”算出真实比例
 *      (见 Vision_FineAlignProcess 里的 [K标定] 实测), 攒够
 *      ALIGN_ADAPT_MIN_SAMPLES 步后用实测值替代 K_GAIN —— 这就是“自适应”的关键:
 *      比例越跑越准 ⇒ 大误差不冲过头、小误差也不会因为步长太小而卡死;
 *   ④ 上一步【冲过头】(误差换向) → 这一步减半, 抑制来回摆。
 * 【日志】每步都打, 照着看就行:
 *      [球][自适应] 横向: 误差 -180px -> 左移 24mm (a≈0.187 mm/px 实测自学习, 下限12mm>死区6.0mm)
 *      [球][自适应] 横向: 误差  -60px -> 左移 12mm (a≈0.187 mm/px 实测自学习, 上步冲过头已减半)
 *   怎么看:
 *      a 一直显示“K_GAIN估算” ⇒ 实测样本不够(每步都被下限截断/方向反了),
 *                                看 [K标定] 那几行的警告;
 *      步长长期被 12mm 下限抬着、车却“不动” ⇒ 死区比 6mm 大: 把
 *                                ALIGN_STEP_MIN_MM 加到 15~18, 或把 Chassis.h 的
 *                                CH_POS_THRESHOLD_COUNT 调小(死区变小 = 最终更准)。
 * ===================================================================== */
/* ⭐ 底盘死区(mm): 由 Chassis.h 的两个宏自动算出(40 计数 ÷ 6.62 计数/mm ≈ 6.0mm)。
 *   ⚠️ 单步指令小于它, 车基本不动 —— 自适应步长的下限就是按它定的。 */
#define CH_DEADZONE_MM          ((float)CH_POS_THRESHOLD_COUNT / CH_COUNTS_PER_MM)
#define ALIGN_STEP_MIN_MM       8.0f//12.0f   /* 步长下限(mm): 必须 > CH_DEADZONE_MM */
/* ⭐⭐ 2026-10-11 用户实测反馈(第二次): 【放桶修正步长不能太小】——
 *   "太小会导致车身偏移"。实测就是这么来的: 3mm 的单步只有约 20 计数, 四轮的
 *   到位判定各自独立(细模式死区 12 计数), 于是【部分轮子动了、部分轮子根本没动】
 *   ⇒ 车不是整体平移而是"原地蹭", 反复几次小步之后车身整体偏掉/车头也偏。
 *   ⇒ 改回下单步粒度: 下限 = ALIGN_STEP_MIN_MM(8mm ≈ 53 计数), 四轮都真的走,
 *     一次平移到位、步数少 ⇒ 偏移少。
 * 【为什么以前调成 3】容差收到 15px 后怕 8mm 一步跨出容差带(8mm ÷ 0.18 ≈ 44px)。
 *   用户的选择是: **宁可步长大、靠 2~3 步收敛, 也不要小步蹭出车身偏移**
 *   ("步长修正大点无所谓, 能收敛就行")。收敛靠: ① K230 发 OK;
 *   ② 连续 ALIGN_DONE_FRAMES(2) 帧进带; ③ 自适应步长的"上步冲过头减半"。
 * ⚠️ 与 15px 容差的配合: 8mm 步对应约 44px, 所以"一步正好落进 15px"是碰运气 ——
 *   减半后 4mm ≈ 22px 仍略大于 15px。若实测发现"老是走不到 15px 内、最后靠超时
 *   兜底", 按这个顺序处理: ① 把容差改回 25px(K230 一起改); 或
 *   ② 把这个下限降到 5~6, 同时把 ALIGN_STEP_DAMP_PCT 从 70 降到 40
 *      (让"接近时"的步长成比例变小, 而不是靠硬性小下限)。
 * ⚠️ 细模式(桶)仍走【小到位死区】(CH_ALIGN_FINE_THRESHOLD_COUNT ≈1.8mm),
 *   否则 8mm 指令会被 4.5mm 的死区吃掉一大截 = 又回到"个别轮子动"的老问题。 */
#define ALIGN_STEP_MIN_MM_FINE  3.0f          /* 细模式(桶)的步长下限(mm) */
#define ALIGN_STEP_MAX_MM       80.0f   /* 步长上限(mm)(= 原来的 MAX_MOVE_MM) */
#define ALIGN_STEP_BIG_PX       250     /* |误差| > 它 → 直接走上限(大步赶过去) */
#define ALIGN_STEP_DAMP_PCT     70      /* 按比例算出的步长打几折(留余量防过冲) */
#define ALIGN_ADAPT_MIN_SAMPLES 2       /* 攒够几次有效实测, 才切到“自学习比例” */
#define ALIGN_ADAPT_A_MIN       0.02f   /* 实测比例 a 的合理范围(mm/px), 超范围丢弃 */
#define ALIGN_ADAPT_A_MAX       1.00f

/* ---- ⭐ 前后(F/B)方向的【独立】步长 (2026-10-04 新增) ----
 * 为什么要单独给一套:
 *   左右只是把目标“摆正”(偏一点关系不大), 而前后是“要不要再往前贴”
 *   的距离 —— 桶/人质就在眼前, 用和左右一样大的步长(最坏 MAX_MOVE_MM=60mm)
 *   一步就冲过头甚至撞上去。
 * 生效条件: 该阶段 allow_fb=1, 即只有【方案二】(ALIGN_AXIS_SCHEME_1=0)。
 *   方案一的球/桶 allow_fb=0、根本不修前后, 这三个宏不参与。
 * 计算公式: 前后步长 = |err| × K_GAIN × FB_SCALE_PCT% ,
 *                  再用 FB_MIN_MM ~ FB_MAX_MM 限幅。
 * ⭐ 实测反馈“左右合适、前后冲太多” → 只需调下面 3 个宏(或桶专用的那 3 个)。 */
#define FB_SCALE_PCT        15          /* 前后步长比例(%): 100=和左右一样大,
                                         * 50=一半。建议 30~60, 不要填 0
                                         * (填 0 会被 FB_MIN_MM 兜底成最小步) */
#define FB_MIN_MM           7          /* 前后单步最小距离(mm): 必须 > 底盘死区≈6mm,（4.5）
                                         * 否则指令下去车不动。建议 10~15 */
#define FB_MAX_MM           20          /* 前后单步最大距离(mm): 比 MAX_MOVE_MM(60)
                                         * 小得多, 就是防止“一下冲过头”。
                                         * 建议 15~30 */

/* ---- 桶阶段专用的前后步长(单独一份, 只调桶不影响其它阶段) ----
 * 想让桶的前后更小/更大, 只改这三行即可。 */
#define BUCKET_FB_SCALE_PCT FB_SCALE_PCT    /* 桶: 前后步长比例(%) */
#define BUCKET_FB_MIN_MM    FB_MIN_MM       /* 桶: 前后单步最小(mm) */
/* ⭐⭐ 2026-10-10 用户要求: 桶“往前/往后”的单步距离【不能大于底盘到位死区的 2 倍】。
 *   底盘死区 = CH_DEADZONE_MM = CH_POS_THRESHOLD_COUNT(30 计数)/CH_COUNTS_PER_MM
 *   ≈ 4.5mm ⇒ 2 倍 ≈ 9mm。超过它就容易一步冲过桶/撞桶(甚至跨过容差来回摆)。
 *   ⚠️ 若 Chassis.h 里改了 CH_POS_THRESHOLD_COUNT, 请同步改下面两个数(它们是同一个值):
 *      BUCKET_FB_MAX_MM        = 实际用的上限
 *      BUCKET_FB_MAX_ALLOWED_MM= 允许的上限(防呆上限, 别调大)
 *   ⚠️ 下限 BUCKET_FB_MIN_MM(=FB_MIN_MM=7) 必须 > 死区(4.5mm), 否则指令下去车不动。 */
#define BUCKET_FB_MAX_ALLOWED_MM   9        /* = 2 × 死区(≈4.5mm) ⇒ 别往上调 */
#define BUCKET_FB_MAX_MM           9        /* 桶: 前后单步最大(mm), 必须 ≤ 上面那个 */

#if BUCKET_FB_MAX_MM > BUCKET_FB_MAX_ALLOWED_MM
#error "BUCKET_FB_MAX_MM 超过 2×底盘到位死区: 桶的前后步长会一步冲过去, 请调回 ≤ 9"
#endif

/* ⭐⭐ 桶“左右对准完成之后”按前后误差后退的【传统(开环)做法】 --------------------
 * ⚠️ 总开关: BUCKET_AFTER_ALIGN_BACK_ENABLE
 *    0(★当前) = 关闭 —— 深度改由【臂的 ID2 补偿】修, 与【抓球完全同款】
 *               (见 BOMB_FB_ARM_ENABLE_BUCKET): 底盘一步不动, 只在放桶姿态
 *               (PLACE_PRE / PLACE_OPEN)下发前, 按“对准那一刻的 Y 误差”
 *               把 ID2 往前/往后补几码;
 *    1        = 打开老做法(底盘按下面两档后退), 此时必须把 BUCKET_FB_CL_ENABLE 置 0,
 *               并且建议把 BOMB_FB_ARM_ENABLE_BUCKET 也置 0 (两者作用在同一自由度)。
 * 【作用位置】排爆子状态机: BOMB_WAIT_BUCKET_ALIGN(精对准桶)对准成功
 *   → 【本步: 往后退这么多】→ BOMB_SETTLE_PLACE(停稳) → BOMB_PERFORM_PLACE(放球)。
 * 【怎么定退多少】按 K230 回的“和桶的前后(深度)误差”分两档:
 *   |前后误差| >  BUCKET_AFTER_ALIGN_BACK_BIG_PX(误差大) → 退 BUCKET_AFTER_ALIGN_BACK_BIG_MM
 *   |前后误差| ≤  BUCKET_AFTER_ALIGN_BACK_BIG_PX(误差小) → 退 BUCKET_AFTER_ALIGN_BACK_SMALL_MM
 *   误差值 = D:<x>,<y> 的 y(桶姿态下画面 Y = 车体前后), 由 BombFbCapture() 在
 *   “对准成功那一刻”抓下来(s_bomb_fb_px; 正 = 目标偏远)。判据只看【大小】, 方向固定后退。
 *
 * ⚠️⚠️ 2026-10-10 起本机制默认【关闭】(BUCKET_AFTER_ALIGN_BACK_ENABLE = 0), 原因:
 *   ① 这是【开环】修正 —— 退完不再看误差, 退多少只能靠实测标定, 而且“该退还是该进”
 *      与误差符号有关, 现在的写法(只看大小)在“桶偏远”时会和臂的往前伸互相抵消;
 *   ② 深度改由【闭环】修(见 BUCKET_FB_CL_* 那一段): 走一小步→再看一帧 y→再走,
 *      比例自学习、过冲自动减半、方向可自动纠正 ⇒ 不需要标定, 也不会打架。
 *   (想回到老做法: 把 BUCKET_AFTER_ALIGN_BACK_ENABLE 置 1, 并记得把桶的
 *    BUCKET_FB_CL_ENABLE 置 0 —— 两条【只能开一条】。)
 * ⚠️ 到位死区 ≈ 4.5mm ⇒ 实际后退 ≈ 本值 − 4.5mm; 绝对值别小于 6。
 * ⚠️ 只走【对准成功】那条路; 超时兜底仍然直接去放球、不动车。
 * ⚠️ 这一步走完会【重新计停稳时间】(FINE_TUNE_SETTLE_MS), 免得边走边放球。 */
#define BUCKET_AFTER_ALIGN_BACK_ENABLE    0    /* 0 = 关(深度走【臂补 ID2】, 与抓球同款); 1 = 开老做法(底盘分档后退) */
#define BUCKET_AFTER_ALIGN_BACK_SMALL_MM  10   /* 前后误差【小】→ 后退 10mm */
#define BUCKET_AFTER_ALIGN_BACK_BIG_MM    15   /* 前后误差【大】→ 后退 15mm */
#define BUCKET_AFTER_ALIGN_BACK_BIG_PX    40   /* 判“误差大”的门限(px, 只看绝对值) */

/* =====================================================================
 * ⭐⭐ 2026-10-10 新增: 桶【前后(深度)】轴的【闭环】修正 + 3 个自动兜底 -----------
 * ---------------------------------------------------------------------
 * 【为什么换成闭环】桶的前后误差原来有两条开环路: 臂的 ID2 补 10/20/30 码、以及上面
 *   那个“分档后退 10/15mm”。两者都必须实测标定(“码→伸出多少 mm”“该退还是该进”),
 *   而且作用在同一个自由度上会互相抵消。闭环则不用标定: 走一小步 → 再看一帧 y →
 *   再走, 比例由 s_adapt_a 自学习, 过冲(误差跨过中心)自动减半。
 *
 * 【3 个自动兜底 —— 全部不需要人工标定/判断方向】
 *   ① 方向自动纠正: 某一步之后 |y| 反而【变大】超过 BUCKET_FB_CL_FLIP_MIN_PX,
 *      就认为极性反了 → 自动翻转一次(并记住); 若翻转后还在变大, 直接放弃前后修正。
 *   ② 步数/累计位移上限: 前后最多修 BUCKET_FB_CL_MAX_STEPS 步, 累计位移不超过
 *      BUCKET_FB_CL_MAX_TOTAL_MM —— 超了就“接受现状”直接去放球, 绝不拖到超时。
 *   ③ 朝桶侧步长限小: 单步上限按方向分开 —— 朝桶那侧(车头前进)用 FWD_MAX_MM,
 *      离开桶那侧(车头后退)用 BACK_MAX_MM。就算极性判断错了, 第一步也只走 6mm。
 *
 * 【判“够准”的容差】BUCKET_FB_CL_TOL_PX(前后比左右宽松): 深度差几 mm 球照样进桶,
 *   但左右差一点就可能落在桶外 —— 所以左右继续用 ALIGN_TOLERANCE, 前后单独放宽。
 *   这也顺手避免了“前后收敛不到 25px → 一直等到 30s 超时”这种事。
 * 【怎么关】BUCKET_FB_CL_ENABLE = 0 ⇒ 桶回到“只修左右”(老行为)。
 * ⚠️ 与 BUCKET_AFTER_ALIGN_BACK_ENABLE、BOMB_FB_ARM_ENABLE_BUCKET 三者【只能开一条】:
 *    桶的深度修正应该只有一个执行者。
 * ⭐⭐ 2026-10-10(用户要求): 本闭环【已关闭】—— 桶改回【抓球那一套】:
 *    精对准只修左右(BUCKET_USE_SCHEME_2 也置 0), 深度交给臂的 ID2 补偿
 *    (BOMB_FB_ARM_ENABLE_BUCKET = 1)。理由: 抓球那条路在实车上已经验证可用(能抓到),
 *    而本闭环还没上过车。
 *    ⚠️ 代码全部保留, 想再试: 把本宏与 BUCKET_USE_SCHEME_2 都置 1,
 *       并把 BOMB_FB_ARM_ENABLE_BUCKET 置 0 (三选一)。
 * ===================================================================== */
#define BUCKET_FB_CL_ENABLE        0      /* 0 = 只修左右(★当前, 与抓球同款); 1 = 前后也走闭环 */
#define BUCKET_FB_CL_TOL_PX        45     /* 前后“算够准”的容差(px): 到了就停手。
                                           * 比左右(25)宽松是故意的 —— 深度没那么关键 */
#define BUCKET_FB_CL_MIN_MM        6      /* 前后单步【下限】(mm): 必须 > 死区 ≈4.5mm,
                                           * 否则指令被死区吃掉 = 等于没动还白等一次 */
#define BUCKET_FB_CL_FWD_MAX_MM    6      /* 【朝桶侧】(车头前进)单步上限: 限小, 防顶到桶 */
#define BUCKET_FB_CL_BACK_MAX_MM   9      /* 【离开桶侧】(车头后退)单步上限: 可稍大 */
#define BUCKET_FB_CL_MAX_STEPS     3      /* 兜底②: 前后最多修这么多步 */
#define BUCKET_FB_CL_MAX_TOTAL_MM  30     /* 兜底②: 前后累计位移上限(mm)。与步数上限
                                           * “谁先到算谁的”: 3 步 × 9mm(离开侧上限) = 27,
                                           * 所以 30 基本只兜极端情况(例如一直判成同方向)。
                                           * ⚠️ 后退(离开桶)是安全的, 前进(朝桶)另有单步小上限管着 */
#define BUCKET_FB_CL_FLIP_ENABLE   1      /* 兜底①: 1 = 允许方向自动翻转一次 */
#define BUCKET_FB_CL_FLIP_MIN_PX   15     /* 兜底①: 判定“越修越大”的最小增量(px), 防噪声误翻 */

#if (BUCKET_FB_CL_FWD_MAX_MM < BUCKET_FB_CL_MIN_MM) || (BUCKET_FB_CL_BACK_MAX_MM < BUCKET_FB_CL_MIN_MM)
#error "BUCKET_FB_CL_FWD/BACK_MAX_MM 小于 BUCKET_FB_CL_MIN_MM: 上限比下限还小, 步长会被夹死, 请检查"
#endif
#if BUCKET_FB_CL_MIN_MM < 6
#error "BUCKET_FB_CL_MIN_MM 必须 ≥ 6(底盘到位死区 ≈4.5mm), 否则单步指令会被死区吃掉"
#endif
#if BUCKET_FB_CL_MAX_STEPS < 1
#error "BUCKET_FB_CL_MAX_STEPS 至少为 1, 否则桶的前后永远不修"
#endif
#if BUCKET_FB_CL_ENABLE && BUCKET_AFTER_ALIGN_BACK_ENABLE
#error "桶的深度修正开了两套(BUCKET_FB_CL_ENABLE 与 BUCKET_AFTER_ALIGN_BACK_ENABLE): 请只留一套"
#endif

#if BUCKET_FB_MIN_MM > BUCKET_FB_MAX_MM
#error "BUCKET_FB_MIN_MM 大于 BUCKET_FB_MAX_MM: 前后步长区间反了, 请检查"
#endif

/* =====================================================================
 * ⭐⭐ 视觉方向映射 (关键! 三个阶段“画面左右”对应车体的哪个方向)
 * ---------------------------------------------------------------------
 * 【为什么要管这个】摄像头装在机械臂上, 底座(ID1)转到不同姿态时,
 *   “画面里的左/右” 对应到车体的方向是会变的:
 *
 *   阶段    底座 ID1   相对“球姿态(2052)” 画面横向 = 车体
 *   ------  --------  ------------------  -------------------
 *   球      2052      0°(基准)            左 / 右   (方向正常)
 *   靶      2052      ≈ 0°(与球同向)      —         打靶已改为「只转底座 ID1」
 *   桶      57        ≈ -175°(≈180°)      左 / 右 【镜像】
 *   救援    2052      ≈ 0°(与球同向)      左 / 右   ← ⚠️ 2026-10-07 改版!
 *
 *   (旧救援姿态是 1190 ≈ +84°, 那时“画面左右”= 车的“前后”;
 *    重新标定后基准变成 2052 = 球姿态, 又变回【左右】)
 *
 *   ⇒ 桶的姿态下: 摄像头报“L(画面左)”时, 车要往【右】移;
 *     ⚠️ 救援(方案一: 用底盘对准)时, 画面横向现在对应车体的【左右】——
 *        若改用方案一, 请把下面 s_align_rescue 的 x_is_fb 改成 0;
 *        (当前用方案二: 底盘不动、只转 ID1, 不受此处影响)
 *     (之前“越修越偏”、“一直左右横移接近不了”就是没区分这个)
 *   ⇒ 打靶(2026-10-04 起)不再动底盘: 画面 L/R 用来小步转机械臂底座 ID1,
 *     方向盘子见上面“任务2 打靶参数”里的 TARGET_ID1_LR_SIGN。
 * ---------------------------------------------------------------------
 * 精对准轴向方案开关:
 *   ALIGN_AXIS_SCHEME_1 = 1 (方案一, 【当前使用】):
 *       排爆(球)、打靶: 精对准【只修横向】
 *       救援            : 精对准【只修前后】(此时画面横向就是车的前后)
 *   ALIGN_AXIS_SCHEME_1 = 0 (方案二):
 *       上述阶段的精对准【前后左右都修】(两个轴都参与)
 *   两种方案都保留 K230 的 start_align/D:x,y 精对准, 只是限制“允许修哪个轴”。
 * ⭐ 例外: 桶【不跟这个全局开关】—— 它由 BUCKET_USE_SCHEME_2 单独决定
 *   (当前 = 0, 即桶只修横向, 与老行为一致; 见 s_align_bucket 上面那段说明)。
 * ===================================================================== */
#define ALIGN_AXIS_SCHEME_1     1

/* ================= 排爆任务参数 ================= */
/* ---- 排爆第一步: 按 K230 回传的 C/L/R 做接近补偿 ----
 * 机械臂在 BALL_LOOK 姿态(底座≈232, 与标定基准同向) → 画面左右 = 车体左右,
 * 且这个姿态实测“小车移动的位置非常准确”, 所以方向不动, 只做左右平移补偿:
 *      C = 目标在中间 → 直行靠近 BOMB_C_APPROACH_MM
 *      L = 目标偏左   → 小车左移  BOMB_L_ADJUST_MM
 *      R = 目标偏右   → 小车右移  BOMB_R_ADJUST_MM
 * ⚠️ 距离为 0 时下面代码会“整条指令都不发”(见 BOMB_IDLE 里的注释),
 *    因为 Chassis_Move_*(0) 并不是空操作。
 * ⭐⭐ 2026-10-11(用户要求): 档位距离调大一点 —— 球 10 → 15mm。
 *    ⚠️ 球走的是【普通】平移入口, 到位死区 30 计数(≈4.5mm)会吃掉一截:
 *       实际有效位移 ≈ 本值 − 4.5mm ⇒ 填 10 只走 ≈5.5mm, 填 15 走 ≈10.5mm。
 *       (所以原来"10mm"其实只有 5.5mm 的效果; 这也是这次要调大的原因之一。)
 *    ⚠️ 别一次加太多: 这是"每报一次方向就走一次"的粗对准, 走大了会冲过目标
 *       (历史上用过 120mm, 就是因为冲过头才一路减到 10)。 */
#define BOMB_C_APPROACH_MM          0//15    /* 目标在正前: 直行接近 */
#define BOMB_L_ADJUST_MM            15//120   /* 目标偏左: 左移调整(实际约 10.5mm) */
#define BOMB_R_ADJUST_MM            15//120   /* 目标偏右: 右移调整(实际约 10.5mm) */

/* ---- 抓完小球后盲走回放置区的横向回程(mm): 反向抵消上面的接近补偿 ----
 * (从哪边平移过去抓的, 抓完就平移回哪边; 路径 C 不需要回程) */
#define BOMB_L_RETURN_MM            0//120   /* 目标偏左时: 抓完右移回去 */
#define BOMB_R_RETURN_MM            0//120   /* 目标偏右时: 抓完左移回去 */

/* ---- 排爆第二步: 对准桶(放球) 的接近补偿 ----
 * ⚠️ 机械臂此时已转到 BUCKET_LOOK(底座≈2243, 比球姿态多转≈180°)
 *    → 画面左右相对车体【镜像】, 所以方向要和球阶段反过来:
 *      L(画面左) → 小车【右】移;  R(画面右) → 小车【左】移
 * ⚠️ C(目标已在画面中心) → 【不往前走】。因为摄像头看到桶居中时,
 *    车头已经抵到桶的边缘了, 再往前会过冲/撞桶。
 *    (原来这里用 BUCKET_C_APPROACH_MM 往前走一点, 已按实测删掉)
 * ⭐⭐ 2026-10-11(用户要求): 档位距离调大一点 —— 桶 20 → 25mm。
 *    ⚠️ 桶走的是【小死区】入口(Chassis_Move_*Fine, 1.8mm) ⇒
 *       实际有效位移 ≈ 本值 − 1.8mm ⇒ 填 20 走 ≈18mm, 填 25 走 ≈23mm。
 *    ⚠️ 桶前面就是桶(撞了就麻烦), 别一次加太多; 历史上用过 60, 冲过头才减下来的。 */
#define BUCKET_L_ADJUST_MM          25//60    /* 画面报 L: 小车右移这个距离(实际约 23mm) */
#define BUCKET_R_ADJUST_MM          25//60    /* 画面报 R: 小车左移这个距离(实际约 23mm) */

/* =====================================================================
 * ⭐⭐ 2026-10-11 新增(用户要求): 桶的横向精对准【分两档】—— 大误差走底盘、轻微误差让 ID1 动
 * ---------------------------------------------------------------------
 * 【问题(用户实测)】底盘有【到位死区】(CH_POS_THRESHOLD_COUNT ≈ 4.5mm, 细模式 ≈1.8mm):
 *   轻微横向误差算出来的步长只有几毫米 ⇒ 四轮里"已经在死区带内"的那个轮子【一步都不动】,
 *   另外三个动 ⇒ 车不是在平移而是在原地蹭, 位置越修越歪。
 * 【做法(用户要求)】
 *   ① |需要的横向位移| ≥ BUCKET_LR_CHASSIS_MIN_MM(12mm) → 走【底盘平移】, 而且步长
 *      【不小于它】⇒ 四轮都超死区、一起走, 是真正的整体平移(不会再出现"个别轮子转");
 *   ② 更小的误差(轻微距离) → 【底盘不动】, 改用【底座 ID1】转一点把臂/摄像头对准桶。
 * 【ID1 的安全范围 —— 这条是硬约束】本阶段 ID1 只允许在
 *   BUCKET_LR_ID1_POS_MIN ~ BUCKET_LR_ID1_POS_MAX(0~40 码)之间动:
 *      · 标定值本来就在这个区间(BUCKET_LOOK 的 ID1 = 40, PLACE_PRE/OPEN = 10);
 *      · 每一步都按【绝对位置】限幅(见 Arm_Id1Step 的 pos_min/pos_max),
 *        到边界就停 ⇒ 不可能出现"ID1 转到 4095"这种越界/绕圈事故。
 * 【方向】画面 lr_px>0 = 目标偏画面左; 底盘的处理是"左移"。
 *   ID1 要转的方向取决于底座安装, 先用 BUCKET_LR_ID1_SIGN 给一个, 实测不对会
 *   【自动翻一次】(BUCKET_LR_ID1_AUTOFLIP): 转完误差反而变大 ⇒ 判为方向反了, 取反重来。
 * 【到边界怎么办】ID1 已经顶到 0 或 40 还修不动 ⇒ 自动回退成"底盘平移(用最小步长)",
 *   保证一定能收敛, 不会卡死。
 * 【怎么关】BUCKET_LR_ID1_ENABLE = 0 ⇒ 全部走底盘(老行为, 即"小步蹭"那个)。
 * ===================================================================== */
#define BUCKET_LR_ID1_ENABLE        1      /* 1 = 轻微误差用 ID1 修(★当前); 0 = 全走底盘 */
#define BUCKET_LR_CHASSIS_MIN_MM    12     /* 需要的地盘位移 ≥ 它 → 走底盘(四轮一起走);
                                            * 小于它 → 交给 ID1。必须 > 底盘死区(≈4.5mm),
                                            * 建议 10~15。 */
#define BUCKET_LR_ID1_PX_PER_CODE   1.78f  /* 1 码 ID1 ≈ 多少 px(与救援同一量级; 实测再调) */
#define BUCKET_LR_ID1_SIGN          (+1)   /* lr_px>0(目标偏画面左)时 ID1 的增量符号;
                                            * 实测"越修越大"会由 AUTOFLIP 自动纠正 */
#define BUCKET_LR_ID1_AUTOFLIP      1      /* 1 = 方向反了自动翻一次; 0 = 不翻(只按 SIGN) */
#define BUCKET_LR_ID1_FLIP_MIN_PX   10     /* 判"越修越大"的最小增量(px), 防噪声误翻 */
#define BUCKET_LR_ID1_MAX_STEP      40     /* 单步 ID1 修正量上限(码) —— 硬上限, 别调大 */
#define BUCKET_LR_ID1_POS_MIN       0      /* ⚠️ 本阶段 ID1 允许的【绝对】位置下限(码) */
#define BUCKET_LR_ID1_POS_MAX       40     /* ⚠️ 本阶段 ID1 允许的【绝对】位置上限(码):
                                            *    用户定的安全范围 0~40, 绝不越界 */
#define BUCKET_LR_ID1_MOVE_MS       300    /* ID1 每步转动时间(ms; 这期间主循环等它走完) */

/* ⭐⭐ 2026-10-11(用户要求): 桶的【锁存 ID1】—— 与救援抓取完全同一个做法 -------------
 * 【做什么】在"摄像头对准桶"那一刻(精对准判定成功时)把 ID1 的数值【锁存】下来,
 *   之后【放球动作】的 ID1 就用这个锁存值(而不是姿态表里 PLACE_PRE/OPEN 的标定值),
 *   ⇒ "对准时底座转到哪, 放球时底座就在哪", 前面用 ID1 修的横向误差不会白修。
 *   (与救援的 s_rescue_id1_lock / Arm_GotoRescuePoseKeepId1 是同一套机制, 见那里的注释)
 * 【手动偏移】BUCKET_PLACE_ID1_OFFSET —— 单独一个宏, 需要微调"放球位"时改它即可(当前 0)。
 * 【用法】锁存值 + 手动偏移 后下发; 没锁过(测试模式/超时直接放球)时退回姿态表标定值。
 * ⚠️⚠️ 注意两种做法的差别, 实测放不准时用 BUCKET_PLACE_ID1_USE_LOCK 切:
 *     = 1(★当前, 用户要求): 锁存值【直接】当放球的 ID1(与救援同款)。
 *           注意 BUCKET_LOOK 的 ID1 标定 = 40、PLACE_PRE/OPEN = 10 ⇒ 放球时底座会比
 *           原来多转 30 码(≈2.6°)。如果你实测"放球位置偏了", 说明那 30 码是必要的
 *           姿态差 ⇒ 把本宏改成 0 试试(那时只搬"对准过程产生的修正量", 保留标定差)。
 *     = 0: 放球 ID1 = 姿态表标定值 + (锁存值 − 对准基准) + 手动偏移。
 * 【安全】最后一定夹到 0~4095(而且对准阶段 ID1 本身被限在 BUCKET_LR_ID1_POS_*(0~40),
 *   所以锁存值不可能跑出这个范围) ⇒ 不会出现"ID1 转到 4095"这类越界。 */
#define BUCKET_PLACE_ID1_USE_LOCK   1     /* 1 = 锁存值直接用于放球(救援同款, ★当前) */
#define BUCKET_PLACE_ID1_OFFSET     0     /* 放球 ID1 的【手动偏移】(码): 先给 0,
                                           * 实测放球位偏左/偏右就改这个(正 = 底座往大码方向转) */

/* =====================================================================
 * ⭐⭐ 2026-10-09 新增: 用【Y 轴(前后)像素误差】补偿机械臂 ID2 —— 抓球/放桶共用
 * ---------------------------------------------------------------------
 * 【为什么需要】路线是写死的距离, 每次底盘停在球/桶前的前后位置都不完全一样;
 *   而底盘精对准【只修横向 X】(allow_fb=0), 前后 Y 的误差一直没人管 ⇒ 固定的
 *   BALL_PRE / PLACE_PRE 姿态有时够不到、贴不住(现场就是“放不进去/抓偏”)。
 * 【做法】(参考打靶“分档纠正”, 但更粗、幅度更小)
 *   在【X 轴精对准成功之后、抓取/放置之前】取最后一帧的 Y 误差 fb_px,
 *   按 |fb_px| 分档换算成 ID2 的补偿码, 叠加到抓取/放置姿态的 ID2 上再下发:
 *       抓球: BALL_PRE / BALL_CLOSE
 *       放桶: PLACE_PRE / PLACE_OPEN
 *   ID2 小 = 往前伸(id2 限幅注释: 50 往前 ~ 2300 往后), 所以:
 *       fb_px>0(目标偏远) → 要往前伸 → ID2 减小 → 符号取 -1
 * 【分层(粗纠正, 档位少、幅度小, 方便试调)】
 *       |fb_px| > FAR_PX(100)  → FAR_CODE
 *       |fb_px| > MID_PX(75)   → MID_CODE
 *       |fb_px| > NEAR_PX(40)  → NEAR_CODE
 *       否则(≤40px, 和 X 容差同量级) → 0(不补, 防噪声乱动)
 * 【为什么取 40px 起补】X 容差现在只有 15px, Y 误差 ≤40px 时机械臂本身伸差不了几毫米,
 *   补了反而把姿态抖坏; 只补“明显够不到/贴不住”的大误差。
 * 【方向标定】球/桶两个任务的 SIGN 分开配:
 *   球(BALL_LOOK 底座≈232)与桶(BUCKET_LOOK 底座≈2243)相差 ≈180°, 理论上只有
 *   画面【横向】镜像、纵向不翻转, 所以两者 SIGN 大概率相同; 但为了好单独翻,
 *   留两个宏。实测“越补越偏”就把对应 SIGN 取反, 别动别的。
 * 【时序】fb_px 是在 BOMB_WAIT_FINE_ALIGN / BOMB_WAIT_BUCKET_ALIGN 里
 *   对准成功那一瞬从 s_align_last_fb 拷贝进 s_bomb_fb_* 的; 之后经过
 *   BOMB_SETTLE 的纯计时等待(不读任何 K230 行)再到抓取/放置, 期间值不会变。
 * 【安全】ID2 下发前仍按 s_servo_pos_min/max(50~2300) 限幅, 最大补偿码 30
 *   远小于该范围, 不会把臂甩飞。
 * ===================================================================== */
#define BOMB_FB_ARM_ENABLE      1      /* 总开关: 1=开启, 0=完全关闭(回旧行为) */
#define BOMB_FB_ARM_SERVO       1      /* 补哪个舵机下标: 1=ID2 大臂(往前/往后) */
#define BOMB_FB_ARM_FAR_PX      100    /* |fb_px| 超过它 → 大步 */
#define BOMB_FB_ARM_MID_PX      75     /* |fb_px| 超过它 → 中步 */
#define BOMB_FB_ARM_NEAR_PX     40     /* |fb_px| 超过它 → 小步; 否则不补 */
#define BOMB_FB_ARM_FAR_CODE    30     /* 大步(码) */
#define BOMB_FB_ARM_MID_CODE    20     /* 中步(码) */
#define BOMB_FB_ARM_NEAR_CODE   10     /* 小步(码) */
#define BOMB_FB_ARM_SIGN_BALL   (-1)   /* 抓球: fb_px>0(目标偏远) 时 ID2 增量符号 */
#define BOMB_FB_ARM_SIGN_BUCKET (-1)   /* 放桶: 同上; 若越补越偏就取反 */
/* ⭐⭐ 2026-10-10: 桶的臂深度补偿【单独开关】 -------------------------------
 * 原来球/桶共用 BOMB_FB_ARM_ENABLE, 想只关桶的做不到。
 * ⭐ 用户要求(2026-10-10): 桶【按抓球的模式走】⇒ 现在置 1(开着) ——
 *   桶的深度就由“按 Y 误差补 ID2 里程”来修, 与抓球的 BALL_PRE/BALL_CLOSE
 *   是同一条代码路径(只是姿态换成 PLACE_PRE/PLACE_OPEN), 底盘不参与。
 *   所以桶那边的 BUCKET_USE_SCHEME_2 / BUCKET_FB_CL_ENABLE 都置 0。
 * ⚠️ 想让桶“用底盘前后闭环补”: 本宏置 0, 并把 BUCKET_FB_CL_ENABLE(连同
 *   BUCKET_USE_SCHEME_2)置 1; 想用“底盘分档后退”: 把 BUCKET_AFTER_ALIGN_BACK_ENABLE
 *   置 1 —— 三套【只能开一套】(作用在同一个自由度, 同时开会互相抵消)。
 * ⚠️ 放桶这侧的 SIGN(BOMB_FB_ARM_SIGN_BUCKET)【没在实车标定过】(球的 BALL_PRE 标过):
 *   第一次试车如果发现“越补越偏/顶到桶”, 把 BOMB_FB_ARM_SIGN_BUCKET 取反;
 *   若完全不想要臂补, 本宏置 0 即可(回到“前后完全不管”)。 */
#define BOMB_FB_ARM_ENABLE_BUCKET  1
/* ⭐⭐ 2026-10-10: 这三套参数在代码里是三个常量参数包(见 s_fb_arm_ball / _bucket / _rescue),
 *   由同一套函数 FbArmCompCode()/FbArmCompApply() 使用 ⇒ 球/桶/救援【各调各的, 互不影响】。
 *   救援那一套叫 RESCUE_FB_ARM_*(定义在救援参数区, 数值现在与球/桶相同), 用它的是
 *   救援抓取位/抱紧位(见 Arm_GotoRescuePoseKeepId1 的第二个参数)。 */

/* ================= 救援任务参数 (2026-10-05 重新定义) ================= */
/* ⭐ 救援“对准”方案开关(两套代码都保留, 改一个数就切):
 *   1 = 方案二【当前使用】: 【小车底盘完全不动】, 按 K230 回的 L/R 小步转
 *       机械臂底座 ID1; 收到 C 就认为对准 → 直接抓取。
 *       (适合“摄像头和夹爪装在同一个可转底座上, 转底座就能同时把镜头
 *        和夹爪对准人质”的机械结构)
 *   0 = 方案一: 与排爆球/桶一样 —— L/R 让【小车底盘】平移一小段
 *       (RESCUE_ADJUST_MM; 车头已右转 90°, 所以 R = 右移), 再发 start_align 用 D:x,y 精对准
 *       (只修前后, 见 s_align_rescue)。
 * ⚠️ 实测哪种都不对就换另一个值重新编译, 不用改其它代码。
 * ⚠️ 方向约定(见文件头“视觉方向映射”): 臂在 HOSTAGE_LOOK(底座 = 2052,
 *    与球姿态同向) ⇒ 画面的“左/右”= 车体的【左/右】。
 *    (旧版底座是 1190 ≈ 多转 90°, 那时才是“前后”; 2026-10-07 重新标定后改了)
 *    ⚠️ 2026-10-07: 救援前车头已【右转 90°】, 车体“前后”在场地里 = 车体“左右”:
 *      方案一: L → 小车前进;  R → 小车右移(原来是“后退”);  C → 不动
 *      方案二: L → ID1 朝一个方向转一步; R → 反向;
 *              C → ⭐ 发 start_align 进【D 精对准】(见 RESCUE_USE_FINE_ALIGN),
 *                  再用 D:x,y 的幅值自适应收敛到 ±RESCUE_ID1_ALIGN_TOL 后抓取
 *    ⚠️ 若以后改用【方案一】, 按上面表格它应当是 L→左移 / R→右移,
 *       并把 s_align_rescue 的 x_is_fb 改成 0(该分支当前没用到, 所以没动它)
 */
#define RESCUE_SCHEME_ID1       1

/* ---- ③ 停下等待时间(ms): 右移 + 航向校正好之后先停一会儿,
 *      等车体晃动停下来再让机械臂摆出去, 免得抓的时候还在晃 ---- */
#define RESCUE_STOP_WAIT_MS     3000

/* ---- ③' ⭐ 2026-10-07 新增: 右转 90° 且航向校准【到位】之后, 先原地停稳这么久
 *      才允许右移进救援区。
 * 为什么需要: 车头刚转到 -90° 上下时, 转向环还在收尾(角速度残余)、车身也可能
 *   因为原地转向的惯性轻微摆动。这时立刻横移会有两个后果:
 *     ① 横移的“航向保持”会拿这个残余角当基准去纠 → 车走成弧线;
 *     ② 麦轮横移本身会“扭”车头, 残余角速度一叠加偏得更厉害。
 * 做法(⭐顺序很重要): 先让航向校准真的到位(Chassis_Task_Is_Complete),
 *   再原地停车开始计时; 计时到点才右移 —— 即“先校准到位, 后额外停稳 3 秒”。
 *   ⚠️ 不是“最多等 3 秒”(那样没等它真到位就跑了)。
 * 建议 2000~4000(默认 3000); 0 = 关闭(校准完立刻右移)。 */
#define RESCUE_ALIGN_SETTLE_MS  3000

/* ---- 【方案二】转 ID1 专用参数(和打靶那套完全同构) ---- */
/* ⭐⭐ 2026-10-08 重新示教得到的【三个对准/抓取位置】(用户实测) ------------
 *      目标居中 = 2059      目标偏左 = 1561      目标偏右 = 2503
 *      (与姿态表 HOSTAGE_GRAB_M / _L / _R 的第 1 列一致)
 * 相对中间位置的【有符号偏移】(向左为负, 与 RESCUE_ID1_LR_SIGN = -1 一致):
 *      左 = 1561 - 2059 = -498 码          右 = 2503 - 2059 = +444 码
 * 现状: 这三个位置【都不跨 0/4095 圈边界】(旧的“过零”位 3827 已不存在),
 *   所以 LEFT_OFF 不再需要 -4096 的绕圈换算; Arm_Id1Step 的“逻辑位置 + 取模
 *   下发”机制保留不动, 只是眼下用不到绕圈而已。
 * ⚠️ 下面这几个值必须和姿态表里 HOSTAGE_GRAB_{L,M,R} 的 ID1 一致(重新示教后一起改)。
 *    对准的【基准(偏移 0 点)】取的是 HOSTAGE_LOOK 那一行的 ID1(现在 2052),
 *    与本宏 MID_POS(2059)相差 7 码 ≈ 0.6° —— 同一物理方向的两次示教之差,
 *    对限幅和“左/中/右”选侧别都没有影响。 */
#define RESCUE_ID1_MID_POS      2059    /* 目标居中时的 ID1 (= HOSTAGE_GRAB_M 那一行) */
#define RESCUE_ID1_LEFT_POS     1561    /* 目标偏左、对准完成时的 ID1 */
#define RESCUE_ID1_RIGHT_POS    2503    /* 目标偏右、对准完成时的 ID1 */
#define RESCUE_ID1_LEFT_OFF     (RESCUE_ID1_LEFT_POS - RESCUE_ID1_MID_POS)  /* -498 */
#define RESCUE_ID1_RIGHT_OFF    (RESCUE_ID1_RIGHT_POS - RESCUE_ID1_MID_POS) /* +444 */
#define RESCUE_ID1_OFF_MARGIN   80      /* 限幅余量(码): 允许略微超过示教范围, 防抖动卡死 */

#define RESCUE_ID1_STEP         60      /* 每收到一次 L/R, 底座 ID1 转多少角度码
                                         * (4096 码 = 360°, 60 码 ≈ 5.3°)。
                                         * 建议 30~120。从中间到最左约 8 步(-498/60)、
                                         * 到最右约 7 步(+444/60), 都在 STEP_MAX 之内 */
/* ⭐⭐ 救援精调(D:x,y)的【自适应步长】 (2026-10-07 新增, 与打靶同构) ------
 * 为什么: 收到 D:x,y 时如果一律用 RESCUE_ID1_STEP(60 码) 去修, 一步可能改掉
 *   上百像素, 而判定窗口只有 ±RESCUE_ID1_ALIGN_TOL(25px) ⇒ 会在窗口两边
 *   来回摆(打靶当初就是这个毛病, 见 TARGET_ID1_STEP_FINE 处的注释)。
 * 分档(误差大走大步求快, 误差小走小步求稳):
 *      |x| > RESCUE_ID1_STEP_BIG_PX(200) → RESCUE_ID1_STEP(60 码)
 *      |x| > RESCUE_ID1_STEP_MID_PX(60)  → RESCUE_ID1_STEP/2(30 码)
 *      否则                            → RESCUE_ID1_STEP_FINE(10 码)
 * ⚠️ 硬约束: 小步换算出的 px 必须 < 窗口半宽(25px), 否则永远跨过中心来回摆。
 *      按打靶实测的 1 码 ≈ 1.78px 估: 10 码 ≈ 18px < 25 ✓ (窗口 20px 时是刚好贴边)
 * ⭐ 怎么定准: 看日志 [救援][ID1步长标定] 那行“上一步 ID1 转 X 码, |x| a->b”:
 *      一步改的 px: < 10 → 太小(加大 FINE); 15~25 → 正合适; > 30 → 太大(必须减)。
 * 不想分档就把 FINE 改成和 RESCUE_ID1_STEP 一样即可。 */
#define RESCUE_ID1_STEP_BIG_PX   200    /* 超过这么多 px 才用大步(= RESCUE_ID1_STEP) */
#define RESCUE_ID1_STEP_MID_PX   60     /* 超过这么多 px 用中步(= RESCUE_ID1_STEP/2) */
#define RESCUE_ID1_STEP_FINE     10     /* 小步(误差小时用, 决定能否收敛进 ±25px)。
                                         * ⭐ 2026-10-07: 15 → 10, 是跟着
                                         *    RESCUE_ID1_ALIGN_TOL(30 → 20) 一起改的:
                                         *    按 1 码 ≈ 1.78px 估, 15 码 ≈ 27px > 窗口半宽
                                         *    ⇒ 会跨过中心来回摆、永远收敛不了。
                                         * ⭐ 2026-10-08: 窗口回到 25px, 10 码(≈18px)仍够用。
                                         * ⚠️ 若日志 [救援][ID1步长标定] 显示 10 码改的 px
                                         *    仍 ≥ 25, 就继续减(8 → 6)。 */
#define RESCUE_ID1_LR_SIGN    (-1)      /* “收到 L”时 ID1 的增量符号:
                                         *  -1 = 数值减小 = 往【左】(与 RESCUE_ID1_LEFT_OFF
                                         *       的符号一致) ★当前实测值
                                         *  +1 = 数值增大(装反了才用) */
#define RESCUE_ID1_MOVE_MS      300     /* ID1 每步转动时间(ms);
                                         * 这期间把 K230 旧帧全丢掉 */
/* ⚠️⚠️ 这两个是【相对基准姿态 HOSTAGE_LOOK 的偏移】(可为负), 不是 0~4095 的
 *      绝对位置。实际限幅 = 基准 + 下面这个值:
 *          下限 = 2052 + (-578) = 1474
 *          上限 = 2052 + (+524) = 2576
 *      ⇒ 正好把【左 1561 / 右 2503】两个对准位各留出约 80 码(≈7°)余量。
 *      (调用处写成 base + RESCUE_ID1_POS_MIN / MAX, 见 Rescue_Id1Step/Rescue_Id1Goto) */
#define RESCUE_ID1_POS_MIN      (RESCUE_ID1_LEFT_OFF  - RESCUE_ID1_OFF_MARGIN)  /* -578 */
#define RESCUE_ID1_POS_MAX      (RESCUE_ID1_RIGHT_OFF + RESCUE_ID1_OFF_MARGIN)  /* +524 */
#define RESCUE_ID1_STEP_MAX     20      /* 🛡防卡死①: 最多转这么多步就强制认为对准
                                         * (到左极限需 498/60 ≈ 8 步, 20 步够用) */
#define RESCUE_ID1_TIMEOUT_MS   20000   /* 🛡防卡死②: 整个 L/R 环节最长等这么久(ms),
                                         * 超时强制去抓取(含 K230 完全不回应) */

/* ---- 【方案一】动底盘专用: L/R 时车前进 / 右移的距离(mm) ---- */
#define RESCUE_ADJUST_MM        15      /* 建议 20~80 */

/* =====================================================================
 * ⭐⭐ 救援“搜目标”: ID1 慢速巡视 (2026-10-06 新增)
 * ---------------------------------------------------------------------
 * 【为什么】救援时底盘【完全不动】, 只有底座 ID1 能动。如果人质一开始不在
 *   画面里, 光发 run_task 干等可能一直等不到 → 最后只能超时盲抓。
 *   先让 ID1 把周围扫一遍, 命中的概率大很多。
 * 【巡视顺序】(相对 HOSTAGE_LOOK 的 ID1 偏移) —— ⭐ 2026-10-07 改成 5 个点(用户要求):
 *       中间(0) → 右(+444) → 中间(0) → 左(-498) → 中间(0)
 *   每扫完一侧都回中间一次: ① 中间是基准, 回去后相机横向参考最正;
 *   ② 避免从最右一步甩到最左(920 码)造成画面拖影; ③ 最后停在中间,
 *   所以扫完一圈后 s_id1_offset 正好回 0。
 *   每个点位: 先用 RESCUE_SWEEP_MOVE_MS 慢慢转过去, 再停 RESCUE_SWEEP_HOLD_MS
 *   给 K230 拍照识别。全程【只动 ID1】(掩码 SERVO_MASK_ID1), ID2~ID5 不动。
 * 【中止条件】巡视期间 K230 要【连续 2 帧报同一个 token】(见
 *   VISION_STABLE_FRAMES_SWEEP)才算“真看到目标”, 随即中止巡视, 把那一行原样
 *   交给正常流程处理(单帧误检不中止, 免得白转一圈):
 *       C        → 已对准 → 停稳后抓取
 *       L / R    → 按原来的小步转 ID1 逻辑继续对准(和以前完全一样)
 *       D:<x>,<y>→ 认为“已发现目标”, 用 x 小步把 ID1 修到中心
 *                  (|x| < RESCUE_ID1_ALIGN_TOL 就算对准) → 抓取
 *   对准完成后统一走【示教好的抓取姿态】(侧别由 ID1 累计偏移量决定)。
 * 【扫完一圈没回应】回到中间, 继续原来的“发 run_task:4 等 C/L/R”。
 * 【调参】视野不够 → 把 RESCUE_ID1_LEFT_OFF / RIGHT_OFF 往两边再加大一点;
 *         嫌慢 → 减 MOVE/HOLD; 不要这个功能 → RESCUE_SWEEP_ENABLE = 0
 * ⭐ 2026-10-07: 巡视点位不再用固定 ±RESCUE_SWEEP_ANGLE, 而是直接用
 *    【实测的两个对准极限】(左 1561 / 右 2503) —— 也就是把人质可能出现的
 *    整段范围都扫一遍, 命中率最高(见下面 s_rescue_sweep_code)。
 * ⭐⭐ 2026-10-08: 重新【打开】巡视(= 1) —— 用户要求摆到识别人质姿态后能看到
 *    “中 → 一侧 → 中 → 另一侧 → 中”的慢速搜索过程, 期间 K230 一有稳定回应就
 *    中止巡视、转入下面的对准流程。
 *    ⚠️ 巡视是阻塞的, 一圈约 5×(MOVE+HOLD) ≈ 11.5s。嫌慢 → 减 MOVE/HOLD;
 *       不要这个功能 → 把本行改回 0(那时 K230 完全不回应就只能等
 *       RESCUE_ID1_TIMEOUT_MS(20s) 超时盲抓)。
 *    (2026-10-07 曾按用户要求关成 0 以省下这 ~11.5s; 现在恢复。)
 * ⭐⭐ 2026-10-10 新增(用户要求): 【一圈没回应就再巡视一圈】—— 原来只扫一圈,
 *    扫完没回应就干等 K230(等不到只能 20s 超时盲抓); 现在扫完一圈没回应会
 *    自动再扫一圈, 直到收到回应或到达下面的圈数上限。
 *    RESCUE_SWEEP_MAX_ROUNDS = 最多巡视几圈:
 *      1 = 老行为(只扫一圈);  2 = 扫两圈;  0 = **不限圈数**(一直扫到有回应 ——
 *      ⚠️ K230 一直不回应就会一直转, 比赛里一般别用 0)。
 *    时间开销 = 圈数 × (5×(MOVE+HOLD) ≈ 11.5s); 每圈结束都会把“对准计时”重新计,
 *    所以不会因为巡视而提前触发 20s 盲抓兜底。
 *    扫完上限仍未回应 → 回到原来的“发 run_task:4 等 C/L/R”流程(照旧有 20s 超时兜底)。 */
#define RESCUE_SWEEP_ENABLE     1
#define RESCUE_SWEEP_MAX_ROUNDS 2      /* 最多巡视 2 圈(1=老行为; 0=不限, 慎用) */
#define RESCUE_SWEEP_MOVE_MS    1500    /* 挪到下一个角度的转动时间(值大 = 慢 = 画面不糊) */
#define RESCUE_SWEEP_HOLD_MS    800     /* 每个角度停多久给 K230 识别 */

/* ⭐⭐ 2026-10-07 新增: 救援也走【D 精对准】(收到 C 之后再发 start_align)。
 * 【为什么要加 —— “机械臂识别时一直转圈”的根因】
 *   接近阶段 K230 只回 C/L/R, 【没有幅值】, 所以粗对准只能固定一步 60 码
 *   (RESCUE_ID1_STEP ≈ 5.3° ≈ 107px)。而 K230 判“居中”的窗口只有 ±40px
 *   (2026-10-07 起; 原来是 ±50px)
 *   ⇒ 一步就跨过整个窗口 → 它报反方向 → 再跨回来 → 一直来回摆,
 *     直到 20 步 / 20s 兜底才盲抓(用户看到的就是“一直转圈”)。
 *   发了 start_align 之后 K230 进 ALIGN、回 D:<x>,<y>(带幅值), 就能用
 *   Rescue_Id1StepFor() 的【自适应步长】收敛(误差大走大步、小走小步)。
 * 1 = 开启(默认); 0 = 关闭(回到“收到 C 就停稳抓取”的老行为)。
 * ⚠️ 兜底(不会卡死): 发完 start_align 后
 *    ① K230 一直不回任何数据(K230 端该任务没实现 ALIGN) → 等
 *       VISION_RESPONSE_TIMEOUT_MS 就按“已对准”去抓;
 *    ② 它还在回 C(仍在接近模式) → 收到 C 也照样去抓。 */
#define RESCUE_USE_FINE_ALIGN   1

/* 收到 D:x,y 时, |x| < 它就算对准(像素容差)。
 * ⚠️⚠️ 必须与 RESCUE_ID1_STEP_FINE 配套(硬约束): 小步换算出的 px 必须 < 本窗口
 *    的半宽, 否则永远跨过中心来回摆 —— 原来 30 配 FINE=15(≈27px < 30 ✓);
 *    收到 20 时 FINE=10(≈18px < 20 ✓)。
 * ⚠️ 还应当 ≤ K230 那边本任务的窗口(ALIGN_TOL_PIX): 本机比它大就会“提前判准”
 *    (误差落在两者之间时本机不再转、K230 不发 OK)。
 * ⭐ 2026-10-08: 20 → 25, 与 ALIGN_TOLERANCE 和 K230 的 ALIGN_TOL_PIX 一起统一
 *    成 25px —— 20px 时它和底盘/舵机的实际分辨率贴得太近, 容易靠超时兜底。
 * ⚠️ 精度还有一层上限: K230 在 ALIGN 里判成功(回 OK)用的是【它自己的】容差,
 *    它判成功后就停发误差。 */
#define RESCUE_ID1_ALIGN_TOL    25

/* ---- ⭐ 救援“抓取位三选一” (2026-10-06 新增) -------------------------
 * 背景: 救援时底盘【完全不动】(HOSTAGE_LOOK 姿态下, 车和臂正好在三个目标
 *   物的中间), 靠底座 ID1 小步转对准人质。人质偏哪边, ID1 就往哪边多转一点,
 *   而夹爪的【抓取姿态】如果只有一套, 转得多了就抱不准/抱不到。
 * 做法: 按 ID1 的【累计偏移量】选一侧(左/中/右), 然后每侧各有
 *   【抓取 / 抱紧 / 抬起】三个姿态, 共 9 个(姿态本体在 s_arm_pose_table 的
 *   HOSTAGE_GRAB_{L,M,R}[_CLOSE|_LIFT], 用示教模式挨个标定):
 *     |偏移| ≤ RESCUE_GRAB_MID_RANGE            → 用中间那 3 个
 *     偏移与 RESCUE_GRAB_LEFT_SIGN 同号且超范围 → 用左边那 3 个
 *     否则                                      → 用右边那 3 个
 * 调参:
 *   ① 多大算“居中”  → 改 RESCUE_GRAB_MID_RANGE (★ 已按实测示教值标定为 220)
 *
 * ⭐⭐ 2026-10-06 按实测示教值标定 RESCUE_GRAB_MID_RANGE -----------------
 *   用户示教量出的三个抓取姿态(ID1~ID5):
 *       中间抓取  1201, 754,1626,1669, 120
 *       左边抓取   791, 390,1889,1979, 121
 *       右边抓取  1582, 515,1791,1909, 121
 *   ⇒ 三个姿态的 ID1 分别是 791 / 1201 / 1582, 以【中间】为原点:
 *        左抓取位 = 1201 - 410 = 791   (偏移 -410 码对应“左”)
 *        右抓取位 = 1201 + 381 = 1582  (偏移 +381 码对应“右”)
 *   ⇒ “离哪个姿态近就用哪个”的分界点:
 *        左/中 分界 = (0 + 410)/2 = 205 码
 *        中/右 分界 = (0 + 381)/2 = 190 码
 *      两边差不多, 统一取 200(误差 ±10 码 ≈ 0.9°, 对抓取没影响)。
 *   ⚠️ 原来的 60 是【错的】: RESCUE_ID1_STEP=60, 所以 ID1 只要转过 1 步
 *      (偏移 60 码)就会去用“按 -410 码标定的左边抓取位” —— 姿态差了 350 码
 *      (≈31°), 夹爪根本够不到人质。
 *   ⚠️ 以后重新示教、三个姿态的 ID1 变了, 按同样方法重算本值即可:
 *      取【两个相邻抓取位 ID1 差值的一半】中较小的那个。
 *
 * ⭐⭐ 2026-10-08 重新示教后重算(用上面那三个新位置) ----------------------
 *   三个对准/抓取位置: 中间 2059 / 左边 1561 / 右边 2503
 *     ⇒ 相邻差值: 中-左 = 498 码;  右-中 = 444 码
 *     ⇒ 左/中 分界 = 498/2 = 249 码;  中/右 分界 = 444/2 = 222 码
 *     ⇒ 取较小的 222 → 取 220(误差 ±2 码 ≈ 0.2°, 对抓取没影响)
 *   ⚠️ 必须取两者中【较小】的那个: 取大了会把本该用左/右姿态的偏移误判成
 *      “居中”, 夹爪就差一截够不到人质。
 *   ② 左右用反了    → 只改 RESCUE_GRAB_LEFT_SIGN 的符号, 别动别的 */
#define RESCUE_GRAB_MID_RANGE   220     /* ±角度码: |ID1 偏移| ≤ 它 → 用中间抓取位
                                         * (由三个示教抓取位的 ID1 差值取半得出) */

/* ⭐ 偏移超过这么多码 ⇒ 已超出所有示教抓取位的覆盖范围, 只提示不改变行为。
 * 依据(2026-10-08 重新示教): 左右抓取位分别落在 -498 / +444 码处,
 *   再多就都是“最远姿态”硬凑了。 */
#define RESCUE_GRAB_FAR_WARN    500
#define RESCUE_GRAB_LEFT_SIGN (-1)      /* 偏移与它同号 ⇒ 人质偏“左” ⇒ 用左抓取位。
                                         * 默认与 RESCUE_ID1_LR_SIGN 同号(-1)。
                                         * ⚠️ 2026-10-08 重新示教后左极限仍是【负偏移】
                                         *    (左边 1561 < 中间 2059 ⇒ 偏移 -498),
                                         *    所以这里仍然是 -1, 不用改。 */

/* ⭐⭐ 2026-10-09 新增 / 2026-10-10 改为【按侧别分开】: 救援【抓取时】给底座 ID1 的
 *    “移动偏移”(舵机码) -------------------------------------------------------
 * 【它修的是什么】救援是靠摄像头带动底座 ID1 一点点转、把人质拖到画面中心, 对准
 *   完成那一刻的 ID1 会被【锁存】(s_rescue_id1_lock), 抓取三连
 *   (该侧抓取位 → 该侧抱紧位 → 该侧抬起位)全程都锁着这个值不动
 *   (见 Arm_GotoRescuePoseKeepId1)。
 *   但“摄像头对准的中心”和“夹爪真正该在的位置”实测有偏差 —— 相机与夹爪不共轴、
 *   对准本身有容差(±RESCUE_ID1_ALIGN_TOL 像素)、机构回程间隙/背隙……
 *   ⇒ 夹爪会差几十码。本组宏就是给锁存值加的那个修正量:
 *        实际下发的 ID1 = 锁存值 + 残余补偿 + 本组偏移(按左/中/右 选一个)
 * 【⭐ 2026-10-10(用户要求): 按侧别分开取值】
 *      抓【左边】物体 → RESCUE_GRAB_ID1_OFFSET_L = +30
 *      抓【右边】物体 → RESCUE_GRAB_ID1_OFFSET_R = -30
 *      抓【中间】物体 → RESCUE_GRAB_ID1_OFFSET_M = -20(保持之前那个值)
 *   侧别由【锁存值自己】判定(归一化 → Rescue_GrabSideFor), 所以不依赖调用顺序;
 *   左右分界 = RESCUE_GRAB_MID_RANGE(±220 码)。想改哪一侧只动对应那一个宏。
 * 【符号】正 = 往【右】修(数值增大; 与日志里“归一化偏移 负=左/正=右”同向)
 *        负 = 往左修;   0 = 不修
 * 【量纲/怎么调】舵机码, 4096 码 = 一圈 ⇒ 60 码 ≈ 5.3°(≈107px, 按实测 1 码 ≈ 1.78px)。
 *   30 码 ≈ 2.6°(≈53px)。实车抓空/抓偏时估个差值, 每次 ±10~20 码 试。
 * 【不影响什么】
 *   ⚠️ 不参与选侧别(左/中/右 仍由【锁存值本身】判断, 见 Rescue_GrabSideFor):
 *      本组宏只把选中那套姿态的 ID1 平移这一点点, 调它不会让侧别跳变。
 *   ⚠️ 只作用于【锁存值】这条路: 没锁存过(退回姿态表里的 ID1, 例如无视觉测试/未对准)
 *      时【不加】—— 那种情况下抓取位是示教标定出来的, 本来就是对的。
 *   ⚠️ 不回写姿态表: 只是运行时修正, 重上电/再跑一次救援仍从本组宏取。 */
#define RESCUE_GRAB_ID1_OFFSET_L   (0)  /* 抓【左】边物体: ID1 偏移(正 = 往右修) */
#define RESCUE_GRAB_ID1_OFFSET_M   (0)  /* 抓【中】间物体: ID1 偏移(沿用原 RESCUE_GRAB_ID1_OFFSET) */
#define RESCUE_GRAB_ID1_OFFSET_R   (0)  /* 抓【右】边物体: ID1 偏移(负 = 往左修) */

/* ⭐⭐ 2026-10-10 新增(用户要求): 抓取前先用【最后一次 D 帧的横向残余】自动补一点 ----
 * 【为什么要】救援对准的收敛判据是 |x| < RESCUE_ID1_ALIGN_TOL(25px) —— 也就是说
 *   只要“够近”就停, 最后停下时目标可能还偏在容差内的任意位置(±25px ≈ ±14 码),
 *   抓取就按这个“停在哪算哪”的角度去抓 ⇒ 夹爪会差一点(实测“ID1 看着还会偏”)。
 *   本机制把这个【已知的残余】补掉: 抓取用的 ID1 = 锁存值 + 本次残余换算的码 + 手动偏移,
 *   于是对准到哪就补到哪, 残余那一截不再随机摆在抓取位上。
 * 开关: RESCUE_GRAB_USE_RESIDUAL = 1(开, 默认) / 0(关, 只用下面的手动偏移)。
 * 残余从哪来: 救援方案二【自己解析 D 帧】, 每收一帧都会记 s_rescue_last_dx(px)
 *   与“本轮到底有没有读到过 D”的 s_rescue_d_valid —— 后者为 0(K230 只回 OK/C,
 *   没给过坐标)时【不补】, 免得拿上一轮/上一个任务的旧残余乱补。
 *   (⭐ 2026-10-10 修正: 原来读的是 Vision_FineAlignProcess 记的 s_align_last_lr,
 *    而救援方案二根本不走那个函数 ⇒ 拿到的是【桶那一步】留下的旧值。)
 * ⚠️ 同一帧的 y(前后误差)另外用于“补 ID2 里程”, 见下面的 RESCUE_FB_ARM_* 那一段。
 * ⚠️ px→码 靠下面的标定值换算, 太离谱的残余(例如对准其实超时失败了)会被
 *   RESCUE_GRAB_RESIDUAL_MAX_CODES 限幅, 防止底座被甩一大截。
 * ⚠️ 方向: 与对准时同一套(D 的 x>0 = 目标偏画面左 ⇒ 按 RESCUE_ID1_LR_SIGN 往左修),
 *   即 residual_codes = x / RESCUE_GRAB_PX_PER_CODE × RESCUE_ID1_LR_SIGN。
 * ⚠️ 与手动偏移的关系: 两个都加上(先自动补残余, 再用手动偏移兜住“固定偏差”),
 *   手动那个只用来修【每次都不一样之外】的固定偏(相机与夹爪不共轴等)。 */
#define RESCUE_GRAB_USE_RESIDUAL        1       /* 1 = 自动补残余 / 0 = 关掉 */
#define RESCUE_GRAB_RESIDUAL_MAX_CODES  20      /* 残余补偿的限幅(码): ±20 码 ≈ ±36px ≈ 1.8°。
                                                 * 对准成功时 |x|≤25px ⇒ 正常补 ≤14 码;
                                                 * 超过就说明“其实没对准”, 别硬补。 */
/* 像素↔舵机码 的标定值(与打靶/救援精调步长同一来源): 实测 1 码 ≈ 1.78px。
 * ⚠️ 怎么校准: 发一次固定码数的 ID1 转动(如 60 码), 看 K230 那边 |x| 变了多少 px
 *    ⇒ px_per_code = Δx / 码数。填不准只会“补多/补少”, 不会乱方向。 */
#define RESCUE_GRAB_PX_PER_CODE         1.78f

/* ⭐⭐ 2026-10-10 新增(用户要求): 救援抓取【也】按 K230 回的 Y 轴(前后)像素误差
 *   修改机械臂“里程”—— 即把抓取姿态的 ID2(大臂)伸出/收回一点, 机制与
 *   抓球/放桶【完全相同】(见文件上方 BOMB_FB_ARM_* 那段说明), 只是一套自己的参数:
 *     ① 对准成功那一刻记下最后一次 D 帧的 y(前后误差) → s_rescue_fb_px;
 *     ② 摆【该侧抓取位】和【该侧抱紧位】时, 按 |y| 分档把补偿码叠到 ID2 上再下发
 *        (抬起位/回程姿态【不补】—— 与抓球/放桶一致: 只补“伸出去抓”的那两下)。
 * 【为什么需要】救援精对准只收敛到 |x| < RESCUE_ID1_ALIGN_TOL(25px)(ID1 那个轴),
 *   而【前后(Y)方向根本没人修】(方案二底盘完全不动) ⇒ 车离人质多远就是多远,
 *   夹爪伸出去的“里程”是死的。用 Y 误差把里程补一点, 抓空/顶到人质的概率会小很多。
 * 【方向】sign 的约定与球/桶一致: y > 0(目标偏远) 时要【往前伸】⇒ ID2 减小 ⇒ sign = -1。
 *   ⚠️ 实测越补越偏(夹爪离人质更远)就把本宏取反成 +1。
 * 【分档】与球/桶同一套数值(40/75/100 px → 10/20/30 码), 想单独调就改下面这几个,
 *   不会影响球/桶(它们用 BOMB_FB_ARM_*)。
 * 【不影响什么】本机制只改 ID2 一列, 不改 ID1(锁存值)/ID3/ID4/ID5;
 *   没读到过 D 帧(K230 只回 OK/C)时【不补】(与球/桶同一保护)。
 * ⚠️ 姿态表本身不动: 补偿是“下发前临时叠加”, 重上电/改回 0 都回到标定值。 */
#define RESCUE_FB_ARM_ENABLE      1      /* 总开关: 1 = 开启, 0 = 完全关闭(回旧行为) */
#define RESCUE_FB_ARM_SERVO       1      /* 补哪个舵机下标: 1 = ID2 大臂(往前伸/往后收) */
#define RESCUE_FB_ARM_SIGN    (-1)       /* y>0(目标偏远) 时 ID2 增量的符号: -1 = 减小(往前伸) */
#define RESCUE_FB_ARM_FAR_PX      100    /* |y| 超过它 → 大步 */
#define RESCUE_FB_ARM_MID_PX      75     /* |y| 超过它 → 中步 */
#define RESCUE_FB_ARM_NEAR_PX     40     /* |y| 超过它 → 小步; 否则不补 */
#define RESCUE_FB_ARM_FAR_CODE    30     /* 大步(码) */
#define RESCUE_FB_ARM_MID_CODE    20     /* 中步(码) */
#define RESCUE_FB_ARM_NEAR_CODE   10     /* 小步(码) */

/* ---- K230 run_task 编号(与 yolo_main3.py handle_command 对齐) ---- */
#define K230_TASK_BALL      1   /* 球: 抓取小球(排爆第一步) */
#define K230_TASK_TARGET    2   /* 靶: 打靶 */
#define K230_TASK_BUCKET    3   /* 桶: 放球(排爆第二步, 抓完球后对准桶放置) */
#define K230_TASK_RESCUE    4   /* ⚠️ 救援=形状(圆柱/腰鼓/圆台): K230 端形状追踪尚未实现,
                                 *    待 K230 加上形状状态并约定任务号后改此值 */

/* ================= 任务2 打靶参数 =================
 * ⭐ 2026-10-04 改版: 打靶时【小车底盘完全不动】, 只转机械臂底座 ID1 对准。
 *    旧实现是“画面 L/R → 小车左右横移 + D:x,y 精对准”, 已废弃。
 *    完整流程见 Handle_Vision_Alignment() 里“任务2”那段注释。
 * ---------------------------------------------------------------------
 * 方向约定(实测确认):
 *      K230 回 L → ID1 数值【减小】(即“向左”)
 *      K230 回 R → ID1 数值【增大】(向右)
 *      K230 回 C → 已对准 → 依次摆 TARGET_FIRE / TARGET_LIFT / SCAN_RESET
 * ⚠️ 若实测转反了, 只改 TARGET_ID1_LR_SIGN 的符号即可, 别动别的。 */
/* ⭐⭐ 横向(ID1)自适应步长 —— 2026-10-06 实测定版
 * 实测换算: 60 码 ≈ 107px(日志: x=-57px 转 +60 码后 下一帧变成 x=+50px)
 *          ⇒ 1 码 ≈ 1.78px
 * ⚠️ 为什么必须自适应: 60 码一步就能改 107px, 而判定窗口只有 ±TARGET_ALIGN_TOLERANCE
 *    (窗口先收到 ±20px, 2026-10-08 又跟着 K230 收到 ±11px; 本机用 10px) ——
 *    一律用 60 码会在窗口两边来回摆(旧日志里 393/333 反复 7 次就是这个原因),
 *    根本收敛不了。
 * 分档(误差大走大步求快, 误差小走小步求稳):
 *    |x| > TARGET_ID1_STEP_BIG_PX(200) → TARGET_ID1_STEP(60 码 ≈ 107px)
 *    |x| > TARGET_ID1_STEP_MID_PX(50)  → 一半(30 码 ≈ 53px)
 *    否则                              → TARGET_ID1_STEP_FINE(5 码 ≈ 9px)
 * ⚠️ 硬约束: TARGET_ID1_STEP_FINE 换算出的 px 必须 < 窗口半宽
 *    (窗口已收紧到 TARGET_ALIGN_TOLERANCE=10px ⇒ FINE 必须 ≤ 9px)
 *    ⭐ 2026-10-08: 10 码(≈18px) 是按"窗口 20px"定的; 窗口改 10px 后
 *    18px 一步就会跨过中心来回摆 ⇒ FINE 同步改成 5 码(≈9px)。
 *    想再稳就继续减 FINE, 想再快就加大 BIG/MID 两档。 */
#define TARGET_ID1_STEP_BIG_PX   200    /* 超过这么多 px 才用大步 */
#define TARGET_ID1_STEP_MID_PX   50     /* 超过这么多 px 用中步(原来 100, 随窗口收紧改小) */
#define TARGET_ID1_STEP_FINE     5      /* 小步(≈9px < 10px, 一定落进容差) */

/* =====================================================================
 * ⭐⭐ 打靶专用判定容差 (2026-10-06 新增; 2026-10-08 跟着 K230 再收紧)
 * ---------------------------------------------------------------------
 * K230(yolo_main.py) 判"打靶对准成功"用的是它自己的窗口
 * `TARGET_ALIGN_TOL_PIX`(现在 = 11px): |dx| 和 |dy| 都 < 11 才回 OK/FIRE。
 * ⚠️⚠️ 硬约束: 本机这个值【必须 ≤ K230 的窗口】。
 *    本机是靠"|误差| ≥ 容差就再动一步"推进的, 本机容差比 K230 大 ⇒
 *    误差落在两者之间时(例如 y=15px)本机认为"够准了、不再动舵机",
 *    而 K230 认为没对准、一直发 D 帧 ⇒ 两边干等到它的 12s 自超时
 *    (它才发 OK+FIRE, 激光带着这十几像素的偏差打出去)。
 *    ⭐ 2026-10-08 实测就是"打靶 ID4 误差 15 时纠正不了" —— 原来本机是 20px。
 * ⚠️ 容差收紧后【步长也必须跟着变小】, 否则一步就跨过中心来回摆:
 *    窗口总宽现在只有 20px ⇒ 已同步把 TARGET_ID1_STEP_FINE 减到 5 码(≈9px)、
 *    并给 ID4 加了 TARGET_ID4_STEP_FINE(15 码)。
 * ===================================================================== */
#define TARGET_ALIGN_TOLERANCE  10      /* 打靶: |dx|,|dy| 都 < 它才算对准
                                         * ⚠️ 必须 ≤ K230 的 TARGET_ALIGN_TOL_PIX(最新 = 11px),
                                         *    否则本机先"认为够准了"不再动舵机 → 两边干等到
                                         *    K230 的 ALIGN_SELF_TIMEOUT(14s) 才(盲)射一枪。
                                         * ⚠️ 本宏与 TARGET_STOP_NUDGE_TOL_PX 要取【同一个值】:
                                         *    "冻结"判据是 < 本宏, "该动一步"判据是 ≥ 本宏,
                                         *    两者互补时才不会有"既不动也判不过"的夹缝。 */

#define TARGET_ID1_STEP         80/*60*/      /* 每收到一次 L/R, 底座 ID1 转多少角度码。
                                         * 4096 码 = 360°, 所以 1° ≈ 11.4 码,
                                         * 60 码 ≈ 5.3°。建议 30~120:
                                         *   太大 → 一步冲过头、来回摆;
                                         *   太小 → 对准慢、步数多
                                         * ⚠️ 粗对准【不再用它】: 见下面 TARGET_COARSE_STEP
                                         *    (60 码 ≈ 107px 一步, 比 K230 的 ±40px 窗口
                                         *     还大 2 倍多 ⇒ 只会在窗口两边来回摆)。
                                         *    现在只有【精对准】的大步档(|x|>200px)用它 */

/* ⭐⭐ 打靶【粗对准】(C/L/R 阶段)的步长 + “过冲”检测 (2026-10-08 新增) --------
 * 【为什么原来不收敛 —— 实测“打靶粗定位没有收敛”的根因】
 *   K230 在接近阶段只回 C/L/R, 【没有幅值】, 所以本机只能固定一步转
 *   TARGET_ID1_STEP(60 码 ≈ 107px)。而 K230 判“居中”的窗口是
 *   DIR_THRESHOLD = ±40px:
 *       误差 45px → 收到 L → 转 107px → 变成 -62px → 收到 R → 又转回来 …
 *   一步跨过的距离(107px) > 2×窗口半宽×2(80px) ⇒ 误差在窗口两边来回摆,
 *   永远不会落进 ±40px, 直到转满 TARGET_ID1_STEP_MAX(20 步)或超
 *   TARGET_ID1_TIMEOUT_MS(20s) 被“强制当作已对准”, 带着几十~上百像素的偏差
 *   进精对准(那时容差只有 10px, 更难收敛)。
 * 【怎么修】
 *   ① 常规步长压到 30 码(≈53px): 只要【步长 < 2 × 窗口半宽(80px)】, 按
 *      “每步把 |误差| 减掉一个步长”的规律, 误差一定单调变小、必然进窗口;
 *   ② 再加一层“过冲检测”: 连续两次方向【相反】= 上一步跨过了中心
 *      ⇒ 立刻换细步(12 码 ≈ 21px), 这样即使 px/码 的估算不准也不会再摆。
 * ⚠️ 硬约束: TARGET_COARSE_STEP 必须 < 2 × K230 的 DIR_THRESHOLD(40px);
 *    细步要明显更小(≈ 半个窗口)。想更快可加大常规步长, 但别超过 ~70px。
 * ⚠️ 出现日志 `[靶] 粗对准: ⚠ 方向翻转(第N次…) -> 改用细步` 就说明确实在摆,
 *    那时看 N 的大小: N 很小(1~2)是正常的“最后一步跨过中心”, N 很大就是步长仍偏大。 */
#define TARGET_COARSE_STEP      40      /* 粗对准常规步长(码) ≈ 53px (< 80px 才不摆) */
#define TARGET_COARSE_STEP_MIN  12      /* 检测到过冲后用的细步(码) ≈ 21px */
#define TARGET_ID1_LR_SIGN    (-1)      /* “收到 L”时 ID1 的增量符号:
                                         *  -1 = 数值减小 = 向左(当前实测值)
                                         *  +1 = 数值增大 = 向左(装反了就改这个) */
#define TARGET_ID1_MOVE_MS      600 /*300*/     /* ID1 每步的转动时间(ms)。这期间主循环
                                         * 【忽略并丢弃】K230 新帧, 等舵机停稳再取
                                         * 下一帧; 否则会拿“转之前”的旧画面连续累加
                                         * → 直接冲过头。建议 200~500, 要与上面
                                         * TARGET_ID1_STEP 匹配(转得多就要等得久) */
/* ⚠️⚠️ 下面两个是【相对基准姿态(TARGET_LOOK)的逻辑偏移】, 不是绝对 0~4095!
 *   TARGET_LOOK 的 ID1 标定值 = 2052 > 2048, 而 Arm_Id1Reset 会把 >2048 的基准
 *   归一化成负数(2052 -> -2044, 见 s_id1_pos 的注释)。若这里仍按绝对 0~4095
 *   限幅, Arm_Id1Step 里 v = -2044 + delta 恒 < 0 ⇒ 第一步就被夹到 0
 *   ⇒ ID1 从 2052 直接甩到 0(≈180° = 半圈), 表现就是“底盘停稳交给视觉后
 *     底座猛地转到靶子反方向”。
 *   写法与救援的 RESCUE_ID1_POS_MIN/MAX 一致: 调用处传 base + 本值
 *   (基准值由 Target_Id1Base() 给出)。
 *   窗口取 ±2048 = 整圈, 与“基准落在 0~2048 内时的绝对 0~4095”可用范围等价。 */
#define TARGET_ID1_POS_MIN  (-2048)     /* ID1 逻辑位置下限(【相对基准】的偏移) */
#define TARGET_ID1_POS_MAX  ( 2047)     /* ID1 逻辑位置上限(【相对基准】的偏移) */
#define TARGET_ID1_STEP_MAX     30      /* 🛡防卡死①: 精/粗对准最多转这么多步就强制认为已对准。
                                         * ⭐⭐ 2026-10-11: 20 → 30。
                                         * 为什么必须加大: K230 新代码里精对准有
                                         *   ALIGN_SELF_TIMEOUT(14s) 自超时 —— 到点它【也会
                                         *   fire_start() 点激光 + 发 FIRE/OK】。而本机每步要
                                         *   TARGET_ID1_MOVE_MS(600)+TARGET_FINE_SETTLE_MS(250)
                                         *   = 850ms ⇒ 20 步 = 17s > 14s(勉强够, 但两轴混着走时
                                         *   ID4 那 20 步 = 13s 会先到)。
                                         *   ⚠️ 若本上限【先到】, 本机就会"抬臂收尾"而没有激光 ⇒ 白打。
                                         *   ⇒ 上限必须满足 步数 × 每步耗时 ≥ 14s(让 K230 的超时
                                         *     发射先发生); 30 步 × 650ms(ID4) = 19.5s, 安全。
                                         *   真正的硬上限仍是 TARGET_ID1_TIMEOUT_MS(20s)。 */
#define TARGET_ID1_TIMEOUT_MS   20000   /* 🛡防卡死②: L/R 调整环节最长等这么久(ms),
                                         * 超时强制走 C 流程(摆 FIRE→LIFT→SCAN_RESET);
                                         * 也兜住“K230 一条都不回”的情况。
                                         * ⚠️ 必须 > K230 的 ALIGN_SELF_TIMEOUT(14s), 否则本机
                                         *    先放弃 → K230 那枪 (超时) 发射没打出来就抬臂了。
                                         * 建议 18000~30000 */
/* ⭐ 打靶“精对准”开关(2026-10-05 新增, 适配新 K230 的激光逻辑):
 *   1 = 【当前】粗对准(按 C/L/R 转底座 ID1)收到 C 后, 再发 start_align 让 K230
 *       进入 ALIGN 状态; 之后按它回的 D:<x>,<y> 里的【横向误差 x】继续小步微调
 *       底座 ID1, 直到 K230 回 OK。
 *       ⚠️ 必须这样做 K230 才会点亮激光 —— 它【只在 ALIGN 状态下、且对准成功
 *          (|dx|<ALIGN_TOL_PIX 且 |dy|<ALIGN_TOL_PIX)时才 "FIRE"】,
 *          停在接近(APPROACH)状态永远不发激光。
 *   0 = 旧行为: 收到 C 就当对准完成, 直接摆 FIRE/LIFT/SCAN_RESET。
 *       (此时 K230 不会发射激光, 只在不打激光的调试里用) */
#define TARGET_USE_FINE_ALIGN   1

/* =====================================================================
 * ⭐⭐ 打靶精对准第二轴: 用 ID4(腕部) 修【竖直误差 dy】 (2026-10-06 新增)
 * ---------------------------------------------------------------------
 * 【为什么需要】
 *   K230(yolo_main.py)判定“打靶对准成功”的条件是【|dx| 且 |dy| 都小于它的
 *   TARGET_ALIGN_TOL_PIX】—— 2026-10-11 的最新代码里是 11px(曾经 50px → 20px),
 *   而且要求【连续 2 帧、持续 ≥0.4s】都满足, 才会 fire_start() 点激光 + 发 "FIRE"。
 *   单片机侧用 TARGET_ALIGN_TOLERANCE 同步跟随(⚠️ 必须 ≤ K230 的值)。
 *   而底座 ID1 只能修横向 dx —— 竖直 dy 没人管 ⇒ K230 永远等不到“成功”
 *   ⇒ 只能等它自己自超时(现在 ALIGN_SELF_TIMEOUT = 14s; 最新代码【超时也会点激光
 *     再发 FIRE+OK】, 旧版超时只回 OK 不点)。
 *   (2026-10-06 实测日志就是这现象: x=0 刷了十几次一直不发 FIRE)
 *   ⇒ 现在让 ID4(腕部) 去修 dy, 两个轴交替修, K230 才能真判成功。
 * ---------------------------------------------------------------------
 * 【方向已确认(2026-10-06 硬件实测)】
 *      ID4 数值【小 = 往下低】    ID4 数值【大 = 往上抬】
 *   K230 的 y = 目标中心y - 画面中心y, 所以:
 *      y > 0 → 目标在【画面下方】 → 视线要往下低 → ID4 要【减小】
 *      y < 0 → 目标在【画面上方】 → 视线要往上抬 → ID4 要【增大】
 *   ⇒ 目标偏下(y>0)时给 ID4 【负】增量 ⇒ TARGET_ID4_DY_SIGN = -1
 *      ★当前默认值就是对的, 不用改; 只有腕部装配换了方向才需要取反。
 * ---------------------------------------------------------------------
 * 【完整标定方法】见下面“⭐⭐ 打靶两轴标定方法总表”。
 * ===================================================================== */
#define TARGET_FIX_DY_WITH_ID4  1       /* 1=打靶精对准用 ID4 修 dy【当前】;
                                         * 0=只修 dx(回到旧行为, 此时激光不会发) */
/* ⭐⭐ 竖直轴(ID4)自适应步长 (2026-10-06) --------------------------------
 * 现象: 用户实测“横向 ID1 动得很合适, 但 ID4 动少了” —— 说明 ID4 的
 *       【px/码】比 ID1 小(转同样的码数, 画面竖直方向变位更小),
 *       所以小误差时一步改不了多少, 要转很多步; 而 K230 只有 12s 自超时。
 * 做法: 和 ID1 一样分档 —— 误差大走大步(快), 误差小走小步(稳):
 *      |y| > TARGET_ID4_STEP_FAR_PX(100)  → TARGET_ID4_STEP_FAR(40 码)
 *      |y| > TARGET_ID4_STEP_FINE_PX(40)  → TARGET_ID4_STEP(30 码)
 *      否则                                → TARGET_ID4_STEP_FINE(15 码)
 * ⚠️ 唯一必须守的硬约束: 小步换算出的 px 必须 < 窗口半宽, 否则会永远跨过中心
 *    来回摆。窗口 2026-10-08 已收紧到 TARGET_ALIGN_TOLERANCE=10px ⇒ 15 码
 *    换算出的一步必须 ≤10px(实测 20 码"动少了" ⇒ ID4 的 px/码 本来就小,
 *    15 码大约 5~12px, 落在安全区)。
 * 怎么定准(跑一次看这行日志):
 *   [靶][竖直轴变化] 上一步 ID4 转 NN 码后, |y| ..px -> ..px (变化 ..px)
 *    一步改的 px:  < 5px → 太小, 继续加大(20~30)
 *                  5~9px → 正合适
 *                  > 12px → 太大(超过窗口半径), 必须减到 10 码
 * 不想分档就把 TARGET_ID4_STEP_FAR 改成和 TARGET_ID4_STEP 一样即可。 */
#define TARGET_ID4_STEP         30      /* 中步(误差中等时用) */
#define TARGET_ID4_STEP_FAR_PX  100     /* 超过这么多 px 算“离得远”, 换大步 */
#define TARGET_ID4_STEP_FAR     45      /* 大步(仅大误差时用; 目的是少走几步赶在 12s 内) */
#define TARGET_ID4_STEP_FINE_PX 40      /* ⭐ 小于这么多 px 才用【细步】(≈窗口的 4 倍) */
#define TARGET_ID4_STEP_FINE    15      /* ⭐ 细步: 决定能否收敛进 ±10px 容差
                                         *   (原来最小只有 30 码, 误差 15px 时
                                         *    一步就跨过 ±10px 窗口 ⇒ 来回摆/收敛不了) */
#define TARGET_ID4_DY_SIGN    (-1)      /* dy>0(目标偏画面下)时 ID4 的增量符号:
                                         *  -1 = 数值减小(往下低) ★正确值
                                         *  +1 = 数值增大(往上抬) —— 装配反了才用 */
#define TARGET_ID4_MOVE_MS      400     /* ID4 每步转动时间(ms); 这期间把 K230
                                         * 旧帧全丢掉, 免得拿“转之前”的误差累加 */
#define TARGET_ID4_POS_MIN      1050    /* ID4 行程限幅(角度码), 防越界堵转。
                                         * 姿态表注释: id4 物理限幅 1050~3010 */
#define TARGET_ID4_POS_MAX      3010
#define TARGET_ID4_STEP_MAX     30      /* 🛡防卡死: ID4 最多转这么多步。
                                         * ⭐⭐ 2026-10-11: 20 → 30 —— 与 TARGET_ID1_STEP_MAX
                                         * 同一理由: 20 步 × (400+250)ms = 13s < K230 的
                                         * ALIGN_SELF_TIMEOUT(14s), 会在 K230 超时发射之前
                                         * 先"抬臂收尾" ⇒ 白打一枪。30 步 = 19.5s 安全。 */

/* ⭐⭐ 2026-10-11 新增(用户要求): 打靶"丢靶"自恢复 —— 让 ID4 上下动一下去把靶子找回来 --
 * 【现象】精对准时 ID1 跟着画面一步步入(每步 400~600ms), 转到某一步之后 K230
 *   有时【识别不到靶子】了 —— 它进了 ALIGN 之后只在看到目标时发 D:<x>,<y>,
 *   目标跑出它的识别范围(或视野被腕部/线缆挡住)就【一行都不发】,
 *   于是本机一直干等到 20s 防卡死兜底才收尾(白等十几秒, 而且此时并没对准)。
 * 【用户的处理办法】这时让 ID4(腕部)【上下动一下】, 换个视角把靶子重新收进识别范围,
 *   然后继续等 K230 的信号(D 帧 / OK)。
 * 【"丢靶"判据】必须同时满足:
 *   ① 精对准期间【收到过 D 帧】(证明它本来能看到, 不是还没进 ALIGN);
 *   ② 距上次收到 K230 任何一行 ≥ TARGET_LOST_SILENCE_MS;
 *   ③ ID1 已经跟着画面动过 ≥ TARGET_LOST_MIN_ID1_STEPS 步(用户: "ID1 跟着摄像头移动之后");
 *   ④ K230 还没发过 FIRE(FIRE 之后它【故意】停发误差, 那不算丢靶);
 *   ⑤ 搜索次数没到 TARGET_LOST_SEARCH_MAX, 且距上次搜索 ≥ TARGET_LOST_SEARCH_GAP_MS。
 * 【动作】Target_Id4Step(±TARGET_LOST_ID4_STEP), 方向【一次上抬、一次下低交替】
 *   ⇒ 搜满 MAX 次(偶数)后净位移 = 0, 不会把已经对好的位置搜跑;
 *   然后照常进 TARGET_FINE_MOVING(丢旧帧)→ 回 TARGET_FINE_IDLE 等信号。
 *   ⚠️ 方向口径: 本工程 TARGET_ID4_DY_SIGN = -1 ⇒ 码值【增大】= 往上抬(画面里的目标
 *      相对往下走), 码值【减小】= 往下低。这里只按码值正负交替, 与装配方向无关。
 * 【取值】TARGET_LOST_SEARCH_ENABLE = 0 ⇒ 老行为(一直干等到 20s 兜底)。
 * ⚠️ TARGET_LOST_SILENCE_MS 别小于 K230 的正常出帧间隔(它偏了才发 D 帧, 对准过程中
 *    可能几百 ms 才一帧): 建议 1000~2000。
 * ⚠️ 搜完还没信号就会一直等到防卡死兜底 —— 若实测"搜了也没用", 把 MAX 调到 0 关掉。 */
#define TARGET_LOST_SEARCH_ENABLE      1     /* 1 = 开启丢靶自恢复; 0 = 直接干等(老行为) */
#define TARGET_LOST_SILENCE_MS         3500  /* 距上次收到 K230 数据超过它 → 判为丢靶。
                                              * ⭐⭐ 2026-10-11: 1500 → 3500(对齐 K230 新代码)。
                                              * 为什么必须放大: 新 yolo_main.py 在精对准里
                                              *   【本来就是断续发帧】的 ——
                                              *     · 每发一帧 D 之后就静默 ALIGN_SETTLE(1.2s);
                                              *     · 两帧 D 之间至少隔 ALIGN_SEND_IVL(1.0s);
                                              *     · 若两帧算出来的误差【一模一样】(靶子没动,
                                              *       但还差一点没进窗口) 则要等
                                              *       ALIGN_RETRY_IVL(3.0s) 才重发。
                                              *   再加上它判成功前要"连续稳定 0.4s", 没进窗口时
                                              *   它可能安静 3~4 秒 —— 这些都是【正常】的。
                                              * ⚠️ 静默门限若小于上面这段正常间隔, 就会出现
                                              *   【误判丢靶】: 我们拿 ID4 上下甩 40 码去找靶,
                                              *   正好把 K230 那 0.4s 的稳定性打断 ⇒ 它永远判不过、
                                              *   我们已经对好的位置也被搜跑 ⇒ 表现就是"激光乱晃"。
                                              * ⇒ 取 3500 > 3.0s(ALIGN_RETRY_IVL), 只有真丢靶
                                              *   (K230 一行都不发) 才会触发。 */
#define TARGET_LOST_MIN_ID1_STEPS      1     /* ID1 至少跟画面动过这么多步才启用 */
#define TARGET_LOST_ID4_STEP           40    /* 每次"上下动一下"的幅度(角度码)。
                                              * 参考: 常规竖直步长是 15/30/45 ⇒ 40 是"大步" */
#define TARGET_LOST_SEARCH_MAX         4     /* 最多搜这么多次(偶数 ⇒ 搜完回到原位) */
#define TARGET_LOST_SEARCH_GAP_MS      1200  /* 两次搜索之间至少隔这么久, 别抖个不停 */

/* =====================================================================
 * ⭐⭐ 2026-10-10 【打靶精对准稳定性包】(用户实测: “机械臂晃动严重”)
 * ---------------------------------------------------------------------
 * 【看到的现象】打靶阶段(尤其精对准 D 帧微调时)整条手臂在晃。
 *
 * 【代码层能查到的一条主因: 微动被“抽搐化”】
 *   精对准一步只转 2~15 码, 而通用接口 Servos_SetPositionsMasked() 内部
 *   有速度下限 SERVO_SPEED_MIN = 200 步/秒(防止“距离算不出来时蠕动”):
 *       速度 = 距离 ÷ time_ms。5 码 ÷ 0.6s = 8 步/秒 < 200
 *       ⇒ 被抬到 200 步/秒 ⇒ 实际只用 5/200 = 25ms 就冲到位!
 *   也就是说我们给的 TARGET_ID1_MOVE_MS(600ms)“慢走”根本没生效, 每修一次
 *   都是给大惯量的手臂来一下 25ms 的“抽搐” ⇒ 过冲 + 余振 ⇒ 摄像头画面跟着
 *   抖 ⇒ 下一帧误差不准 ⇒ 再修一次 = 肉眼看到的“一直在晃”。
 *   ⇒ 对策①: 微动改走 Servos_SetPositionsMaskedEx(), 用小的速度下限,
 *     让它真的按 600/400ms 慢慢走完(见 TARGET_FINE_SPEED_MIN)。
 *
 * 【另外三条“别抖着决策”的对策】
 *   ② 每步走完再多【静置】一会儿, 期间 K230 的 D 帧全丢掉, 等手臂余振衰减、
 *      画面稳了再决策(TARGET_FINE_SETTLE_MS);
 *   ③ 刚进精对准也要先【静置】(粗对准/扫描刚停, 整臂还在摆, 第一帧不可信)
 *      (TARGET_FINE_START_SETTLE_MS);
 *   ④ 万一还是跨过中心(误差符号翻转), 立刻改用【微步】, 避免在窗口两边
 *      来回摆(TARGET_FINE_MICRO_ENABLE / *_STEP_MICRO)。
 *
 * ⚠️ ②③ 是“用时间换稳定”: K230 那边精对准只有 12s 自超时(超时只回 OK、
 *    【不点激光】)。若日志出现它的 12s 超时, 先把 ②③ 调小(如 150/200)。
 * ⚠️ 只影响【打靶精对准】: 粗对准(C/L/R)、救援的 ID1 动作、其它姿态切换
 *    一律照旧(它们传的还是通用速度下限)。
 * ⚠️ ①②③④ 每一项都给了“回老行为”的值(200 / 50 / 0 / 0)。
 * ===================================================================== */
#define TARGET_FINE_SPEED_MIN       20   /* 微动速度下限(步/秒)。★核心开关:
                                          * 200 = 回老行为(25ms 抽搐一下);
                                          * 建议 20~60。例: 5 码微步 ÷ 20 步/秒
                                          * ⇒ 250ms 走完(不再是 25ms) */
#define TARGET_FINE_ACC             20   /* 微动加速度(0~254, 越小起步/停住越柔)。
                                          * 老行为是 50(SERVO_ACC_DEF) */
#define TARGET_FINE_SETTLE_MS       250  /* 每步走完再多等这么久才看下一帧
                                          * (期间 D 帧全丢)。0 = 关掉 */
#define TARGET_FINE_START_SETTLE_MS 400  /* 刚进精对准先静置这么久(丢帧)。
                                          * 0 = 关掉 */
#define TARGET_FINE_MICRO_ENABLE    1    /* 1=误差符号翻转(上一步跨过中心)时改用微步
                                          * 0=关掉 */
#define TARGET_ID1_STEP_MICRO       2    /* 横向微步(码) ≈ 4px(±10px 窗口内够用) */
#define TARGET_ID4_STEP_MICRO       8    /* 竖直微步(码) ≈ 3~6px */

/* ---- 收到 FIRE 之后给 K230 的宽限期 ----
 * ⚠️ 实测(2026-10-06): K230 在判“对准成功”那一刻会发一行 "FIRE" 并点亮激光,
 *    【然后就不再发 D:x,y, 也不再回 OK】。如果单片机只认 OK, 就会白等
 *    TARGET_ID1_TIMEOUT_MS(20s) 才收尾 —— 比赛里这 20 秒很致命。
 * 所以: 收到 FIRE 后开始计时, 宽限期内又收到 OK 就用 OK 结束;
 *       宽限到期还没 OK, 也照样当作完成。
 * 取值/推荐: 1000~3000(默认 1500)。设太小 → 万一 OK 晚到就丢了;
 *            设太大 → 白等。 */
#define K230_FIRE_GRACE_MS      1500

/* ⭐ 收到 K230 的 "OK"/"FIRE" 之后, 【保持当前对准姿态】这么久再抬臂。
 * 为什么需要: 激光是 K230 在它判“对准成功”那一刻自己点亮的, 而抬臂会【立刻】
 *   改变摄像头/激光的朝向 —— 实测“刚收到 OK 就抬臂”时激光只闪了一下,
 *   靶上打不出稳定的点。
 * ⚠️⚠️ 必须 ≥ K230 侧的 FIRE_DURATION_MS(现在 = 2500ms): 那是它把激光
 *   【一直点亮】多久。本值比它小的话, 激光还没灭我们就开始抬臂 ⇒ 后半段
 *   打在别处。3000 比 2500 多留 500ms 余量。
 * 取值/推荐: 3000(需 ≥ 2500)。设小→激光照得短/后半截打偏; 设大→白耗时。 */
#define TARGET_FIRE_HOLD_MS     3000

/* ⭐⭐ 2026-10-11(用户实测反馈 + K230 新代码对齐): 打靶【冻结带】——够准了就别再动舵机 --
 * 【现象】"摄像头瞄准靶心之后 ID4 还在抖, 激光出靶子"。
 * 【两个来源】
 *   ① K230 判成功后会先发 "FIRE"(点激光)再发 "OK"。老代码收到 FIRE 只是记个时间戳,
 *      【继续拿后面的 D 帧微调】⇒ 激光亮着的那 2.5s 里 ID4 还在一步几码地转, 腕部余振
 *      把激光点晃出靶心。⇒ 现在收到 FIRE 【立刻冻结姿态】(见 TARGET_FINE_IDLE)。
 *   ② 即使还没 FIRE, 只要两轴误差还够小, 我们仍按 TARGET_ALIGN_TOLERANCE 继续"蹭"
 *      ⇒ 舵机一直在动, 抖动不停。
 * 【K230 新代码(2026-10-11 yolo_main.py)的判成功条件 —— 本宏必须跟着它对】
 *   K230 判成功 = |dx| < TARGET_ALIGN_TOL_PIX(11) 且 |dy| < 11, 而且要
 *   【连续 OK_STABLE_FRAMES(2) 帧、持续 ≥ OK_STABLE_HOLD(0.4s)】才 fire_start() + "FIRE"。
 *   ⇒ 本宏必须 ≤ 11(K230 的窗口), 两边才对得齐:
 *        · 两轴都 < 本宏时立刻停手 ⇒ 舵机一动不动, K230 这 0.4s 里能连判 2 帧成功
 *          (腕部不动 = 画面不抖 = 激光不晃);
 *        · 若本宏【大于】K230 的窗口, 我们就会"冻在 K230 判不过的位置" ⇒ 它一直等到
 *          ALIGN_SELF_TIMEOUT(14s) 自超时才盲射一枪 —— 又慢又不准。
 *   ⚠️ 老值 20 是配【旧 K230】(那时窗口 50px)定的; 换新代码后 20 就是上面第二种情况,
 *      所以改成 10(= TARGET_ALIGN_TOLERANCE, 与 K230 的 11px 对齐)。
 * 【取值】10。0 = 关闭(回到"一直微调到容差或收到 OK 为止", 会有抖动风险)。
 *   ⚠️ 可以调小(如 6)换更高精度, 代价是接近靶心时还会多动几下(多晃几下);
 *      不要往大调(> 11 就冻在 K230 判不过的地方)。
 * --------------------------------------------------------------------- */
#define TARGET_STOP_NUDGE_TOL_PX   10

/* ⭐ 打靶两轴标定(mode 6)专用: 1 = 只标竖直轴(ID4), 横向(ID1)完全不动。
 * 用途: ID1 标好之后, 单独标 ID4 的符号 / 步长。
 * ⚠️ K230 每帧只发“误差大的那一个轴”, 所以用本项时必须【人工把靶子横向
 *    摆到画面中间】(让 |x| < 容差), 否则它一直报 x、而我们又不修 x, 会僵住。
 * 标完记得改回 0。 */
#define TCAL_ONLY_Y             0

/* =====================================================================
 * ⭐⭐ 打靶两轴标定方法总表 (2026-10-06) —— 对着日志一步一步来就行
 * =====================================================================
 * 【硬件事实(已确认)】
 *    ID1 = 底座(左右转)      ID4 = 腕部(俯仰): 数值【小=往下低 / 大=往上抬】
 *    K230 误差定义:  x = 画面中 - 目标x  → x>0 = 目标偏【画面左】
 *                   y = 目标y - 画面中  → y>0 = 目标偏【画面下】
 *    K230 判“对准成功”的条件: |dx|<11px 且 |dy|<11px(两者都要满足!),
 *    只有判成功它才会 fire_start() 点激光并发一行 "FIRE"。
 *    (K230 侧常量 TARGET_ALIGN_TOL_PIX; 单片机侧用 TARGET_ALIGN_TOLERANCE=10,
 *     ⚠️ 必须 ≤ 它, 否则本机先"认为够准了"不再动舵机 → 两边干等到 12s 自超时)
 * =====================================================================
 * ① 竖直轴 ID4 —— 方向(TARGET_ID4_DY_SIGN)
 *    推导: y>0(目标偏下) → 视线要往下低 → ID4 减小 → 增量取负 ⇒ 符号 = -1
 *    验证: 单跑 MISSION_DEBUG_VISION_TASK=2, 看日志
 *          [靶] 精对准[竖直] y=-80px (y>0=偏下) -> ID4 转 +40 码
 *          [靶][竖直轴] 上一步 ID4 动作后, 竖直误差 -120px -> -80px (变化 40px)
 *          判定: |y| 越修越小 = 方向对; 越修越大(-120→-200) 就把符号取反。
 *    ★当前 TARGET_ID4_DY_SIGN = -1 就是正确值。
 * ---------------------------------------------------------------------
 * ② 竖直轴 ID4 —— 步长(TARGET_ID4_STEP / _FINE / _FAR / _FAR_PX)
 *    看上面那行“变化 xx px”, 它打的是【上一步实际转的码数】:
 *      一步只改很少像素(如 15 码才改 3px) → 两步都加大(细步 20~25, 远步 50~60)
 *      一步就冲过头(如 -15 变成 +20)      → 减小, 且【细步必须 ≤ 10px】
 *    经验: 大误差用大步(快), 小误差用大步会在 ±10px 窗口里来回摆 ——
 *    所以“能否收敛”只看【细步】的 px, “快不快”看【远步】。
 *    目标手感: 让“细步的 px 变化 ≈ 5~9px”, 2~4 步内收敛。
 * ---------------------------------------------------------------------
 * ③ 水平轴 ID1 —— 方向(TARGET_ID1_LR_SIGN)
 *    看日志: [靶] 精对准[横向] x=-80px (x>0=偏左) -> ID1 转 +60 码
 *    判定: |x| 越修越小 = 对; 越修越大就把 TARGET_ID1_LR_SIGN 取反。
 * ④ 水平轴 ID1 —— 步长(TARGET_ID1_STEP), 默认 60 码 ≈ 5.3°, 同样按“越修越小”
 *    和“是否冲过头”来调。
 * ---------------------------------------------------------------------
 * ⑤ 底盘(球/桶阶段)的 K_GAIN —— 用 [K标定] 日志
 *      [球][K标定] 上一步走 57mm 使横向误差变化 -354px => 本次实测 a≈0.161 mm/px
 *      [球][K标定] 横向实测 a 平均 ≈ 0.187 mm/px -> 建议 K_GAIN ≈ 0.093 (当前 0.25)
 *    把 K_GAIN 改成那个“建议值”, 重新构建再试; 若还来回摆就再取一半。
 * ---------------------------------------------------------------------
 * 【标定成功的标志】日志按顺序出现:
 *      [靶] 精对准[横向] x=..px -> ID1 转 ..码      (或 [竖直] ID4)
 *      [靶] 两轴都已在容差(10px)内 (x=.., y=..), 等 K230 回 OK/FIRE
 *      [靶] K230 已发射激光(FIRE)                    ← 激光真亮了
 *      [靶] 精对准完成(收到OK, 共微调N步) -> 摆发射位
 *      [靶] 打靶收尾(收到OK -> 抬起大臂 -> 收回手臂)
 * ===================================================================== */
/* ⭐ 2026-10-03: 本工程【不再用单片机控制激光】。
 *    激光由 K230(摄像头模块) 自己控制: K230 在 ALIGN 状态判“对准成功”时自己
 *    fire_start() 点亮, 并给单片机发一行 "FIRE" 做通知;
 *    单片机只负责把两个轴(ID1 横向 / ID4 竖直)微调到 K230 判成功,
 *    然后收到 "OK" 就抬大臂 → 收回手臂。
 *    (2026-10-06 起不再摆 ARM_POSE_TARGET_FIRE 发射位, 见 TARGET_PERFORM 注释)
 *    (原来的 LASER_FIRE_DURATION_MS / Laser_On() / Laser_Off() 已从任务2 移除;
 *     Mission_Init() 里的 Laser_Off() 保留, 只为保证上电时激光是关的) */

/* =====================================================================
 * 路线距离/角度宏 (单位: 距离 mm, 角度 度)
 * —— 想改“某一段走多远/转多少”, 改这里即可, 无需翻下面状态机。
 * ===================================================================== */

/* =====================================================================
 * ⭐⭐⭐ 2026-10-10(用户要求): 全工程【“挪一小步”】的宏 —— 集中在本段 ---------
 * ---------------------------------------------------------------------
 * 【同一个原理】本段的宏都是一回事: 平移/转向到位之后、【原地转正之前】先按下面
 *   的值让车挪一小步(抢在转向前面), 也就是 Route_MinStep() / Route_BackMinStep()
 *   的入参:   >0 = 车头前进 | <0 = 车头后退 | 0 = 不挪这一步(只做紧接着的转向)
 * 为什么要挪:
 *   ① 车完全停死时靠四轮差速原地转正本来就容易转不到位(见 Chassis.h 的“卡住”检测),
 *      先让轮子滚一下, 后面那次转向更容易到位、更准;
 *   ② 顺手补掉这一段平移攒下的【前后】漂移(它【不改】左右行程)。
 * ⚠️ 方向: Chassis_Move_Forward 是【车体坐标系】的“朝车头”(见 Chassis.c 的 add_move)
 *    ⇒ “前进”在场地里是哪个方向, 取决于此刻车头朝向 —— 下面每一段都标了车头朝向。
 * ⚠️ 死区: 到位判定死区 = CH_POS_THRESHOLD_COUNT(30 计数) ≈ 4.5mm
 *    ⇒ 【实际位移 ≈ |本值| − 4.5mm】; 绝对值 < 6 会被死区整个吃掉(等于没动还白等一次)。
 * 排列顺序 = 车实际跑的顺序(排爆后 → 打靶走位 → 救援段), 每行都标了“前进/后退”。
 * ⚠️ 原来散在各段路线宏旁边的这些定义【已全部搬到这里】(原位只留一行指路注释);
 *    本段【只放“挪一小步”这一类】, 路线距离/转向角度仍在各段自己的小节里。
 * ===================================================================== */

/* ==== ① 排爆做完 → 打靶走位之前 (STATE_11A_HEADING_CORRECT; 车头 0°) ========
 * 作用: 排爆(抓球 + 放桶)全做完、进打靶走位之前先校一次航向 ——
 *       ① Route_MinStep(本宏); ② Turn_Angle_Compat(0.1f)(转到绝对 0°)。
 * ⚠️ 此刻车头还是 0° ⇒ “车头前进”就是【朝场地前方】走; 与救援段(车头 -90°)含义不同。 */
#define ROUTE_BOMB_AFTER_STEP_MM    0      /* 车头前进(实际约 本值−4.5mm); 0 = 不挪, 只转到 0° */

/* ==== ② 打靶走位途中: 每次航向校正【之前】(车头 0°) ======================
 * 作用状态 = 走位链的 4 处航向校正(每半段末尾各一次, 共 4 处), 每处做法完全一样:
 *   右移到位 → 挪一小步 → 再原地转正到 0°。
 *      第 1 处 STATE_12_PART1_CORRECT_A   (第1段右移【前半】走完)
 *      第 2 处 STATE_12_PART1_CORRECT_A2  (第1段右移【后半】走完)
 *      第 3 处 STATE_12_PART1_CORRECT_B   (第2段右移【前半】走完)
 *      第 4 处 STATE_12_PART1_CORRECT_B2  (第2段右移【后半】走完)
 * ⭐ 4 处的步长【各自一个宏, 互不影响】; 第 2 / 第 4 处还能再选“按偏航角”(方案二)。
 * ⚠️ 这一步发生在“右移段结束、车身还没转回 0°”的时刻, 方向是车的【前后】:
 *    它【不改】右移的左右行程(ROUTE_12_P1_A_MM / _B_MM 一点不动)。
 * ⚠️ 右移途中的前后漂移也可以改用“全程按比例补”(ROUTE_12_RIGHT_BACK_COMP)那条路
 *    —— 两条一般只开一条。 */
#define ROUTE_12_A_MINSTEP_MM       0//(-15)   /* 第 1 处(CORRECT_A ): 车头【后退】15mm(实际约 10.5mm) */
#define ROUTE_12_A2_MINSTEP_MM      0//(-15)   /* 第 2 处(CORRECT_A2): 车头【后退】15mm(方案一时的固定值) */
#define ROUTE_12_B_MINSTEP_MM       15      /* 第 3 处(CORRECT_B ): 车头【后退】15mm(实际约 10.5mm)。
                                                * ★2026-10-10: 它现在也是【“一条路”走法中间那次
                                                * 航向校正】的“挪一小步”(见 ROUTE_12_MID_CORRECT_ENABLE)
                                                * —— 中间那一下要改步长就改这里。 */
#define ROUTE_12_B2_MINSTEP_MM      0//(-15)   /* 第 4 处(CORRECT_B2): 车头【后退】15mm(方案一时的固定值) */

/* 打靶走位这 4 处的【总开关】:
 *   0 = 关闭: 4 处都是“右移到位 → 直接原地转正”, 不挪这一步;
 *   1 = 打开(当前): 4 处各按自己的值先挪一步再转正。
 * 为什么要单独做个开关而不是直接看步长是不是 0: 实车要反复在“挪 / 不挪”之间对比,
 *   每次改步长会把标定好的绝对值冲掉; 用开关切更保险 —— 打开时只要把值填回去即可。
 * ⚠️ 开关打开却把 4 处全留成 0 = 白开: 下面有 #error 提前拦住。
 *    (单独某一处填 0 是合法用法 = 只有那一处不挪, 不会报错)
 * ⚠️ 只作用于打靶走位这些校正处; 排爆后那处与救援段那几处【不受它控制】。
 * ⚠️ 2026-10-10: 置 1(打开) —— 因为“一条路”走法【中间那次航向校正】也要挪这一小步
 *    (Route_BackMinStep 是打靶走位里唯一带“阻塞等到位”的挪步实现, 关掉它会连中间
 *     那一步一起失效)。当前各处的值:
 *      中间那次 = ROUTE_12_B_MINSTEP_MM(-15)   最后那次 = ROUTE_12_B2_MINSTEP_MM(0 = 不挪) */
#define ROUTE_12_MINSTEP_ENABLE     1

/* 第 2 / 第 4 处各有两个方案, 【各自独立】选:
 *   0 = 方案一(稳定值): 每次都挪该处上面那个固定值;
 *   1 = 方案二(按偏航角): 先看此刻的偏航角, 偏出 ±ROUTE_12_YAW_LIMIT_DEG 才挪, 方向由
 *        偏角符号决定 —— 车头往哪边歪, 就往【反方向】补一点, 把“斜着走”攒下的横向
 *        漂移在段末掰回来; 偏角在区间内(车头基本正)就【不挪】(0)。
 *        映射(取值见下面各处的宏):
 *          yaw > +门限(车头偏“左”) → 负值(车头后退)
 *          yaw < −门限(车头偏“右”) → 正值(车头前进)
 *          |yaw| ≤ 门限            → 0(不挪)
 * ⭐ 当前实车选择(2026-10-10): 4 处都用【方案一固定值 -15】(两个模式宏都是 0)。
 * ⚠️ 第 1 / 第 3 处(CORRECT_A / _B)目前【只有固定值】(没有方案二)。想给它们也加
 *    “按偏航角”那套: 照 A2/B2 复制一份(一个模式宏 + 三个取值宏 + 一个包装函数)即可。
 * ⚠️ 判据用的是【当前实时偏航角】(Chassis_GetYaw(); 本阶段航向基准 = 0°, 所以读数
 *    就是偏差), 取自“挪这一小步之前、还没发转向指令”那一刻 —— 也就是这次航向校
 *    正要修掉的那个偏差。
 * ⚠️ 与 ROUTE_12_MINSTEP_ENABLE 的分工: 那个是【总开关】(置 0 则 4 处都不挪),
 *    这里的两个开关决定【总开关打开后】第 2 / 第 4 处走哪个方案。
 * ⚠️ 方案二选“区间内 → 0”时, Route_BackMinStep(0) 会直接返回(不挪、不等待), 只做
 *    紧接着的那次航向校正。 */
#define ROUTE_12_A2_STEP_MODE   0   /* 第 2 处(CORRECT_A2): 0 = 方案一(固定 ROUTE_12_A2_MINSTEP_MM) / 1 = 方案二(按偏航角) */
#define ROUTE_12_B2_STEP_MODE   0   /* 第 4 处(CORRECT_B2): 0 = 方案一(固定 ROUTE_12_B2_MINSTEP_MM) / 1 = 方案二(按偏航角) */

/* 方案二公用的【偏角门限】(度): 只看绝对值, 两个方向都管
 * (用户要求 1.1°, 实车又调到 0.9 —— 改这一个数就同时管第 2 / 第 4 处) */
#define ROUTE_12_YAW_LIMIT_DEG      0.9f

/* 第 2 处(CORRECT_A2) 方案二 的三个取值(mm; 符号同 Route_MinStep: >0 前进 / <0 后退) */
#define ROUTE_12_A2_YAW_POS_MM      (-15)   /* yaw > +门限(车头偏左) → 车头【后退】15mm */
#define ROUTE_12_A2_YAW_NEG_MM      (+15)   /* yaw < −门限(车头偏右) → 车头【前进】15mm */
#define ROUTE_12_A2_YAW_MID_MM      (0)     /* |yaw| ≤ 门限 → 不挪 */

/* 第 4 处(CORRECT_B2) 方案二 的三个取值(mm) */
#define ROUTE_12_B2_YAW_POS_MM      (-15)   /* yaw > +门限 → 车头【后退】15mm */
#define ROUTE_12_B2_YAW_NEG_MM      (+15)   /* yaw < −门限 → 车头【前进】15mm */
#define ROUTE_12_B2_YAW_MID_MM      (0)     /* |yaw| ≤ 门限 → 不挪 */

/* ⚠️ 防呆: 总开关开着, 但 4 处【一处都挪不了】= 一定是填漏了。
 *    (单独某一处填 0 是合法用法“这一处不挪”; 第 2/4 处走方案二时不看固定值, 也不报错) */
#if ROUTE_12_MINSTEP_ENABLE && (ROUTE_12_A_MINSTEP_MM == 0) && (ROUTE_12_B_MINSTEP_MM == 0)
#if (ROUTE_12_A2_STEP_MODE == 0) && (ROUTE_12_B2_STEP_MODE == 0)
#if (ROUTE_12_A2_MINSTEP_MM == 0) && (ROUTE_12_B2_MINSTEP_MM == 0)
#error "ROUTE_12_MINSTEP_ENABLE=1 但 4 处挪一小步全为 0(一处都不会挪): 请填值(如 -15), 或把总开关置 0"
#endif
#endif
#endif

/* ==== ③ 救援段(阶段四; ⚠️ 车头已经是 RESCUE_HEADING_DEG = -90°) ============
 * ⚠️⚠️ 方向重映射: 车头右转 90°(顺时针)之后, 车体【前方】在场地里 = 【右】方向
 *    (与 ④⑧⑩ 的右移同向) ⇒ 本段里的“车头前进”= 往【场地右】走,
 *    “车头后退”= 往场地左 —— 千万别按字面理解成“朝场地前方/后方”。
 * 排列 = 车实际跑的顺序。 */

/* ② 转完 90° 之后、那次航向校准【之前】
 *    (位置: ①转90° → 【本步】 → ②航向校准 → ③停稳 RESCUE_ALIGN_SETTLE_MS → ④右移进救援区)
 * 作用状态 = STATE_15_TURN_FOR_HOSTAGE。
 * ⭐ 2026-10-08 实车调整: 这个宏可正可负(符号约定同 Route_MinStep), 且与 ⑨⑪ 各自独立;
 *    实车最后定的是 80 —— 即“转完 90° 后先往救援区那一侧送 80mm”。 */
#define ROUTE_RESCUE_AFTER_TURN_MM   90     /* 车头【前进】80mm(实际约 75.5mm); 0 = 关掉这一步 */

/* ⭐⭐ 2026-10-11 新增(用户要求): 转完 90° 后那一步(上面那个 75mm)的【冲击保护】------
 * 【为什么要】这一步是"刚掉完头、贴着边界往救援区送一段", 只有 75mm:
 *   全局速度 300mm/s 时它 0.25s 就走完, 起步就是全速 ⇒ 实测容易"冲太快出界"。
 *   位置环自带的"提前减速"只削末段速度, 削不掉"起步就冲", 所以这里两件事一起做:
 *     ① 压【本段速度上限】= ROUTE_RESCUE_AFTER_TURN_SPEED_CAP_MMPS(全程都慢, 起步也慢);
 *     ② 拉长【减速起始距离】= ROUTE_RESCUE_AFTER_TURN_RAMP_MM(提前开始收油)。
 * 【取值】速度上限 120mm/s(≈全局的 40%), 减速起始 40mm(75mm 的段从一半就开始减速)。
 *   觉得还是冲 → 上限降到 80~100; 觉得太慢 → 上限 150、减速 25。
 *   把两个都置 0 = 关闭冲击保护(回到纯默认)。
 * ⚠️ 只作用于【这一步】; 生效时会打一行 "冲击保护: 本步 75mm —— 限速 … " 便于核对。 */
#define ROUTE_RESCUE_AFTER_TURN_SPEED_CAP_MMPS  120   /* 本段最高速度(mm/s); 0 = 不限 */
#define ROUTE_RESCUE_AFTER_TURN_RAMP_MM         40    /* 本段减速起始距离(mm); 0 = 用默认(30mm) */

/* ⑥ 到人质处、【开始识别/伸臂之前】先退开的一段
 *    (位置: ⑤停等 RESCUE_STOP_WAIT_MS → 【本步】→ 摆 HOSTAGE_LOOK → ⑦视觉识别/对准)
 * 作用状态 = STATE_16_RESCUE_RIGHT_A 的进入动作。
 * ⚠️ 这一处是本家族里【唯一不接转向】的: 它后面跟的是“摆臂 + 视觉识别”(不是航向校正);
 *    而且代码里【不走 Route_MinStep】, 是自己发 Chassis_Move_Forward/Backward(见状态机)。
 * 为什么要: 车右移进救援区容易停得比标定位更贴近人质, 先退开一点, 伸臂/识别/抓取都有
 *   余量, 不会顶到人质架(退的这一步会被“按 Y 误差补 ID2 里程”顺带吸收一部分)。
 * ⚠️ 这一步走完才伸臂(代码阻塞等它到位) ⇒ 机械臂一定是车停稳后才摆出去;
 *    退的时候臂还收着(上一个姿态), 不会剐蹭。 */
#define ROUTE_RESCUE_GRAB_BACK_MM   (15)   /* 车头【后退】25mm(实际约 20.5mm); 0 = 关掉这一步 */

/* ③' 停稳之后、④ 右移进救援区之前: 【再纯校一次航向】
 * 作用状态 = STATE_15D_RESCUE_RECORRECT(夹在 STATE_15C_RESCUE_ALIGN_SETTLE 与
 *            STATE_15B_RESCUE_APPROACH_RIGHT 之间), 进入动作 = Route_MinStep(本宏)
 *            → Heading_AlignTo(RESCUE_HEADING_DEG)。
 * 为什么要它: ① 原地右转 90° 本身可能把车身带偏一点(麦轮原地转的残余角/回正不足);
 *   ②(转完那次校准)之后又挪了一小步 + 停稳 3s, 期间车身也可能轻微走动
 *   ⇒ 真正开始右移之前再按绝对角校一次, 免得带着偏差一路横移进救援区。
 * ⭐ 本处的取值 = 0(用户要求): 【不前进后退, 只纠正陀螺仪】—— 而 0 在 Route_MinStep
 *   里就是“直接返回、不挪步”, 所以这一处只发转向指令。
 * ⚠️ 为什么平时都要先挪一小步: 车完全停死时靠四轮差速原地转正容易转不到位
 *   (见 Chassis.h 的“卡住”检测)。这一处车身刚校过、偏差小, 不挪也够用;
 *   若实车发现这次“转不到位/停在半路”, 把本宏改成 ±6 即可(其余逻辑不用动)。 */
#define ROUTE_RESCUE_PRE_RIGHT_STEP_MM   0   /* 0 = 不挪步(只校航向) */

/* ④' 右移进救援区【中途】那次航向校准之前
 *    (965mm 拆两段: 第一段 = ROUTE_14_MID_MM → 【本步 + 校准到 -90°】→ 第二段)
 * 作用状态 = STATE_15B2_RESCUE_MID_CORRECT。
 * ⚠️ 走到这里时车头已是 -90° ⇒ “车头前进”实际走的是【场地“右”】、“车头后退”走的是
 *    【场地“左”】(分别与 ④ 的右移同向 / 反向)。
 * ⚠️ 2026-10-10 核对修正: 本宏当初是按“+10 = 前进(把这段右移再加一点)”设计的, 但当前
 *    生效的值是 -15 = 【后退】⇒ 语义已经变成“这段右移【少走】约 15mm”(实车确认要后退,
 *    所以【不改数值】, 只把说明改对)。 */
#define ROUTE_RESCUE_MID_STEP_MM    0/*-15*/      /* 车头【后退】15mm(实际约 10.5mm) ⇒ 这段右移少走 15mm */

/* ⑨ 抓完人质后【第 1 段右移】之后那次航向校准之前
 * 作用状态 = STATE_17A_RESCUE_HEADING_CORRECT(位置: ⑧右移 → 【本步 + 校准到 -90°】→ ⑩右移) */
#define ROUTE_RESCUE_FWD_MM         0/*-25*/     /* 车头【后退】25mm(实际约 20.5mm); 0 = 关掉这一步 */

/* ⑪ 抓完人质后【第 2 段右移】之后那次航向校准之前(最后一次)
 * 作用状态 = STATE_18A_RESCUE_HEADING_CORRECT(位置: ⑩右移 → 【本步 + 校准到 -90°】→ ⑫停下) */
#define ROUTE_RESCUE_LAST_STEP_MM   0     /* 车头【后退】25mm(实际约 20.5mm); 0 = 关掉这一步 */

/* ⚠️ 救援段的【航向校准目标角】不在本段: 见 RESCUE_HEADING_DEG(-90°);
 *    救援段的【路线距离(右移多少)】也不在本段: 见 ROUTE_14/17/18_*。 */

/* ---------- 阶段一: 扫码区走位 ---------- */
                             /*正式地图*//*自己地图*/
#define ROUTE_1_TO_QR_MM           656      /* 起点 → 二维码扫描点(直行) */
#define ROUTE_3_LEFT_A_MM          557    /* 扫码后左移 A 段 */
#define ROUTE_4_DIAG_FWD_MM        130      /* 左上斜跑: 前进分量(≈45°斜走) */
#define ROUTE_4_DIAG_LEFT_MM       130     /* 左上斜跑: 左移分量(≈45°斜走) */

/* ---------- 过斜坡段 ---------- */
#define ROUTE_5_TO_RAMP_MM         /*893*/   895    /* 斜坡前直行距离 */
/* ⭐⭐ 2026-10-10(用户要求): 左移 B 段【对半拆开走】, 中间插一次“车头前进” ---------
 * 走法: 左移【前半】 → 车头前进 ROUTE_7_LEFT_B_FWD_MM → 左移【后半】
 *   为什么中间要往前顶一下: 一次左移 871mm 车身会被甩/前后漂一点, 在中间往前
 *   顶一下能把车“顺”回来(顺带修正左移时攒下的前后偏差), 后半段的左移更直。
 * 下面 _B_MM 仍然是【整段总长】(871), 前后半段由 _HALF / _HALF2 自动推出
 *   ⇒ 以后只想改“一共左移多远”, 改总长这一个数就行, 不用同时改两个半段。
 * ⭐⭐ 2026-10-10(用户要求): 现在【默认不再拆半】—— 中间那一步"车头前进"也去掉,
 *    整段【一条路走完】(开关 = 下面的 ROUTE_7_LEFT_B_SPLIT_ENABLE, 当前 = 0)。
 *    老走法(前半 → 前进 → 后半)的代码与三个宏都保留, 把开关置 1 就能切回去。
 * 对应状态:
 *   开关=0: STATE_7_MOVE_LEFT_B(整段) → STATE_7A_INTERMEDIATE_STOP(停顿 + 转正)
 *           (STATE_7B_MOVE_FWD_MID / STATE_7_MOVE_LEFT_B2 不会被进入)
 *   开关=1: STATE_7_MOVE_LEFT_B(前半) → STATE_7B_MOVE_FWD_MID(前进)
 *           → STATE_7_MOVE_LEFT_B2(后半) → STATE_7A_INTERMEDIATE_STOP(停顿 + 转正)
 * ⚠️ 底盘到位死区 ≈ CH_POS_THRESHOLD_COUNT(30 计数) ≈ 4.5mm ⇒
 *    前进的【实际位移 ≈ 本值 − 4.5mm】: 填 20 → 实际约 15.5mm; 别小于 6。 */
#define ROUTE_7_LEFT_B_MM          /*885 */  878   /* 左移 B 段【总长】(开关=0 时整段就走它) */
#define ROUTE_7_LEFT_B_SPLIT_ENABLE   0   /* 0 = 一整段左移(★当前); 1 = 前半→车头前进→后半 */
#define ROUTE_7_LEFT_B_HALF_MM     (ROUTE_7_LEFT_B_MM / 2)                      /* 前半(仅"拆半走法"用) */
#define ROUTE_7_LEFT_B_FWD_MM      20                                           /* 中间那一步: 车头前进 20mm(仅"拆半走法"用) */
#define ROUTE_7_LEFT_B_HALF2_MM    (ROUTE_7_LEFT_B_MM - ROUTE_7_LEFT_B_HALF_MM)  /* 后半(仅"拆半走法"用) */
#define ROUTE_7_LEFT_C_MM           408    /* 左移 C 段 不使用*/

/* ---------- 排爆区走位 ---------- */
#define ROUTE_8_TO_BOMB_AREA_MM     /* 990 */  985/*993*/  /* 直行进入排爆区 */
#define ROUTE_8B_RIGHT_MM           /* 672 */  722    /* 右移微调: 672 → 702 (+30mm)
                                                      * 实测排爆区差一点到中心观看点; 每次 ±10~20mm 微调 */

/* ⭐⭐ 2026-10-11(用户要求): "这几段不要平滑减速, 直接急停" 的开关 --------------
 * 【"这个平滑减速"指什么】= 2026-10-10/11 加的"提前减速"(按剩余距离压期望速度):
 *   短段固定 30mm, ≥300mm 的长段按 CH_SLOWDOWN_LONG_PCT(20%) 拉长(封顶 250mm),
 *   见 Chassis.h 的 CH_SLOWDOWN_* / CH_SLOWDOWN_LONG_*。
 * 【为什么这几段要关掉】慢速收尾时车最容易"被地面拧着走"(速度低时四轮静摩擦/辊子
 *   刮地差异占比大) ⇒ 位置和朝向都容易偏; 不如全速走到位就断输出, 精度交给到位容差。
 * 【取值】0 = 平滑减速(默认, 长段按比例拉长); 1 = 【直接急停】(本段完全不减速);
 *         2 = 只保留最早那个固定 30mm 的小减速(不要"按比例拉长"那一版)。
 * ⚠️ 只改停车曲线, 【不改走的距离】。
 * ⚠️ 急停代价: 全速断输出后会靠惯性再滑一点点; 若日志 MOVE 行出现"冲过头再倒回来"
 *    (e: 反向且不小), 就改成 2 或 0。
 * 【生效位置】分别打在那几段移动之前(见 STATE_3_MOVE_LEFT_A / STATE_7_MOVE_LEFT_B /
 *   STATE_8B_ADJUST_RIGHT), 串口会打一行 "停车方式: 本段【直接急停】…" 供核对。 */

/* ① 扫码后左移 A 段(STATE_3_MOVE_LEFT_A, ROUTE_3_LEFT_A_MM = 557mm):
 *    ⭐⭐ 用户确认(2026-10-11): 就是这一段 —— 不要减速, 直接急停。 */
#define ROUTE_3_LEFT_STOP_MODE      1
/* ② 左移 B 段(STATE_7_MOVE_LEFT_B, ROUTE_7_LEFT_B_MM = 866mm):
 *    ⭐⭐ 用户要求(2026-10-11): 这一段也不要减速, 直接急停。 */
#define ROUTE_7_LEFT_B_STOP_MODE    1
/* ③ 进排爆区前那次右移微调(STATE_8B_ADJUST_RIGHT, ROUTE_8B_RIGHT_MM = 722mm):
 *    ⚠️ 之前按"从扫码区域右移"的说法误判成这一段才开的, 用户确认后已恢复默认
 *       (平滑减速)。要让这一段也急停, 把这里置 1 即可。 */
#define ROUTE_8B_RIGHT_STOP_MODE    0

#define ROUTE_9_TURN_DEG            (0.1)  /* 转向排爆点(相对角度, 负=右转) */
#define ROUTE_9A_LEFT_MM            0      /* 转向后左移微调 */
#define ROUTE_10_APPROACH_MM        0     /* 接近炸弹最后一段直行 */

/* ⭐⭐ 排爆做完 → 打靶走位之前那次航向校正的“挪一小步”(ROUTE_BOMB_AFTER_STEP_MM)----
 * ⚠️ 2026-10-10: 本宏已搬到文件上方【“挪一小步”集中段】(第 ① 组)
 *   —— 全工程所有“校正/移动前先挪一小步”的宏现在都在那一段里, 方便对照着调。 */

/* ⭐⭐ 2026-10-11 新增(用户要求): 排爆后那次航向校正的【停稳 + 二次压正】---------
 * 【为什么要】实测日志发现: 这次校正"判完成"之后, 车的最终朝向偶尔还差 1~2°
 *   (日志: 校正中 yaw +0.4°, 一进右移就变 −1.4°)。原因有两个:
 *     ① 判“转向完成”的条件允许角速度到 CH_TURN_RATE_THRESHOLD(0.2°/周期 = 10°/s)
 *        —— 车还在慢慢转就可能被判“完成”, 而完成动作是 Chassis_Stop() = 断输出
 *        【滑行】(不是刹车) ⇒ 之后还会再滑 1~2°, 而这时已经没人再看它了;
 *     ② 0.4° 这么小的修正, 折算到轮子只有约 9 个计数, 落在底盘到位死区
 *        (CH_POS_THRESHOLD_COUNT=30 计数 ≈ 4.5mm)里面 ⇒ 执行起来是"死区内的
 *        任意值", 不是精确 0°。
 * 【做法】(按用户选择, 最稳、改动最小)
 *      ① 转到 0° → 阻塞等它到位;
 *      ② 原地【停稳】BOMB_AFTER_ALIGN_SETTLE_MS(让滑行/抖动结束);
 *      ③ 【再压一次】转到 0°, 把滑行/陀螺仪跳变攒下的残余补掉, 等它到位后才切走。
 *   ⇒ 相当于"校两次 + 中间停稳", 两次之间的残余不再带到 854mm 长距离里。
 * ⚠️ 只作用于【排爆后那一次】(STATE_11A); 打靶走位中间/最后两次校正不受影响。
 * ⚠️ 每次转向的等待上限 = BOMB_AFTER_ALIGN_TIMEOUT_MS(超时也会往下走, 只打日志)。
 * ⚠️ 期间用 Mission_Coop_Wait 等(照刷 yaw、照收 K230 行), 否则转向环读冻结角度。 */
#define BOMB_AFTER_ALIGN_SETTLE_MS   300    /* 二次压正前的停稳等待(ms); 0 = 不等待 */
#define BOMB_AFTER_ALIGN_TIMEOUT_MS  3000   /* 每次转向的等待上限(ms)。
                                             * ⚠️ 2026-10-11: 5s→3s —— 实车日志显示
                                             * 两次都撞上限(共白等 10s); 正常校正 ≲1s
                                             * 就完成, 超时说明这次根本转不动(见
                                             * TURN_SKIP_DEG), 早停早往下走。 */

/* ⭐⭐ 2026-10-11 新增(用户要求): 【放完球之后那次航向校正】的转向力度加大 ------------
 * 位置 = STATE_11A_HEADING_CORRECT(排爆放球做完 → 进打靶走位之前那次校 0°)。
 * 【为什么要加大】这次校正常常只有 2~4° 的残留:
 *   转向环输出是"每周期往四轮位置目标里加 adj(≤ CH_MAX_TURN_ADJUST = 22 计数)"——
 *   2~4° 折算到轮子行程只有几毫米, 麦轮原地转还要克服侧向刮地的静摩擦
 *   ⇒ 用默认 22 时经常"咬住不动 / 转不到位"(实测日志里这次校正两次都撞超时)。
 *   把这一步的力度单独加大(只作用于这一次转向, 用完即回默认), 就压得过去了。
 * 【取值】BOMB_AFTER_ALIGN_TURN_ADJUST = 35(默认 22 的 ~1.6 倍)。
 *   ⚠️ 别太大: 力度越大越容易冲过头/甩尾(本处已有"停稳 + 二次压正"兜着, 但仍建议 30~45)。
 *   ⚠️ 只影响 STATE_11A 那两次转向(Chassis_SetNextTurnAdjust 是一次性的),
 *      其它地方的转向力度不变; 想整场都加大请改 Chassis.h 的 CH_MAX_TURN_ADJUST。
 *   ⚠️ 置 0 = 不覆盖(用全局默认 22, 即老行为)。 */
#define BOMB_AFTER_ALIGN_TURN_ADJUST   35.0f

/* ⭐⭐ 2026-10-11 新增: 航向校正的“小角度跳过”门槛 (单位: 度) ---------------------
 * 【为什么需要】实车日志(排爆后那次)铁证: 请求只 0.5~1.3° 的校正【根本完不成】——
 *   ① 转向环死区 CH_ANGLE_ERR_THRESHOLD = 0.35°, 所以它一定要真的转那 1.3°;
 *   ② 但 1.3° 折算到轮子只有约 2.3mm 的行程, 麦轮原地转还要克服侧向刮地的静摩擦,
 *      转向环的输出又限幅在 CH_MAX_TURN_ADJUST(22 计数/周期) ⇒ 轮子只在原地刮,
 *      车体不转 ⇒ "剩余角"永不减小 ⇒ 环一直不结束(实测两次各撞 5s 上限);
 *   ③ 期间 PID 积分打满(日志 integ=500, out=192)并把限幅后的输出【每周期累加进
 *      位置目标】⇒ 目标越积越大, 最后轮子突然撒手冲出去(实测 10s 内轮子走了
 *      781mm、车头从 +1.3° 甩到 −18.7°, 用户看到"转了几圈")。
 *   ⇒ 这种"转也转不动、还容易失控"的微校正, 直接跳过最安全:
 *     1~2° 的残留本来就在全程各段校正的残差量级里(日志里各段 yaw 都在 ±1.5° 内)。
 * 【做法】Turn_SkipIfTiny(): |目标 − 当前 yaw| ≤ 本宏 ⇒ 不发转向指令, 只打一行日志。
 *   作用点: Turn_Angle_Compat(0.1f)(全程各处校 0°)、Heading_AlignTo(-90°)、
 *          STATE_11A(排爆后那次)。
 * 【取值】2.0(度)。调大→更省时间但残留角变大; 调小→又把 1~2° 的校正交回给
 *   那个"转不动"的环; 0 = 关闭(恢复"无论如何都发转向")。
 * ⚠️ 想让小角度校正【真的转得动】的正确做法是调大 CH_MAX_TURN_ADJUST(如 22→35,
 *    转向更有力) 或调小底盘到位死区 CH_POS_THRESHOLD_COUNT —— 那是另一条路,
 *    改完可以把这个门槛调小试试。 */
#define TURN_SKIP_DEG                2.0f

/* ⭐⭐ 2026-10-11(用户要求): 航向保持两套方案的总开关 —— 已上移到
 *    MissionControl.h(项目级开关: main.c 里"是否注入 gz / 是否读 GZ / 是否打
 *    GBIAS 日志"也要看它), 这里不再重复定义。详见 MissionControl.h 里的说明。 */

/* ---------- 打靶路线 (2026-10-04 改版后只剩两段真正在用) ----------
 * ⭐ 2026-10-09: 每一段右移都【对半拆开走】(实测: 一次右移 850mm 会攒下约 5° 航向误差,
 *   拆成半段走, 每一小段自己的偏差更小);
 * ⭐⭐⭐ 2026-10-10(用户要求): 【每一半走完都做一次航向校正】= 一共 4 次:
 *   MOVE_A → CORRECT_A → MOVE_A2 → CORRECT_A2 → MOVE_B → CORRECT_B → MOVE_B2 → CORRECT_B2
 *   而且这 4 处【每一处各有自己的“挪一小步”宏】:
 *   ROUTE_12_A_MINSTEP_MM / _A2_ / _B_ / _B2_(各自独立, 初始都是 -15)。
 *   下面 _A_MM / _B_MM 仍是【整段总长】, 前后半段由 _HALF_MM / _HALF2_MM 自动推出
 *   ⇒ 以后只想改“一共走多远”, 改总长这一个数就行, 不用同时改两个半段。 */
#define ROUTE_12_P1_A_MM           /* 850 */  864    /* ⭐ 打靶第 1 段右移【总长】(mm): 排爆结束后从桶边右移这么多 */
/* ⭐⭐ 2026-10-10(用户要求): 打靶走位的分段方式 --------------------------------
 *   放桶结束 → 先航向校准(STATE_11A) → 【右移】→ (中间可能再校一次) → 最后一次校正
 *   → 摆臂(MOVE_C) → 交给打靶视觉对准。
 *   ROUTE_12_SPLIT_ENABLE:
 *      0(★当前) = 【一条路】: 全程 = ROUTE_12_P1_A_MM + ROUTE_12_P1_B_MM(合计约 1708mm);
 *      1        = 老做法(每段再对半拆, 每一半末尾各校一次, 共 4 次)。
 *   ROUTE_12_MID_CORRECT_ENABLE(只在 SPLIT = 0 时有意义):
 *      1(★当前) = 把这条长路【对半走】: 前半走完 →【先挪一小步, 再原地校一次航向】
 *                 → 走后半 → 最后由 CORRECT_B2 再校一次(带停稳) → 摆臂;
 *      0        = 中途不校正, 一条路直接走完(只剩最后那一次校正)。
 *   ⚠️ 两种走法【总距离一样】, 所以终点位置不变, 不用重新标定路线。
 *   ⚠️ SPLIT = 0 时 CORRECT_A / MOVE_A2 / CORRECT_A2 / MOVE_B2 不会被进入;
 *      中间那次校正用的是 CORRECT_B(它的挪一步 = ROUTE_12_B_MINSTEP_MM)。 */
#define ROUTE_12_SPLIT_ENABLE       0
#define ROUTE_12_MID_CORRECT_ENABLE 1

/* ⭐⭐ 打靶走位“中间那次摆正”的【等待上限】(ms) -----------------------------
 * 中间那次校正的动作顺序 = 【先原地摆正车头 → 等它真的转到位 → 再后退 → 再走后半段】;
 * 本宏就是"等它转到位"的最长时间(用协作式等待, 期间照刷 yaw、照收 K230 行)。
 *   2026-10-11(用户要求): 5 秒 → 8 秒(给"车停得比较死、原地转得慢"留余量)。
 * ⚠️ 超时后会【照旧执行后退】—— 而后退会取消没转完的转向, 于是车会带着偏差走完
 *    后半段。日志里看这一行判断有没有超时:
 *      "打靶走位(中间): 车头已摆正 yaw=0.3°(目标 0°, 耗时 1240ms)…"
 *      ⇒ 耗时接近本宏上限 **且 yaw 不是 0** = 这次没转到位, 要查陀螺仪/卡住。
 * ⚠️ 调大 = 更不容易"带着偏差走", 但真出问题时也会白等更久; 建议 5000~10000。 */
#define ROUTE_12_MID_ALIGN_TIMEOUT_MS   8000
#define ROUTE_12_P1_A_HALF_MM      (ROUTE_12_P1_A_MM / 2)                      /* 第1段 前半(走完由 CORRECT_A 校正) */
#define ROUTE_12_P1_A_HALF2_MM     (ROUTE_12_P1_A_MM - ROUTE_12_P1_A_HALF_MM)  /* 第1段 后半(走完由 CORRECT_A2 校正) */
#define ROUTE_12_P1_B_MM            /*860 */  844    /* ⭐ 打靶第 2 段右移【总长】(mm): 航向校正完再右移这么多 */
#define ROUTE_12_P1_B_HALF_MM      (ROUTE_12_P1_B_MM / 2)                      /* 第2段 前半(走完由 CORRECT_B 校正) */
#define ROUTE_12_P1_B_HALF2_MM     (ROUTE_12_P1_B_MM - ROUTE_12_P1_B_HALF_MM)  /* 第2段 后半(走完由 CORRECT_B2 校正) */

/* "一条路"走法(ROUTE_12_SPLIT_ENABLE = 0)用的全程总长与前后半段 -------------
 *   ⚠️ 只想改"一共右移多远" → 改上面两个总长(A/B)就行, 无需动这里。 */
#define ROUTE_12_TOTAL_MM          (ROUTE_12_P1_A_MM + ROUTE_12_P1_B_MM)
#define ROUTE_12_TOTAL_HALF_MM     (ROUTE_12_TOTAL_MM / 2)                       /* 前半(走完做中间那次校正) */
#define ROUTE_12_TOTAL_HALF2_MM    (ROUTE_12_TOTAL_MM - ROUTE_12_TOTAL_HALF_MM)  /* 后半(走完做最后一次校正) */

/* ⭐⭐ 打靶走位: 右移段的“向后分量”比例(无量纲, 0 = 关掉 = 普通右移) --------
 * 用法: 打靶走位这 4 个右移小段(MOVE_A/A2/B/B2)走的是
 *       Chassis_Move_Right_WithBack(距离, 本宏)
 *       ⇒ 每右移 1mm 同时后退 本宏 mm(0.085 ⇒ 后退 8.5%), 全程斜着补。
 * 为什么改成“全程补”而不是“段末挪一步”:
 *   右移时车会一直往上(车头前方)漂, 漂移量≈右移距离 × 固定比例。
 *   段末一次性挪回来是“折线”, 补偿量对不上就变成超调/欠补(实测一直在 ±50 之间试,
 *   就是把总漂移硬怼回去); 摊到全程则“右移多远就补多少”, 走的是直的。
 * ⚠️ 只作用于打靶走位这 4 段: 收尾的 PART2_MOVE_A 用的是普通 Chassis_Move_Right。
 * ⚠️ 正 = 向后补(车头后退方向); 若实车发现是【向下偏】(漂到车头后方), 填负数。
 * ⚠️ 换算: 这 4 段右移总长 = 864 + 844 = 1708mm ⇒ 0.085 相当于全程一共后退约
 *      1708 × 0.085 ≈ 145mm(原来是段末“一次性挪”: 现在是 4 处 × |ROUTE_12_A2_MINSTEP_MM| = 60mm)。
 *      想和当年 “-50 挪两次 = 100mm” 等效, 比例填 100/1708 ≈ 0.059。
 *      实车按“整段走完偏了多少”微调这一个数即可。 */
#define ROUTE_12_RIGHT_BACK_COMP    0.0f

/* ⭐⭐ 打靶走位 4 处航向校正的“挪一小步” -------------------------------------
 * ⚠️ 2026-10-10: 这些宏【统一放在文件上方“⭐⭐⭐ 全工程【挪一小步】的宏”那一段(第 ② 组)】:
 *    ROUTE_12_A/A2/B/B2_MINSTEP_MM、ROUTE_12_MINSTEP_ENABLE、ROUTE_12_A2/B2_STEP_MODE、
 *    ROUTE_12_YAW_LIMIT_DEG、ROUTE_12_A2/B2_YAW_{POS,NEG,MID}_MM、以及那段 #error 防呆。
 *    这里【不再重复定义】(原来是两处都定义, 后面的会覆盖前面的, 容易改错地方)。 */

#define ROUTE_12_P1_C_MM            400    /* (未使用: MOVE_C 改成只摆 TARGET_LOOK, 不再走位) */
#define ROUTE_12_TURN_A_DEG         400    /* (未使用: TURN_A/TURN_B 已不在流程里) */
#define ROUTE_12_P2_A_MM           /* 602 */  625   /* ⭐ 打靶收尾第 1 段 = 右移到【拐角】(mm):
                                            * ⚠️ 2026-10-07 实测: 这段右移到位后小车正好到车场
                                            * 拐角, 那里原地转 90° 的余量才充足(转就紧跟在它后面)。
                                            * 590 → 560: 原来会略微冲过拐角, 减 20mm 让它停在
                                            * 余量最足的位置。想微调就改这一个数(每次 ±10mm)。 */

/* ⭐ 打靶收尾第 2 段(mm) —— 作用状态 = STATE_12_PART2_MOVE_BACKWARD。
 * 完整顺序(⚠️ 转 90° 必须排在最后, 理由见上):
 *      打靶视觉结束 → ①右移 ROUTE_12_P2_A_MM(到【拐角】)
 *      → ②后退 ROUTE_12_P2_BACK_MM
 *      → ③航向校正到 0°
 *      → ④进救援阶段: 右转90° → 校准到 -90° → 原地停稳 3s
 *      → ⑤右移 ROUTE_14_TO_HOSTAGE_MM(进救援区, ⭐ 现在拆两段、中途校一次)
 *        → 停等3s → 摆 HOSTAGE_LOOK
 * ⚠️ 走到②时车头还是 0° ⇒ 用的是“后退”(Chassis_Move_Backward)。 */
#define ROUTE_12_P2_BACK_MM         25
                                           
#define ROUTE_12_P2_B_MM            200    /* (已废弃) */
#define ROUTE_12_P2_C_MM            200    /* (已废弃) */

/* ⭐ 打靶走位: 第 2 段右移结束后的“停车停稳延时”(ms)
 * 作用状态 = STATE_12_PART1_CORRECT_B2(走位链最后一次航向校正)。
 * 为什么需要: 底盘“到位/转向完成”判定成立之后车其实还在轻微晃, 这里多等一段
 *   再让臂/摄像头伸出去 —— 免得底盘刹车/麦轮摆动还没停, 机械臂就带着摄像头一起晃。
 * 建议 0(关闭) ~ 1500; 默认 800。 */
#define TARGET_STOP_SETTLE_MS   200

/* ---------- 救援(掉头) ---------- */
#define ROUTE_14_TO_HOSTAGE_MM      /* 950 */  985  /* 救援前横移距离(mm)。⚠️ 2026-10-07 起:
                                             * 车头先【右转 90°】, 这一段的动作由
                                             * 原来的“后退”改成“右移”(距离不变) */
/* ⭐⭐ 2026-10-09 新增(用户要求): ④ 右移进救援区【中途插一次航向校准】 ------------
 * 目的: 965mm 一路走到底才校准, 中间攒的航向误差没人收; 现在在半路先校一次
 *       (还没到救援区), 走法/做法和前面几处(②⑨⑪ 以及打靶走位那几处)完全一样 =
 *       【先挪一小步 → 再按绝对角校准到 -90°】。
 * 拆法(⚠️ 总长只在那一个宏里, 别在两处各写一半):
 *      第一段 = ROUTE_14_MID_MM, 校准后第二段 = 总长 − 第一段
 *   ⇒ 只想改“一共右移多远”就改 ROUTE_14_TO_HOSTAGE_MM; 想挪校准点改 ROUTE_14_MID_MM。
 * 对应状态: STATE_15B_RESCUE_APPROACH_RIGHT(第一段)
 *        → STATE_15B2_RESCUE_MID_CORRECT(挪一步 + 航向校准)
 *        → STATE_15B3_RESCUE_APPROACH_RIGHT2(第二段) → ⑤ 停等。 */
#define ROUTE_14_MID_MM        (ROUTE_14_TO_HOSTAGE_MM / 2)                /* 第一段(≈482mm): 走完在这儿校一次 */
#define ROUTE_14_MID2_MM       (ROUTE_14_TO_HOSTAGE_MM - ROUTE_14_MID_MM)  /* 第二段(≈483mm): 校准后接着右移进救援区 */

/* ⭐ 中途那次航向校准【之前】的“挪一小步”(ROUTE_RESCUE_MID_STEP_MM) -------------
 * ⚠️ 2026-10-10: 本宏【已搬到文件上方“挪一小步”集中段(第 ③ 组)】, 值 = -15
 *    (车头【后退】15mm); 这里不再重复定义 —— 两处都定义时【后面那个会生效】,
 *    以前就出过“在集中段改了值却不起作用”的事故。 */

/* ⭐ ③ 停稳之后、④ 右移进救援区之前那次【纯航向校正】的“挪一小步” -------------
 * (ROUTE_RESCUE_PRE_RIGHT_STEP_MM)
 * ⚠️ 2026-10-10: 本宏(值 = 0 = 不挪步, 只校航向)【已搬到文件上方“挪一小步”集中段
 *    (第 ③ 组)】(原说明也一起搬过去了), 这里不再重复定义。 */
// #define ROUTE_15_TURN_180_DEG       (-91) /* 原地掉头 180° */

/* ⭐⭐ 2026-10-07 救援阶段改为「先掉头, 再横移进救援区」-------------------------
 * 关键点: 车头【右转 90°(顺时针)】之后, 车体的【右】方向 = 原来的【后】方向
 *   ⇒ 原来所有“后退”的路线段, 改成“右移”就能保持行走轨迹完全不变,
 *     只是车身姿态转了 90°(机械臂/摄像头的朝向随之改变)。
 * 所以下面这些距离宏的【数值不用动】, 方向由状态机里
 *   Chassis_Move_Backward → Chassis_Move_Right 决定(见 Mission_Update)。
 * ⚠️ 航向基准: 掉头后必须把航向基准也改成 RESCUE_HEADING_DEG(-90°),
 *    否则后续横移的“航向保持”会按旧基准(0°)把车硬拽回原朝向 ——
 *    90° 掉头就白做了(见 Heading_AlignTo())。 */
#define RESCUE_HEADING_DEG          (-90.1f) /* 救援阶段车头朝向(绝对角, 度):
                                             * 正=逆时针/左转, 负=顺时针/右转
                                             * ⇒ -90 = 右转 90°。
                                             * 后续所有平移的航向基准 + 航向校准
                                             * 都用它(原来是统一的 0°)。 */

/* ---------- 救援阶段 (2026-10-07 改为掉头版) ----------
 *  ①车头右转 90° → ②航向校准到 -90° → ③原地停稳 RESCUE_ALIGN_SETTLE_MS(3s)
 *  → ④右移 ROUTE_14_TO_HOSTAGE_MM(进救援区, ⭐ 2026-10-09 拆成两段: 中途插一次
 *       “挪一步 + 航向校准”, 见 ROUTE_14_MID_MM / ROUTE_RESCUE_MID_STEP_MM)
 *     → ⑤停等 RESCUE_STOP_WAIT_MS
 *  → ⑥摆 HOSTAGE_LOOK → ⑦视觉对准 + 抓取
 *  → ⑧右移 ROUTE_17_RIGHT_B_MM → ⑨航向校准
 *  → ⑩右移 ROUTE_18_RIGHT_C_MM → ⑪航向校准 → ⑫停下
 *  ⭐ ⑦ 抓取 (2026-10-06): 底盘不动, 靠底座 ID1 小步转对准人质; 对准完
 *     (收到 C, 或步数/时长超时兜底)按 ID1 的【累计偏移量】选左/中/右一侧,
 *     再执行该侧的【抓取 → 抱紧 → 抬起】三个姿态(共 9 个姿态, 待示教标定,
 *     见 Arm_Start_Rescue_Grab / Arm_Start_Rescue_Retract)。
 *  ⚠️ 车头转了 90° 之后, 机械臂那几套姿态(ID1 底座尤其)需要重新示教标定。 */
#define ROUTE_16_RIGHT_A_MM         300    /* (未使用: 该状态已改成“只摆 HOSTAGE_LOOK”) */
#define ROUTE_17_RIGHT_B_MM         650    /* ⭐ 抓完后第 1 段右移(mm)(原来叫“后退”) */
#define ROUTE_18_RIGHT_C_MM         /* 890 */  964/*934*/    /* ⭐ 抓完后第 2 段右移(mm)(原来叫“后退”) */

/* ⭐⭐ 2026-10-11(用户要求): 救援【撤退段】降速 -----------------------------------
 * 【为什么要】抱上人质之后重心偏了, 350mm/s 横移时辊子刮地扰动大、段末单对角甩尾也大
 *   (实测段末甩 +5.9°) ⇒ 从【抓完人质那一步(⑧)】起把全场车速降到 250, 走到终点(⑫)再恢复。
 * 【作用范围】STATE_17 进入时降速 → STATE_19 进入时恢复, 中间整段(⑧ + ⑩ + 中途校正 +
 *   那 20mm 前进)全部按 250 走; 恢复是为了"下次任务 / 手动模式"从 350 开始。
 * 【怎么关掉】把 ROUTE_RESCUE_RETURN_SPEED_MMPS 改成和 ROUTE_DEFAULT_SPEED_MMPS 一样的值
 *   (= 350)即可, 两个宏相等时降速/恢复都是同一速度, 等于没改。
 * ⚠️ 不会再影响转向速度: 250mm/s → 33.1 计数/周期, 而转向限幅 CH_MAX_TURN_ADJUST = 22.0
 *   (见 Chassis_SetMaxSpeed 的换算), 33.1 > 22 ⇒ 转弯力度照旧。
 * ⚠️ 与 main.c 里 Chassis_SetMaxSpeed(350) 必须一致 —— 改一个就一起改。 */
#define ROUTE_RESCUE_RETURN_SPEED_MMPS   250   /* 撤退段车速(mm/s); 与下面相等 = 关闭降速 */
#define ROUTE_DEFAULT_SPEED_MMPS         350   /* 默认车速(mm/s), 与 main.c Chassis_SetMaxSpeed(350) 一致 */

/* ⭐⭐ 2026-10-10(用户要求): 救援【抓完人质 → 终点】的走法 ---------------------
 *   【1(★当前)】抓完人质 → ⑧【先原地校一次航向(不挪步)】→ ⑩【一段走到终点】→ ⑫停下:
 *        · ⑧ 只校航向: Heading_AlignTo(RESCUE_HEADING_DEG = -90°);
 *          校前那一步挪多少 = ROUTE_RESCUE_FWD_MM(当前 = 0 ⇒ 不挪, 只原地摆正)。
 *        · ⑩ = ROUTE_17_RIGHT_B_MM + ROUTE_18_RIGHT_C_MM【合并成一次连续右移】,
 *          中途不再停车、不再校正航向; 走完直接进 ⑫ 停车 → 任务完成。
 *   【0】老做法: ⑧右移 → ⑨校 → ⑩右移 → ⑪校 → ⑫停(两段各自走完 + 各校一次)。
 *   ⚠️ 两种走法【总距离一样】(⑧ 的 650mm 只是挪进 ⑩ 里一起走)。
 *   ⚠️ 置 1 时 STATE_17A/STATE_18A 不会被进入 ⇒ ROUTE_RESCUE_LAST_STEP_MM(⑪ 的挪步)
 *      用不到; ROUTE_RESCUE_FWD_MM 仍用于 ⑧ 的"校前挪步"。 */
#define RESCUE_TAIL_ONESHOT         1

/* ⭐⭐ 2026-10-11 新增(用户要求): 抓完人质后【最后这一段右移】带一个“车头左方分量” -------
 * 【作用位置】= 抓完人质 → 一路走到底的那一次连续右移(= ⑧+⑩ 的距离, 见 RESCUE_TAIL_ONESHOT),
 *   以及老做法下 ⑩(STATE_18_RESCUE_RIGHT_C)那一段 —— 也就是"最后一段"。
 * 【怎么补】每右移 1mm 同时向【车头左方】走 RESCUE_TAIL_LEFT_COMP mm。
 * ⚠️⚠️ 先看方向关系(不然会调反):
 *     救援段车头已右转 90° ⇒ 车体的 6 个方向在场地里是:
 *        车头【前】 = 场地右       车头【后】 = 场地左
 *        车头【右】 = 场地后       车头【左】 = 场地前
 *     而这一段走的就是【右移】= 车头右方。
 *   ⇒ 【左】和【右】是同一根轴的两端 ⇒ “加左分量” = 这段右移【少走】那么多,
 *      不是横着偏出去。所以本宏的净效果 = 实际右移距离 × (1 − 本值)。
 *      例: 0.025 + 1614mm ⇒ 少走 ≈40mm, 实际右移 ≈1574mm。
 * 【已关闭(2026-10-11 用户实测)】这个固定比例偏差会让【车头歪】:
 *   横移时左右两侧轮子的负载/打滑本来就不对称, 硬扣掉一段固定距离(0.038×1614≈61mm)
 *   相当于把整段变形, 车头会跟着偏 —— 实测不如【用两次航向校正把偏差就地清掉】。
 *   ⇒ 现在本宏 = 0(不补), 改用: 抓完人质先校一次 + 走到一半再校一次
 *      (见 RESCUE_TAIL_MID_CORRECT_ENABLE / RESCUE_TAIL_MID_STEP_MM)。
 *   改回非 0 就恢复"按比例少走"的老行为(仅作备选)。
 * 【取值】正 = 往车头【左】补(少走); 负 = 往车头【右】补(多走); 0 = 不补(★当前)。 */
#define RESCUE_TAIL_LEFT_COMP       0.0f     /* 每右移 1mm 同时向车头左方走这么多 mm(= 少走) */
/* ⭐ 另一个旋钮(2026-10-11 同批加的): 沿【车头前方】的分量 —— 这个才是真正横着偏出去
 *    (车头前方 = 场地右, 与右移垂直 ⇒ 走出来是一条斜线, 终点会横移 距离×本值)。
 *    ⚠️ 默认 0 = 不使用; 若实测需要的是"横着偏 40mm"而不是"少走 40mm", 就把 LEFT 置 0、
 *       本值置 0.025(正 = 往车头前方/场地右偏, 负 = 往车头后方/场地左偏)。 */
#define RESCUE_TAIL_FWD_COMP        0.0f     /* 沿车头前方的分量比例; 0 = 关 */
#define RESCUE_TAIL_TOTAL_MM        (ROUTE_17_RIGHT_B_MM + ROUTE_18_RIGHT_C_MM)

/* ⭐⭐ 2026-10-11(用户要求): 最后一段的【中点】插一次“航向校正 + 往前进 20mm” ----
 * 【最终定稿的做法(2026-10-11 用户确认)】抓取人质后 → ①【校一次航向】→
 *   ② 走到【一半】再【校一次航向 + 往前进 20mm】→ ③ 剩下的一次性冲到终点。
 *   (之前试过"全程按比例补左分量"作为终点偏差的补偿, 实测会让车头歪 ⇒ 已关闭,
 *    见 RESCUE_TAIL_LEFT_COMP = 0; 现在改成"两次校正", 偏差就地清掉、不残留。)
 * 【为什么中点要校】这一段(⑧+⑩ 合并 ≈1614mm)是全程最长的一次连续右移:
 *   横移时辊子侧向刮地本来就会让车头慢慢偏, 走 1.6m 攒下的偏差比短段大;
 *   且这一段的偏差【没有后续校正】兜底(走完就 🏁 结束), 所以在中点清一次。
 * 【做法】把这段【对半拆成两段走】, 中间做:
 *     ① 阻塞等前半段走完(超时也往下走, 但会强制停车, 见 RESCUE_TAIL_HALF_TIMEOUT_MS);
 *     ② Heading_AlignTo(RESCUE_HEADING_DEG = -90°): 按绝对角把车头摆正
 *        —— 顺带【重新同步航向基准】(Chassis_SetHeadingRef), 后半段按新基准走直线;
 *        ⚠️ 偏差 ≤ TURN_SKIP_DEG(2°) 时它会自动跳过转向, 只同步基准(见该宏);
 *     ③ Route_MinStep(RESCUE_TAIL_MID_STEP_MM = +20): 【往车头前方】挪 20mm
 *        (此刻车头 -90° ⇒ 车头前方 = 场地右方, 与"右移"垂直), 把轮子带活,
 *        同时也是一次位置微调(Route_MinStep 内部阻塞到位, 自带日志)。
 *     ④ 剩下的路【一次性冲到终点】(不再停车、不再校正)。
 * 【怎么关】RESCUE_TAIL_MID_CORRECT_ENABLE 置 0 ⇒ 回到"一段走到底, 中途不停不校正"。
 * ⚠️ 只在 RESCUE_TAIL_ONESHOT = 1(当前)这条走法里生效 —— 老做法(两段各自校一次)
 *    本来就有 ⑪ 那次校正, 不需要这个。
 * ⚠️ 总距离不变: 前半 + 后半 = 原来那一段(现在 LEFT_COMP = 0 ⇒ 就是 ⑧+⑩ 全长的对半)。 */
#define RESCUE_TAIL_MID_CORRECT_ENABLE  1     /* 1 = 中点插一次校正 + 前进一小步; 0 = 一段走到底 */
#define RESCUE_TAIL_MID_STEP_MM         20    /* 中点校正【之后】挪的一小步(mm):
                                               * ⭐ 用户定值 20(2026-10-11);
                                               * 正 = 车头前进(此处 = 场地右方), 负 = 车头后退 */
#define RESCUE_TAIL_HALF_TIMEOUT_MS     15000 /* 等【前半段】走完的上限(ms):
                                               * 半个长段约 3~4s, 15s 是兜底(槽糕情况不卡死) */
#define RESCUE_TAIL_MID_ALIGN_TIMEOUT_MS 5000 /* 中点那次转向的等待上限(ms); 超时会强制停车 */
/* ⭐ 救援段(阶段四)的“挪一小步” -------------------------------------------------
 * ⚠️ 2026-10-10: 这些宏【统一放在文件上方“⭐⭐⭐ 全工程【挪一小步】的宏”那一段(第 ③ 组)】:
 *      ②  ROUTE_RESCUE_AFTER_TURN_MM
 *      ⑥  ROUTE_RESCUE_GRAB_BACK_MM
 *      ③' ROUTE_RESCUE_PRE_RIGHT_STEP_MM
 *      ④' ROUTE_RESCUE_MID_STEP_MM
 *      ⑨  ROUTE_RESCUE_FWD_MM
 *      ⑪  ROUTE_RESCUE_LAST_STEP_MM
 *    (连同“车头 -90° ⇒ 车头前进 = 场地右”的方向说明、死区说明一起搬过去了。)
 *    ⭐ ⑥ 那处 = ROUTE_RESCUE_GRAB_BACK_MM = -15(车头后退 15mm): 集中段里也注明了
 *       它是本家族【唯一不接转向】的一处 —— 后面跟的是“摆臂 + 视觉识别”,
 *       代码也不走 Route_MinStep, 是直接发 Chassis_Move_Backward。
 *    这里【不再重复定义】(原来是两处都定义, 后面的会覆盖前面的 —— 实测就出过
 *    “在集中段改成 -15 却不起作用”这种事故)。 */

#define ROUTE_19_RIGHT_D_MM         800    /* (未使用) */
#define ROUTE_21_RIGHT_E_MM         0//300    /* (未使用) */
#define ROUTE_22_RIGHT_F_MM         0//500    /* (未使用) */

/* =====================================================================
 * ⭐ 机械臂姿态表 (实测标定值, 0~4095)
 *   每个姿态 5 个数, 顺序固定:
 *     [0]=ID1 底座(左右转)  [1]=ID2 大臂(抬/降)  [2]=ID3 副关节
 *     [3]=ID4 腕部(姿态)    [4]=ID5 夹爪(数值小=张开, 数值大=闭合)
 *   数值是舵机角度码: 0=0°, 4095≈360°; 4096 步 = 一圈。
 *   标定/微调提示:
 *     - 只调一个姿态的一个舵机时, 只改表里那一行那一列, 别的行不要动。
 *     - ⚠️ 夹爪(第 5 列)别一次调太大, 超出物理行程会堵转发热。
 *     - ⚠️ 相邻两姿态的差值别超过 2048(半圈): 飞特舵机按“最短路径”
 *       转向, 超过 2048 会朝反方向甩近一整圈(撞车架/摄像头)。ServoArm.c
 *       里有可选的大步长保护(默认关闭), 详见那里的 SERVO_LONG_JUMP_* 宏。
 *   ⚠️ HOME(复位)不在这里: 唯一来源是 ServoArm.c 的 SERVO_POS_HOME,
 *      避免两处初始位不一致(见 ArmPose_Ptr())。
 * 
 * ===================================================================== */
//id2限幅（50往前~2300往后）id3限幅（700往下~3100往上）id4限幅（900往下~3010往上）id5限幅（25张开~600闭合）
static uint16_t s_arm_pose_table[ARM_POSE_COUNT][SERVO_COUNT] = {
    /*  名称             ID1   ID2   ID3   ID4   ID5  */
    {  2052, 2274, 810, 1413, 93  },   /* HOME          复位(运行时取自 ServoArm, 此行不生效) */
    {  2052,  656,1760, 2185, 93  },   /* SCAN          扫码: 车停稳后伸臂给摄像头
                                       * ⭐ 2026-10-07: ID4 由 2175 → 2185(向上 +10, 用户要求)
                                       *    —— 三个扫描角度(SCAN ± SCAN_ID4_DELTA)是
                                       *    本值加 s_scan_id4_off[]{0,-60,+60} 算出来的,
                                       *    所以这三个角度【整体】跟着抬 10。
                                       *    只想动某一个角度 → 改 s_scan_id4_off[]。 */
    {  2052, 1843, 878, 1834, 93  },   /* SCAN_RESET    扫码之后复位: 扫到码后把机械臂收回 */
    {  2047, 1661,1359,  920, 93  },   /* BALL_LOOK     看球: 摄像头对准小球(抓球前对准) */
    {  2047,  452,1855, 1480, 93  },  /* BALL_PRE       抓夹移动到小球前 */
    {  2047,  452,1855, 1480, 600 },   /* BALL_CLOSE    夹爪夹紧小球 */
    {  2047, 1160,1795, 1748, 600 },   /* BALL_LIFT     抓到小球后大臂抬起 */

    // { 7,  886,1872, 1650, 600 },   /* BUCKET_CARRY  携带姿态: 端着球, 底盘移动到另一侧 */
    // { 7, 1342,1645, 970, 600 },   /* BUCKET_LOOK   看桶: 摄像头对准球桶(放置前对准) */
    // { 0,  197,1719, 2594, 600 },   /* PLACE_PRE     机械臂移动到放置小球的位置 */

    { 40,  886,1872, 1650, 600 },   /* BUCKET_CARRY  携带姿态: 端着球, 底盘移动到另一侧 */
    { 40, 1342,1645, 970, 600 },   /* BUCKET_LOOK   看桶: 摄像头对准球桶(放置前对准) */
    { 10,  197,1719, 2594, 600 },   /* PLACE_PRE     机械臂移动到放置小球的位置 */
    { 10,  197,1719, 2594, 93  },   /* PLACE_OPEN    夹爪松开(放球) */
    { 10, 1285,2155, 2234, 93  },   /* PLACE_LIFT    放置完之后大臂抬起 */
    {  2052, 1925, 988, 1463, 93  },   /* TARGET_READY  转动到准备识别靶子的位置 */
    {  2052,  585,1899, 2180, 93  },   /* TARGET_LOOK   识别靶子: 摄像头对准靶子 */
    {  2052,  565,1889, 2175, 93  },   /* TARGET_FIRE   激光发射位 */
    {  2052, 1843, 878, 1834, 93  },   /* TARGET_LIFT   发射完激光后大臂抬起(与 LOOK 同值) */
    {  2052, 1942,1327,837, 93  },   /* HOSTAGE_LOOK  识别人质: 摄像头对准人质(中间目标)
                                       * ⭐ ID1 = 2052 = 救援【对准的基准】(Arm_Id1Reset /
                                       *    慢速巡视 / 小步对准都以它为偏移 0 点)。
                                       *    必须 ≈ RESCUE_ID1_MID_POS(现在 2059 = 中间抓取位,
                                       *    两者是同一物理方向的两次示教值, 相差 7 码 ≈ 0.6°),
                                       *    否则偏移量和抓取位(左/中/右)的选择会偏。
                                       * ⚠️ 本行 ID2~ID5 还是旧的示教值;
                                       *    若这轮重新示教过“识别人质”姿态, 请把实测的
                                       *    4 个数填进来(它们只在 STATE_16 的整表下发时
                                       *    起作用; 小步转 ID1 时只写 ID1, 不受影响)。
                                       * ⚠️ 下面 HOSTAGE_PRE/CLOSE/LIFT 三行已不再被代码
                                       *    引用(已被 9 个 GRAB 姿态取代), 留着备用。 */
    { 3023,  484,1868, 1754, 93  },   /* HOSTAGE_PRE   机械臂准备抱人质 */
    { 3023,  484,1868, 1754, 600 },   /* HOSTAGE_CLOSE 抱紧人质 */
    { 3015, 1892,1040, 2037, 600 },   /* HOSTAGE_LIFT  抱起人质后大臂抬起 */
    /* ⭐ 救援抓取位(三选一, 2026-10-06 新增) —— 初值全部 = HOSTAGE_PRE,
     *    ⇒ 改装后行为与以前【完全一致】, 不影响现有比赛流程。
     *    用示教模式(长按 KEY2 进入, KEY2 切到该姿态, KEY1 读出当前值)标定:
     *      GRAB_L: 人质偏左  (ID1 偏移 ≥ +RESCUE_GRAB_MID_RANGE)
     *      GRAB_M: 人质居中  (|ID1 偏移| ≤ RESCUE_GRAB_MID_RANGE)
     *      GRAB_R: 人质偏右  (ID1 偏移 ≤ -RESCUE_GRAB_MID_RANGE)
     *    ⚠️ 这三行只决定【抓取姿态】; 合夹爪时 ID5 会自动按 HOSTAGE_PRE→
     *       HOSTAGE_CLOSE 的差值 +507 变成闭合值(见 Arm_Start_Rescue_Grab)。
     *    ⚠️ 第 5 列(ID5)留 93 就行: 示教时舵机是卸力的, 读出的 120/121 是
     *       夹爪“张开”的机械位置; 写 93 时 93+507=600(正好是闭合上限),
     *       写 120 时 120+507=627 会被限幅成 600 —— 两者结果完全一样。 */
    {  1561,570,1850,1674, 93  },   /* HOSTAGE_GRAB_L 抓取位[左]
                                    * ⚠️ 第 1 列(ID1) 2026-10-07 起【运行时不生效】:
                                    *    对准完成后会把当前 ID1 锁存进去(见
                                    *    s_rescue_id1_lock / Arm_GotoRescuePoseKeepId1),
                                    *    下发时用锁存值, 这里的 1561 只在“没锁过”
                                    *    (如测试模式)时兵底。真正决定抓取姿态的是 ID2~ID5。 */
    {  2059,842,1632,1416, 93  },   /* HOSTAGE_GRAB_M 抓取位[中] */
    {  2503,772,1628,1759, 93  },   /* HOSTAGE_GRAB_R 抓取位[右] */
    /* ⭐ 各方向的【抱紧】与【抬起】(2026-10-06 新增, 待示教填入) ----------
     * 初值来源(保证与拆成 9 个之前的行为完全一致):
     *   GRAB_x_CLOSE = GRAB_x + (HOSTAGE_CLOSE - HOSTAGE_PRE)
     *                  = {ID1-8, ID2, ID3, ID4, ID5+507}
     *   GRAB_x_LIFT  = HOSTAGE_LIFT = {1190,1892,1040,2037,600}
     * 示教方法: 先让臂摆到 GRAB_x(可以进示教模式手动掰), 合上夹爪后
     *   KEY1 读出 → 填 GRAB_x_CLOSE; 再抱起人质读一次 → 填 GRAB_x_LIFT。
     * ⚠️ 抱紧行通常只比抓取行多一个“ID5 由 93 → 600”, 其余 4 个列最好相同
     *    (差得多了会在合夹爪的那一瞬间把已经对准的位置带偏)。
     * ⚠️ 若实测“抱紧/抬起”的整臂动作很大, 记得把下面 s_arm_pose_time 里
     *    对应行的时间从 2000/2500 调大(如 3000), 否则会抰得太猛。 */
    {  1561,570,1850,1674, 600 },   /* HOSTAGE_GRAB_L_CLOSE 抱紧[左] */
    {  2059,842,1632,1416, 600 },   /* HOSTAGE_GRAB_M_CLOSE 抱紧[中] */
    {  2503,772,1628,1759, 600 },   /* HOSTAGE_GRAB_R_CLOSE 抱紧[右] */
    {  1561,1753,1205,1656, 600 }, /* HOSTAGE_GRAB_L_LIFT  抬起[左] */
    {  2059,1753,1205,1656,600 }, /* HOSTAGE_GRAB_M_LIFT  抬起[中] */
    {  2503,1753,1205,1656, 600 },  /* HOSTAGE_GRAB_R_LIFT  抬起[右] */
    /* ⭐ 救援回程姿态 (2026-10-06 新增, 用户示教值)
     *   抱起人质 + 大臂抬起之后执行这张, 把臂收到“适合带着人质横移”的位置,
     *   三个方向共用(见 Arm_Start_Rescue_Return)。 */
    { 2052,1753,1205,1656, 600 }  /* HOSTAGE_RETURN 抱起后回程姿态 */
};

/* ⭐ 每个舵机的【物理行程限幅】(角度码) —— 只给上面的抓取位做安全钳位用。
 * 来源: 姿态表上方的实测注释
 *       (id2 50~2300  id3 700~3100  id4 900~3010  id5 25张开~600闭合;
 *        底座 ID1 不限, 按 0~4095)。 */
static const uint16_t s_servo_pos_min[SERVO_COUNT] = {    0,   50,  700,  900,  25 };
static const uint16_t s_servo_pos_max[SERVO_COUNT] = { 4095, 2300, 3100, 3010, 600 };

/* ⭐ 每个姿态的“运动时间”(ms): 从上一个姿态走到本姿态用多久。
 *   值大 = 慢而稳; 值小 = 快但容易抖/过冲。想单独调某个动作就改对应行。
 *   下标与 ArmPose_t 一一对应; 下面先给一套默认值, 实测后逐个改。 */
static uint16_t s_arm_pose_time[ARM_POSE_COUNT] = {
    1500,   /* HOME          复位 */
    6000,   /* SCAN          扫码 */
    2500,   /* SCAN_RESET    扫码之后复位 */
    2500,   /* BALL_LOOK     看球 */
    ARM_GRAB_PRE_MS,   /* BALL_PRE      抓夹到小球前(⭐2026-10-11 由 2500 加长, 见 ARM_GRAB_*_MS) */
    ARM_GRAB_CLOSE_MS, /* BALL_CLOSE    夹紧(只有夹爪动; ⭐2026-10-11 由 2000 加长) */
    ARM_GRAB_LIFT_MS,  /* BALL_LIFT     抓完抬起(⭐2026-10-11 由 2500 加长) */
    2500,   /* BUCKET_CARRY  携带姿态 */
    2500,   /* BUCKET_LOOK   看桶 */
    2500,   /* PLACE_PRE     移到放置位 */
    2000,   /* PLACE_OPEN    松开(只有夹爪动) */
    2500,   /* PLACE_LIFT    放完抬起 */
    2500,   /* TARGET_READY  准备识别靶子 */
    2000,   /* TARGET_LOOK   识别靶子 */
    2000,   /* TARGET_FIRE   激光发射位 */
    2000,   /* TARGET_LIFT   发射完抬起。
             * ⚠️ 2026-10-06: 本动作是全场【负载最重】的(ID2 从 585 抬到 1843 =
             *    +1258 码 / ID3 从 1899 转到 878 = -1021 码, 全程顶着重力)。
             *    实测“抬完大臂后机械臂薓下去”, 疑似峰值电流/过热把舵机拉降额。
             *    这里放慢到 6000ms, 并配合 Arm_Start_Target_Lift() 的【分两步】
             *    下发(先 ID1/2/3 再 ID4/5), 把峰值电流压下来。
             *    验证完(若不薓了)可以改回 4000~5000 省时间。 */
    2500,   /* HOSTAGE_LOOK  识别人质 */
    2500,   /* HOSTAGE_PRE   准备抱人质 */
    2000,   /* HOSTAGE_CLOSE 抱紧(只有夹爪动) */
    2500,   /* HOSTAGE_LIFT  抱起抬起 */
    2500,   /* HOSTAGE_GRAB_L 抓取位[左](同 PRE 的 2500) */
    2500,   /* HOSTAGE_GRAB_M 抓取位[中] */
    2500,   /* HOSTAGE_GRAB_R 抓取位[右] */
    2000,   /* HOSTAGE_GRAB_L_CLOSE 抱紧[左](同 HOSTAGE_CLOSE 的 2000;
             * ⚠️ 示教后若这一行整臂动作很大(不只是夹爪), 调大到 3000) */
    2000,   /* HOSTAGE_GRAB_M_CLOSE 抱紧[中] */
    2000,   /* HOSTAGE_GRAB_R_CLOSE 抱紧[右] */
    2500,   /* HOSTAGE_GRAB_L_LIFT  抬起[左](同 HOSTAGE_LIFT 的 2500) */
    2500,   /* HOSTAGE_GRAB_M_LIFT  抬起[中] */
    2500,   /* HOSTAGE_GRAB_R_LIFT  抬起[右] */
    2500    /* HOSTAGE_RETURN       回程姿态(收臂)。
             * ⚠️ 抱起人质后 ID2 从 1843降到 1753 / ID3 从 878升到 1205,
             *    带着负载动作, 实测抖就调到 3000。 */
};


/* 姿态名表(顺序与 ArmPose_t 一致, 日志/OLED 显示用) */
static const char *const s_arm_pose_names[ARM_POSE_COUNT] = {
    "HOME", "SCAN", "SCAN_RESET", "BALL_LOOK", "BALL_PRE", "BALL_CLOSE", "BALL_LIFT",
    "BUCKET_CARRY", "BUCKET_LOOK", "PLACE_PRE", "PLACE_OPEN", "PLACE_LIFT",
    "TARGET_READY", "TARGET_LOOK", "TARGET_FIRE", "TARGET_LIFT",
    "HOSTAGE_LOOK", "HOSTAGE_PRE", "HOSTAGE_CLOSE", "HOSTAGE_LIFT",
    "HOSTAGE_GRAB_L", "HOSTAGE_GRAB_M", "HOSTAGE_GRAB_R",
    "HOSTAGE_GRAB_L_CLOSE", "HOSTAGE_GRAB_M_CLOSE", "HOSTAGE_GRAB_R_CLOSE",
    "HOSTAGE_GRAB_L_LIFT", "HOSTAGE_GRAB_M_LIFT", "HOSTAGE_GRAB_R_LIFT",
    "HOSTAGE_RETURN"
};

/* 取姿态数组指针(HOME 指向 ServoArm 的上电初始姿态, 保证唯一来源) */
static uint16_t *ArmPose_Ptr(uint8_t pose_idx)
{
    if (pose_idx >= ARM_POSE_COUNT) {
        return NULL;
    }
    if (pose_idx == ARM_POSE_HOME) {
        return Servos_GetHomePositions();
    }
    return s_arm_pose_table[pose_idx];
}

/* ---- 姿态访问接口(示教写入/日志打印用) ---- */
const char *ArmAction_GetName(uint8_t pose_idx)
{
    if (pose_idx >= ARM_POSE_COUNT) {
        return "?";
    }
    return s_arm_pose_names[pose_idx];
}

/**
 * @brief  诊断: 读回 5 个舵机的【实际位置】并打印
 * @param  tag  日志前缀(如 “靶-抬完大臂”)
 * @note   用法: 在一个重载动作(如抬大臂)之后调一次, 把读到的实际值和姿态表的
 *         目标值对比, 用来区分“机械臂薓下去”是供电/过载还是机械问题:
 *           实际 ≈ 目标(差 < 20 码)      → 舵机有力, 保持正常;
 *           实际比目标【明显偏离/偏小】 → 被负载压下去了 → 供电跌落/扭矩不足/
 *                                          过热降额。
 */
void Arm_LogActualPositions(const char *tag)
{
    static const char *const idn[SERVO_COUNT] = { "ID1", "ID2", "ID3", "ID4", "ID5" };

    Servos_ReadPositions();   /* 先读回, 再取 */
    MLOG("机械臂回读[%s]: %s=%ld %s=%ld %s=%ld %s=%ld %s=%ld", tag,
         idn[0], (long)Servos_GetPosition(1),
         idn[1], (long)Servos_GetPosition(2),
         idn[2], (long)Servos_GetPosition(3),
         idn[3], (long)Servos_GetPosition(4),
         idn[4], (long)Servos_GetPosition(5));
}

void ArmAction_SetPositions(uint8_t pose_idx, const uint16_t pos[5])
{
    uint16_t *dst = ArmPose_Ptr(pose_idx);

    if (dst == NULL) {
        return;
    }
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        dst[i] = pos[i];
    }
    /* HOME 时 dst 就是 ServoArm 的 SERVO_POS_HOME, 无需额外同步 */
}

/**
 * @brief  摆到指定姿态(阻塞: 等舵机走完该姿态的运动时间 + 保持时间 再返回)
 * @param  pose_idx  ARM_POSE_* (见 MissionControl.h)
 * @note   本函数会阻塞主循环数秒; 但底盘闭环在 TIM9 中断里跑, 所以
 *         “先发底盘指令 → 再调本函数”可以让车和臂同时动作, 节省时间。
 */
/**
 * @brief  姿态下发的公共实现(给一份位置数组 + 姿态号, 用它自己的运动时间阻塞)
 * @param  pos      要下发的 5 个舵机位置(不一定等于姿态表里那行, 比如限幅后的副本)
 * @param  pose_idx 姿态号(只用来取运动时间 / 打日志)
 * @param  hold_ms  ⭐ 本步的"保持(稳定)时间"(ms) —— 由调用处决定用哪一档,
 *                  见 ARM_HOLD_MS_TARGET / ARM_HOLD_MS_OTHER 与 Arm_HoldMs()
 */
static void Arm_GotoPoseBufferHold(uint16_t *pos, uint8_t pose_idx, uint16_t hold_ms)
{
#if MISSION_TEST_NO_ARM
    (void)pos;
    (void)hold_ms;
    MLOG("机械臂: 跳过姿态 %s (%ums) —— MISSION_TEST_NO_ARM=1, 不驱动舵机",
         ArmAction_GetName(pose_idx), (unsigned)s_arm_pose_time[pose_idx]);
#else
    uint16_t t = s_arm_pose_time[pose_idx];

    MLOG("机械臂: 摆向 %s (运动时间 %ums, 之后还需等 %ums 稳定)",
         ArmAction_GetName(pose_idx), (unsigned)t, (unsigned)hold_ms);
    Servos_SetPositions(pos, t);
    /* ⭐ 必须用协作式等待(不能直接用 HAL_Delay): 等摆臂的这几秒里
     * 陀螺仪 yaw 要照刷、K230 收到的行要照收, 否则底盘航向/转向闭环
     * 会读到冻结角度(原地转向会“转不完”→ 车一直自转)。 */
    Mission_Coop_Wait(t + hold_ms);
#endif
}

/**
 * @brief  同上, 保持时间按【本姿态】自动取(打靶 200 / 其它 450, 见 Arm_HoldMs)
 */
static void Arm_GotoPoseBuffer(uint16_t *pos, uint8_t pose_idx)
{
    Arm_GotoPoseBufferHold(pos, pose_idx, Arm_HoldMs(pose_idx));
}

/**
 * @brief  摆到指定姿态(阻塞: 等舵机走完该姿态的运动时间 + 保持时间 再返回)
 * @param  pose_idx  ARM_POSE_* (见 MissionControl.h)
 * @note   本函数会阻塞主循环数秒; 但底盘闭环在 TIM9 中断里跑, 所以
 *         “先发底盘指令 → 再调本函数”可以让车和臂同时动作, 节省时间。
 */
void Arm_GotoPose(uint8_t pose_idx)
{
    uint16_t *pos = ArmPose_Ptr(pose_idx);

    if (pos == NULL) {
        MLOG("机械臂: 姿态编号非法 %d", (int)pose_idx);
        return;
    }
    Arm_GotoPoseBuffer(pos, pose_idx);   /* 保持时间按姿态名自动分档(见 Arm_HoldMs) */
}

/* ⭐ 2026-10-11: 保持时间【由调用处指定】的版本。
 * 用途: 打靶收尾那步虽然叫 SCAN_RESET(姿态名属"其它"档), 但它是在【打靶阶段】里
 * 执行的 ⇒ 调用处显式传 ARM_HOLD_MS_TARGET, 与"打靶 200 / 其它 450"的口径一致。
 * ⚠️ 只有那个调用点在 MISSION_TEST_NO_VISION=1 时会被编译掉, 所以加 unused 属性
 *    (与本工程其它"条件编译下可能用不到"的静态函数同一写法)。 */
static void __attribute__((unused)) Arm_GotoPoseHold(uint8_t pose_idx, uint16_t hold_ms)
{
    uint16_t *pos = ArmPose_Ptr(pose_idx);

    if (pos == NULL) {
        MLOG("机械臂: 姿态编号非法 %d", (int)pose_idx);
        return;
    }
    Arm_GotoPoseBufferHold(pos, pose_idx, hold_ms);
}

/* ⭐⭐ 救援: 对准完成时“锁存”的底座 ID1 (2026-10-07 新增, 用户要求) ----------
 *   -1 = 还没对准过 ⇒ 抓取姿态仍用姿态表里那一列的 ID1 标定值(兵底)
 *   ≥0 = 锁存值(== 对准完成那一刻的 ID1, 0~4095)
 * 【为什么要锁存】救援时底盘不动, 是靠 K230 的 C/L/R / D:x,y 一点点转底座 ID1
 *   把摄像头对准人质的 ⇒ 对准之后【ID1 当前的角度就是对的】。
 *   如果抓取/抱紧/抬起还是整表下发(连 ID1 一起写成表里的 1561/2059/2503),
 *   就会在合夹爪那一瞬把已经对准的底座又掰走几十~几百码 → 夹爪离开人质。
 *   ⇒ 下发时把 ID1 换成这个锁存值: ID1 一步都不动, 只动 ID2~ID5
 *     (它们才是“伸手/合爪/抬起”的动作)。
 * ⭐ 2026-10-09 / ⭐⭐ 2026-10-10: 锁存值不是原样下发, 还要 + 手动抓取偏移
 *   (RESCUE_GRAB_ID1_OFFSET_L/M/R, 按左/中/右 三选一) ——
 *   “摄像头对准的中心”与“夹爪真正该在的位置”实测差几十码(相机/夹爪不共轴、
 *   对准容差、机构回间隙), 用那组宏一次修掉; 见 Arm_GotoRescuePoseKeepId1。
 * ⚠️⚠️ 它只是个【运行时变量】, 【不会写回 s_arm_pose_table】:
 *    重新上电 / 再跑一次救援时, 姿态表还是原始标定值(ID1 那一列不变),
 *    只是在下次对准完成时“重新锁存”一次。
 * 设置/清除: RESCUE_PERFORM 里锁存(时机 = 刚好对准完), 并把
 *          s_rescue_id1_valid 置 1; RESCUE_COMPLETE 里两者一起清。 */
static int32_t s_rescue_id1_lock  = 0;    /* ID1 的【逻辑位置】—— ⚠️ 可为负, 见 s_id1_pos */
static uint8_t s_rescue_id1_valid = 0;    /* ⭐ 2026-10-11: 是否【真的锁存过】 */

/* ⭐⭐ 2026-10-11 修 BUG: 原来用“s_rescue_id1_lock >= 0”判断“锁存过没有”, 是错的 ——
 *   救援基准 HOSTAGE_LOOK 的 ID1 = 2059 (> 2048) ⇒ 归一化后是【负的逻辑位置 -2037】,
 *   对准完锁存下来的值必定是【负数】(实车日志: 锁存 = -2104)。
 *   ⇒ 判据失效, 抓取永远走“未锁存”分支, 用【姿态表里的 ID1(2059)】而不是
 *     【摄像头对准的那个角度(≈1992)】, 两者差 67 码 ≈ 5.9° ——
 *     这正是实测“摄像头带动 ID1 转到的位置 与 机械臂抓取的 ID1 位置不一样”的原因。
 *   ⭐ 现在改成独立的有效标志: 值可以是任意数(含负数), “有没有锁存过”只看这个标志。
 *   ⚠️ 新增“用逻辑位置(可负)”的地方时, 不要再拿数值的正负当作有效判据。 */

/* ⭐ 2026-10-10: 本次抓取的【残余补偿量】(舵机码) —— 锁存那一刻由“最后一次 D 帧的
 *   横向残余”换算而来(见 RESCUE_GRAB_USE_RESIDUAL 那段说明); 没读到过 D 帧时为 0。
 *   抓取/抱紧/抬起 三连都用它 ⇒ ID1 三处仍然是同一个值(不会中途又动一下)。 */
static int32_t s_rescue_id1_resid = 0;

/**
 * @brief  ⭐ ID1 角度【归一化】: 把任意角度码化成“相对中间位置的有符号偏移”
 * @param  pos  ID1 角度码(0~4095, 也可以是逻辑位置/负数, 内部自动取模)
 * @retval 相对 RESCUE_ID1_MID_POS(2059) 的最短偏移, 范围 (-2048, +2048]
 *            2059 →    0    (中间 = 基准)
 *            2503 → +444    (右; 正)
 *            1561 → -498    (左; 负)
 * @note   【为什么一定要归一化】伺服是单圈 0~4095 且按【最短路径】转,
 *         两个角度码直接相减会跨过 0/4095 的圈边界, 算出“绕远路”的差值。
 *         例(旧标定): 中间 252、左边 3827 —— 直接相减得 +3575, 看着像
 *         “往右转 314°”, 左右判断会完全反掉(巡视方向、抓取位三选一 都跟着错);
 *         归一化后是 -521(往左 521 码), 才对。
 *         归一化后: |偏移| = 离中间多少码, 符号 = 左/右, 而且
 *         【没有 0/4095 断点】, 加减、比较、限幅都能正常做。
 *         ⭐ 当前标定(2026-10-08)的三个位置归一化后正好是:
 *             RESCUE_ID1_RIGHT_OFF = +444  <-- 2503
 *             0                           <-- 2059(中间/基准)
 *             RESCUE_ID1_LEFT_OFF  = -498  <-- 1561
 *         ⚠️ 它与 RESCUE_ID1_MID_POS/姿态表里的其它角度做比较前也要先归一化。
 * @note   ⭐ 2026-10-10: 定义位置上移到 Rescue_GrabId1Code() 之前 ——
 *         现在“抓取 ID1 的手动偏移”要按侧别(左/中/右)分开取, 得先能判侧别。
 */
static int32_t Rescue_Id1Norm(int32_t pos)
{
    int32_t d = (pos - (int32_t)RESCUE_ID1_MID_POS) % 4096;

    if (d > 2048)  d -= 4096;
    if (d < -2048) d += 4096;
    return d;
}

/**
 * @brief  按底座 ID1 的累计偏移量, 判断人质偏哪一侧(0=左 1=中 2=右)
 * @param  id1_offset ID1 相对基准姿态 HOSTAGE_LOOK 的累计偏移(角度码, 已归一化)
 * @note   判定规则见上面 RESCUE_GRAB_MID_RANGE 处的注释。
 *         ID1 一直没转过(偏移=0, 例如超时兜底直接去抓) → 返回中间(1)。
 *         返回的是【侧别下标】而不是姿态号, 方便下面用同一张表取
 *         “抓取/抱紧/抬起”三个姿态。
 *         ⭐ 2026-10-10: 定义位置上移到 Rescue_GrabId1Code() 之前(同 Rescue_Id1Norm)。
 */
static uint8_t Rescue_GrabSideFor(int32_t id1_offset)
{
    int32_t side = id1_offset * (int32_t)RESCUE_GRAB_LEFT_SIGN;   /* >0 = 偏“左” */

    if (side > (int32_t)RESCUE_GRAB_MID_RANGE)  return 0u;   /* 左 */
    if (side < -(int32_t)RESCUE_GRAB_MID_RANGE) return 2u;   /* 右 */
    return 1u;                                              /* 中 */
}

/**
 * @brief  “本次抓取”该用的【手动 ID1 偏移】(按侧别取), 只给日志/调试用
 * @retval 左 = RESCUE_GRAB_ID1_OFFSET_L / 中 = _M / 右 = _R; 未锁存时返回 _M
 */
static int32_t Rescue_GrabId1OffsetNow(void)
{
    uint8_t side;

    if (!s_rescue_id1_valid) {
        return (int32_t)RESCUE_GRAB_ID1_OFFSET_M;
    }
    side = Rescue_GrabSideFor(Rescue_Id1Norm(s_rescue_id1_lock));
    if (side == 0u) {
        return (int32_t)RESCUE_GRAB_ID1_OFFSET_L;
    }
    if (side == 2u) {
        return (int32_t)RESCUE_GRAB_ID1_OFFSET_R;
    }
    return (int32_t)RESCUE_GRAB_ID1_OFFSET_M;
}

/**
 * @brief  救援抓取实际要下发的 ID1(带残余补偿 + 手动偏移), 返回值域 0~4095
 * @note   ① 未锁存(!s_rescue_id1_valid)时返回 -1: 调用方要退回姿态表里的 ID1;
 *         ② 否则 = 锁存值 + 残余补偿(限幅) + 手动偏移(按侧别再取) , 再取模 4096。
 *         ⚠️ 锁存值是【逻辑位置, 可为负】(救援基准 2059 → -2037), 所以这里
 *            必须先加再取模(末尾那两行), 不能拿正负当有效性判据(见
 *            s_rescue_id1_valid 的说明)。
 *         ⭐⭐ 2026-10-10(用户要求): 手动偏移改成【按侧别分开】——
 *              抓【左】边物体: +RESCUE_GRAB_ID1_OFFSET_L(+30)
 *              抓【右】边物体: +RESCUE_GRAB_ID1_OFFSET_R(-30)
 *              居中          : +RESCUE_GRAB_ID1_OFFSET_M(保持原值)
 *            侧别由【锁存值自己】算(归一化 → Rescue_GrabSideFor), 所以跟“谁先调它”
 *            无关 —— 对准成功那一刻打日志调的也是同一个结果。
 *         ⚠️ 选侧别用的仍是【锁存值本身】(本函数不改选侧别的依据)。
 */
static int32_t Rescue_GrabId1Code(void)
{
    int32_t v;

    if (!s_rescue_id1_valid) {
        return -1;
    }
    v = s_rescue_id1_lock + s_rescue_id1_resid + Rescue_GrabId1OffsetNow();
    v %= 4096;
    if (v < 0) {
        v += 4096;
    }
    return v;
}

/**
 * @brief  摆到指定姿态, 但先把每个舵机限幅到【物理行程】内再下发
 * @note   为什么需要: 救援那 9 个姿态是【用示教模式手工填表】的, 手滑把
 *         ID5 填成 700(闭合上限 600) 之类会让舵机堵转发热。这里下发前
 *         逐个限幅到 s_servo_pos_min/max, 超限就打印一行提醒。
 *         ⚠️ 只给救援抓取那一串用 —— 其余姿态仍走 Arm_GotoPose(原样下发),
 *             ﹝HOSTAGE_LOOK 的 ID4 实测就是 870, 低于表里标称的 900 下限,
 *             全局限幅会把已标定好的姿态改掉。
 */
static void Arm_GotoPoseClamped(uint8_t pose_idx)
{
    uint16_t *src = ArmPose_Ptr(pose_idx);
    uint16_t  pose[SERVO_COUNT];
    uint8_t   clipped = 0;

    if (src == NULL) {
        MLOG("机械臂: 姿态编号非法 %d", (int)pose_idx);
        return;
    }
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        int32_t v = (int32_t)src[i];

        if (v < (int32_t)s_servo_pos_min[i]) { v = (int32_t)s_servo_pos_min[i]; clipped = 1; }
        if (v > (int32_t)s_servo_pos_max[i]) { v = (int32_t)s_servo_pos_max[i]; clipped = 1; }
        pose[i] = (uint16_t)v;
    }
    if (clipped) {
        MLOG("机械臂: ⚠ %s 有舵机值超出物理行程(id2 50~2300 / id3 700~3100 / "
             "id4 900~3010 / id5 25~600), 已限幅后下发 —— 请检查姿态表",
             ArmAction_GetName(pose_idx));
    }
    Arm_GotoPoseBuffer(pose, pose_idx);
}

/* ---- ⭐⭐ 2026-10-10 新增(用户要求): 救援抓取也用同一套“Y 误差 → ID2 里程”补偿 ----
 * 救援走的是【方案二】(底盘不动, 自己解析 D 帧转底座 ID1), 不经过
 * Vision_FineAlignProcess() ⇒ 它的误差要自己抓:
 *   s_rescue_fb_px    : 最后一次 D 帧的 y(前后误差, px; 正 = 目标偏远) → 补 ID2 里程
 *   s_rescue_last_dx  : 最后一次 D 帧的 x(横向误差, px) → 抓取时补底座 ID1 残余
 *   s_rescue_d_valid  : 1 = 本轮确实读到过 D 帧(为 0 时上面两个都【不参与】补偿)
 * 生命周期: 发 start_align 时清 0, 抓完(RESCUE_COMPLETE)再清 0。
 * ⚠️ 这三个变量 + 下面的参数包/函数【必须定义在 Arm_GotoRescuePoseKeepId1 之前】:
 *    救援抓取位/抱紧位就是通过它的第二个参数用这套补偿的。 */
static int32_t s_rescue_fb_px    = 0;
static int32_t s_rescue_last_dx  = 0;
static uint8_t s_rescue_d_valid  = 0;

/* ---- ⭐⭐ 2026-10-10: “按 Y 误差补某个舵机”的【参数包】----
 * 原来球/桶两套参数是直接写死在 BombFbCompCode / Arm_GotoPoseYComp 里的;
 * 现在救援也要用同一套机制, 所以抽成参数包: 球 / 桶 / 救援 各一个常量,
 * 阈值、步长、方向、补哪个舵机【各套独立】, 改一套不影响另一套。
 * (对齐 AlignAxisCfg_t 的写法, 见上面 s_align_ball/bucket/rescue) */
typedef struct {
    uint8_t  enable;     /* 总开关: 1 = 开启, 0 = 完全关闭(回旧行为) */
    uint8_t  servo_idx;  /* 补哪个舵机下标: 1 = ID2 大臂(往前伸/往后收) */
    int32_t  sign;       /* fb_px > 0(目标偏远) 时该舵机增量的符号 */
    int32_t  far_px;     /* |fb_px| > 它 → 大步 */
    int32_t  mid_px;     /* |fb_px| > 它 → 中步 */
    int32_t  near_px;    /* |fb_px| > 它 → 小步; 否则不补 */
    int32_t  far_code;   /* 大步(码) */
    int32_t  mid_code;   /* 中步(码) */
    int32_t  near_code;  /* 小步(码) */
} FbArmCfg_t;

static const FbArmCfg_t s_fb_arm_ball   = { BOMB_FB_ARM_ENABLE, BOMB_FB_ARM_SERVO, BOMB_FB_ARM_SIGN_BALL,
                                            BOMB_FB_ARM_FAR_PX, BOMB_FB_ARM_MID_PX, BOMB_FB_ARM_NEAR_PX,
                                            BOMB_FB_ARM_FAR_CODE, BOMB_FB_ARM_MID_CODE, BOMB_FB_ARM_NEAR_CODE };
static const FbArmCfg_t s_fb_arm_bucket = { BOMB_FB_ARM_ENABLE_BUCKET, BOMB_FB_ARM_SERVO, BOMB_FB_ARM_SIGN_BUCKET,
                                            BOMB_FB_ARM_FAR_PX, BOMB_FB_ARM_MID_PX, BOMB_FB_ARM_NEAR_PX,
                                            BOMB_FB_ARM_FAR_CODE, BOMB_FB_ARM_MID_CODE, BOMB_FB_ARM_NEAR_CODE };
static const FbArmCfg_t s_fb_arm_rescue = { RESCUE_FB_ARM_ENABLE, RESCUE_FB_ARM_SERVO, RESCUE_FB_ARM_SIGN,
                                            RESCUE_FB_ARM_FAR_PX, RESCUE_FB_ARM_MID_PX, RESCUE_FB_ARM_NEAR_PX,
                                            RESCUE_FB_ARM_FAR_CODE, RESCUE_FB_ARM_MID_CODE, RESCUE_FB_ARM_NEAR_CODE };

/**
 * @brief  Y 轴像素误差 → 补偿码(分层, 粗纠正) —— 参数来自 cfg(球/桶/救援各一套)
 * @param  cfg    哪一套(见 s_fb_arm_ball / _bucket / _rescue)
 * @param  fb_px  Y(前后)像素误差(正 = 目标偏远)
 * @retval 要叠加的码数(已含方向); 0 = 误差太小, 不补
 */
static int32_t FbArmCompCode(const FbArmCfg_t *cfg, int32_t fb_px)
{
    int32_t a    = abs(fb_px);
    int32_t code;

    if (a > cfg->far_px)       code = cfg->far_code;
    else if (a > cfg->mid_px)  code = cfg->mid_code;
    else if (a > cfg->near_px) code = cfg->near_code;
    else                       return 0;

    return (fb_px > 0) ? (code * cfg->sign) : (-code * cfg->sign);
}

/**
 * @brief  把 Y 误差补偿【叠加】到 pose[cfg->servo_idx] 上(含物理行程限幅 + 日志)
 * @param  cfg      哪一套参数(NULL 或 enable=0 时什么都不做)
 * @param  fb_px    本次要用的 Y 误差(px)
 * @param  pose     正在拼的姿态数组(原地修改)
 * @param  pose_idx 姿态编号(只为打日志用)
 * @note   只动 cfg->servo_idx 那一个舵机; 补偿码为 0 时也会打印一行“未超阈值”。
 */
static void FbArmCompApply(const FbArmCfg_t *cfg, int32_t fb_px,
                           uint16_t *pose, uint8_t pose_idx)
{
    int32_t comp;
    int32_t v;

    if (cfg == NULL || !cfg->enable) {
        return;
    }
    comp = FbArmCompCode(cfg, fb_px);
    v    = (int32_t)pose[cfg->servo_idx] + comp;
    if (v < (int32_t)s_servo_pos_min[cfg->servo_idx]) {
        v = (int32_t)s_servo_pos_min[cfg->servo_idx];
    }
    if (v > (int32_t)s_servo_pos_max[cfg->servo_idx]) {
        v = (int32_t)s_servo_pos_max[cfg->servo_idx];
    }
    pose[cfg->servo_idx] = (uint16_t)v;

    MLOG("机械臂: %s 按 Y 误差 %ldpx 补偿 ID%d %+ld 码 -> %d%s",
         ArmAction_GetName(pose_idx), (long)fb_px,
         (int)(cfg->servo_idx + 1u), (long)comp, (int)v,
         (comp == 0) ? "(未超阈值, 不补)" : "");
}

/**
 * @brief  摆到指定姿态, 但【底座 ID1 用锁存的当前角度】, 其余舵机限幅后按表下发;
 *         ⭐ 2026-10-10: 还可以按 Y(前后)像素误差给 ID2 补“里程”
 * @param  pose_idx 目标姿态(救援 GRAB_x / GRAB_x_CLOSE / GRAB_x_LIFT)
 * @param  fb_cfg   补里程的参数包: 传 &s_fb_arm_rescue = 抓取位/抱紧位要补;
 *                  传 NULL = 不补(抬起位/回程姿态走这条 —— 与抓球/放桶只补
 *                  “伸出去抓”的那两下保持一致)
 * @note   用于救援的“抓取→抱紧→抬起”三连: 见 s_rescue_id1_lock 的说明。
 *         ① ID1: 若锁存过(s_rescue_id1_valid)就用它 + 手动偏移
 *            (按侧别取: 左 +30 / 中 -20 / 右 -30, 见 RESCUE_GRAB_ID1_OFFSET_L/M/R;
 *             修“摄像头对准位 vs 夹爪实际该在的位置”那几十码误差),
 *            再取模成 0~4095 下发; 三连全程 ID1 都是同一个值 ⇒ 【ID1 一步都不会动】。
 *            没锁过(测试模式等)则退回用姿态表里的 ID1, 且【不加】那个偏移。
 *         ② ID2~ID5: 照姿态表下发(这三个动作的差别全在这四列上)并限幅。
 *         ③ 整个数组是【局部】的, 不写回 s_arm_pose_table(重上电不受影响)。
 *         阻塞时长 = 该姿态自己的运动时间 + 保持时间(与 Arm_GotoPose 一致)。
 *         ⭐ 误差值 = s_rescue_fb_px(救援方案二自己解析 D 帧时记下的最后一次 y);
 *            没读到过 D 帧(s_rescue_d_valid=0)时不补。补偿是【下发前临时叠加】,
 *            不改姿态表; 叠加后仍按物理行程限幅。
 */
static void Arm_GotoRescuePoseKeepId1(uint8_t pose_idx, const FbArmCfg_t *fb_cfg)
{
    uint16_t *src = ArmPose_Ptr(pose_idx);
    uint16_t  pose[SERVO_COUNT];
    int32_t   v1;
    uint8_t   clipped = 0;

    if (src == NULL) {
        MLOG("机械臂: 姿态编号非法 %d", (int)pose_idx);
        return;
    }
    /* ⭐ 2026-10-10: 锁存值 + 残余补偿 + 手动偏移, 一次算好(见 Rescue_GrabId1Code)。
     *    残余补偿修“对准停在容差内某处”那一截, 手动偏移修固定偏差(相机/夹爪不共轴);
     *    两个都为 0 时与老行为完全一致。 */
    v1 = Rescue_GrabId1Code();
    if (v1 < 0) {
        v1 = 0;      /* 没锁过: 下面 i==0 分支会退回姿态表里的值, 这里只是占位 */
    }
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        int32_t v = (int32_t)src[i];

        if (i == 0) {
            if (s_rescue_id1_valid) {
                pose[0] = (uint16_t)v1;     /* ⭐ ID1: 锁存值 + 抓取偏移(不跟姿态表) */
            } else {
                pose[0] = (uint16_t)v;      /* 没锁过: 退回表里的值(不加偏移) */
            }
            continue;
        }
        if (v < (int32_t)s_servo_pos_min[i]) { v = (int32_t)s_servo_pos_min[i]; clipped = 1; }
        if (v > (int32_t)s_servo_pos_max[i]) { v = (int32_t)s_servo_pos_max[i]; clipped = 1; }
        pose[i] = (uint16_t)v;
    }
    if (clipped) {
        MLOG("机械臂: ⚠ %s 有舵机值超出物理行程(id2 50~2300 / id3 700~3100 / "
             "id4 900~3010 / id5 25~600), 已限幅后下发 —— 请检查姿态表",
             ArmAction_GetName(pose_idx));
    }
    if (s_rescue_id1_valid) {
        MLOG("机械臂: %s —— 底座 ID1 = 锁存 %d %+d(残余补) %+d(手动偏移) = %d(不跟姿态表), 本步只动 ID2~ID5",
             ArmAction_GetName(pose_idx), (int)(s_rescue_id1_lock % 4096),
             (int)s_rescue_id1_resid, (int)Rescue_GrabId1OffsetNow(), (int)v1);
    }
    /* ⭐⭐ 2026-10-10: 按 Y(前后)误差给 ID2 补“里程”(抓取位/抱紧位才补, 见形参说明) */
    if (fb_cfg != NULL && s_rescue_d_valid) {
        FbArmCompApply(fb_cfg, s_rescue_fb_px, pose, pose_idx);
    }
    Arm_GotoPoseBuffer(pose, pose_idx);
}

/**
 * @brief  分两步摆到指定姿态(专治“一步摆到位会剐蹭”的动作)
 * @param  pose_idx   目标姿态 (ARM_POSE_*)
 * @param  first_mask 第一步动的舵机掩码(SERVO_MASK_*), 剩下的在第二步动
 * @note   典型用法(PLACE_LIFT 实测剐蹭):
 *           Arm_GotoPoseSplit(ARM_POSE_PLACE_LIFT, SERVO_MASK_ARM_BODY);
 *         先让 ID1/ID2/ID3(底座/大臂/副关节)转到位并停稳,
 *         再动 ID4/ID5(腕部/夹爪) —— 避免大臂还在转的时候腕/夹爪扫到车架。
 *         第 1 步用该姿态自己的运动时间, 第 2 步用其一半(不短于 ARM_SPLIT_2ND_MIN_MS)。
 *         等待同样用 Mission_Coop_Wait: 摆臂期间照刷陀螺仪 yaw、照收 K230 行。
 */
void Arm_GotoPoseSplit(uint8_t pose_idx, uint8_t first_mask)
{
    uint16_t *pos = ArmPose_Ptr(pose_idx);

    if (pos == NULL) {
        MLOG("机械臂: 姿态编号非法 %d", (int)pose_idx);
        return;
    }
#if MISSION_TEST_NO_ARM
    (void)first_mask;
    MLOG("机械臂: 跳过分步姿态 %s —— MISSION_TEST_NO_ARM=1",
         ArmAction_GetName(pose_idx));
#else
    {
        uint16_t t  = s_arm_pose_time[pose_idx];
        uint16_t t2 = (uint16_t)(t / 2);
        uint16_t hold_ms = Arm_HoldMs(pose_idx);   /* ⭐ 2026-10-11: 分档(打靶 200 / 其它 450) */
        uint8_t  second_mask;

        if (t2 < ARM_SPLIT_2ND_MIN_MS) t2 = ARM_SPLIT_2ND_MIN_MS;
        if (t2 > t)                    t2 = t;
        second_mask = (uint8_t)((~first_mask) & SERVO_MASK_ALL);

        MLOG("机械臂: 分两步摆向 %s (第1步 掩码0x%02X %ums; 第2步 掩码0x%02X %ums; 每步后等 %ums)",
             ArmAction_GetName(pose_idx), (unsigned)first_mask, (unsigned)t,
             (unsigned)second_mask, (unsigned)t2, (unsigned)hold_ms);

        /* 第 1 步: 底座/大臂/副关节先转到位 */
        Servos_SetPositionsMasked(pos, first_mask, t);
        Mission_Coop_Wait(t + hold_ms);

        /* 第 2 步: 前面已停稳, 再动腕部/夹爪 */
        Servos_SetPositionsMasked(pos, second_mask, t2);
        Mission_Coop_Wait(t2 + hold_ms);
    }
#endif
}

#if !MISSION_TEST_NO_ARM
/* 示教标定状态 */
static volatile uint8_t s_arm_teach_active = 0;
static uint8_t s_arm_teach_action = ARM_POSE_HOME;

/* ---- 示教标定模式(联调期) ----
 * KEY2 长按 = 进入/退出; 示教中 KEY2 = 切换动作, KEY1 = 读取当前位置写入当前动作 */
bool ArmTeach_IsActive(void)
{
    return s_arm_teach_active != 0;
}

void ArmTeach_Enter(void)
{
    s_arm_teach_active = 1;
    s_arm_teach_action = ARM_POSE_HOME;
    Chassis_SyncTarget(); /* 示教: 丢弃累积残差, 免得小车自己往前蹭 */
    Chassis_Stop();       /* 停住小车 */
    Laser_Off();
    Servos_UnloadAll();   /* 卸力, 便于手动摆臂 */
    MLOG("示教: 已进入(可手动掰机械臂) KEY2=切到下一个姿态, KEY1=记录当前姿态");
}

void ArmTeach_Exit(void)
{
    s_arm_teach_active = 0;
    for (uint8_t i = SERVO_ARM_ID_MIN; i <= SERVO_ARM_ID_MAX; i++) {
        Servos_SetTorque(i, 1);
    }
    Servos_SetPositions(Servos_GetHomePositions(), 1500);
    MLOG("示教: 已退出(舵机上力, 回初始姿态)");
}

void ArmTeach_NextAction(void)
{
    s_arm_teach_action = (uint8_t)((s_arm_teach_action + 1) % ARM_POSE_COUNT);
    MLOG("示教: 当前要记录的姿态[%d] = %s", (int)s_arm_teach_action,
         ArmAction_GetName(s_arm_teach_action));
}

void ArmTeach_WriteCurrent(void)
{
    uint16_t pos[SERVO_COUNT];

    Servos_ReadPositions();
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        int32_t p = Servos_GetPosition(i + 1);
        if (p < 0 || p > 4095) {
            MLOG("示教: 警告 舵机%d 位置读取失败, 该位按 0 处理", (int)(i + 1));
            p = 0;
        }
        pos[i] = (uint16_t)p;
    }
    // ArmAction_SetPositions(s_arm_teach_action, pos);
    MLOG("示教: 读出当前姿态 %s = %d,%d,%d,%d,%d",
         ArmAction_GetName(s_arm_teach_action),
         (int)pos[0], (int)pos[1], (int)pos[2], (int)pos[3], (int)pos[4]);
}

uint8_t ArmTeach_GetActionIdx(void)
{
    return s_arm_teach_action;
}

const char *ArmTeach_GetActionName(void)
{
    return ArmAction_GetName(s_arm_teach_action);
}
#endif /* !MISSION_TEST_NO_ARM */

/* =====================================================================
 * K230 行队列 + 协作式等待(摆臂期间保命用, 详见 Mission_Coop_Wait 注释)
 *   K230 的接收是“单缓冲”: UART4 中断直接覆盖 g_k230_rx_line。摆臂一次阻塞
 *   好几秒, 其间连来两帧就会丢前一帧(可能是扫码结果或 C/L/R) →
 *   这里把收到的行先搬进小队列, 由 Mission_GetNewLine() 按顺序取走。
 * ===================================================================== */
#define K230_QUEUE_DEPTH   8    /* 队列深度(行数); 8 行 = 512 字节 RAM */

static char    s_k230_q[K230_QUEUE_DEPTH][K230_LINE_MAX];
static uint8_t s_k230_q_head = 0;    /* 取(最早入队) */
static uint8_t s_k230_q_tail = 0;    /* 存(下一个空位) */
static uint8_t s_k230_q_count = 0;

/* 由 main.c 注入的“刷新陀螺仪 yaw”回调(见 Mission_SetYawPollHook) */
static void (*s_yaw_poll_hook)(void) = NULL;

/**
 * @brief  把中断里新收到的一行搬进队列(队列满时丢最旧的一行)
 */
static void K230_QueuePush(void)
{
    if (!g_k230_new_data_flag) {
        return;
    }
    g_k230_new_data_flag = 0;
    memcpy(s_k230_q[s_k230_q_tail], (const void *)g_k230_rx_line, K230_LINE_MAX - 1);
    s_k230_q[s_k230_q_tail][K230_LINE_MAX - 1] = '\0';
    s_k230_q_tail = (uint8_t)((s_k230_q_tail + 1) % K230_QUEUE_DEPTH);
    if (s_k230_q_count < K230_QUEUE_DEPTH) {
        s_k230_q_count++;
    } else {
        s_k230_q_head = (uint8_t)((s_k230_q_head + 1) % K230_QUEUE_DEPTH);  /* 满了: 丢最旧 */
    }
}

/**
 * @brief  清空 K230 接收缓冲 + 阻塞期间攒下的队列
 * @note   相当于原来的 Serial_FlushRx(), 只是把队列也一起清掉,
 *         否则“发新指令前先清旧数据”会清不干净。
 */
static void K230_FlushAll(void)
{
    Serial_FlushRx();
    s_k230_q_head = 0;
    s_k230_q_tail = 0;
    s_k230_q_count = 0;
}

/**
 * @brief  协作式等待(机械臂动作专用, 替代 HAL_Delay)
 * @param  ms 要等的时间(ms)
 * @note   与 HAL_Delay 的区别:
 *         ① 每 20ms 调一次 s_yaw_poll_hook 刷新陀螺仪 yaw ——
 *            否则 TIM9 里的航向/转向闭环读到的是冻结角度, 原地转向会
 *            “转不完”而让车一直自转;
 *         ② 把 K230 中断里收到的新行搬进队列 ——
 *            否则阻塞期间再来一帧就会把上一帧覆盖掉(丢扫码/对准结果)。
 *         底盘闭环在 TIM9 中断里照跑, 所以车和臂是真并行。
 */
void Mission_Coop_Wait(uint32_t ms)
{
    uint32_t t0 = HAL_GetTick();
    uint32_t next_yaw = t0 + 20;

    for (;;) {
        uint32_t now = HAL_GetTick();

        if (now - t0 >= ms) {
            break;
        }
        if ((int32_t)(now - next_yaw) >= 0) {
            next_yaw += 20;
            if (s_yaw_poll_hook != NULL) {
                s_yaw_poll_hook();
            }
        }
        K230_QueuePush();
    }
    K230_QueuePush();   /* 离开前再收一次, 别把刚到的丢掉 */
}

/**
 * @brief  注入“刷新陀螺仪 yaw”的回调(由 main.c 在初始化时调用)
 */
void Mission_SetYawPollHook(void (*fn)(void))
{
    s_yaw_poll_hook = fn;
}

/* =====================================================================
 * ⭐ 换任务“静默期” (2026-10-04 新增, 修“打靶被旧帧误判”的 bug)
 * ---------------------------------------------------------------------
 * 现象: 排爆放完球、右移 1510mm 后进打靶, 日志里刚发完 run_task:2 就
 *       立刻收到 "Task2 Dir: C", 于是 ID1 一步没转就去摆 TARGET_FIRE 了。
 * 原因: K230 是【连续输出】的(比如每 33ms 一帧)。发 run_task:2 那一刻,
 *       K230 还在按上一个任务(桶/球)的模式往外发 C/L/R/OK —— 这些帧要么
 *       在路上、要么刚进 UART, K230_FlushAll() 只能清 MCU 软件层, 清不掉。
 *       ⇒ 下一帧一读到, 就被当成“靶子已经在画面中心”。
 * 对策: 发新 run_task 时记下时刻 + 置静默标志, 之后 VISION_MODE_SETTLE_MS
 *       内的所有行全部丢弃, 等 K230 真正切完模式再从新帧开始判断。
 *       ⚠️ 只影响“换任务”的那一小段; 精对准(start_align)不触发静默期。
 * ===================================================================== */
static uint32_t s_vision_mode_tick = 0;   /* 换任务时刻(静默期起点) */
static uint8_t  s_vision_settling  = 0;   /* 1 = 正在静默期 */
static uint8_t  s_vision_drop_cnt  = 0;   /* 本次静默期丢掉了多少行(仅供日志) */

/* ⭐ K230 数据流监控(给超时日志用, 见 K230_RxSilenceMs) */
static uint32_t s_k230_last_rx_tick = 0;   /* 最近一次真正读到 K230 一行的时刻 */
static uint32_t s_k230_rx_lines     = 0;   /* 累计读到多少行 */

/** @brief 开始换任务静默期(由 Vision_SendTask 在首次发 run_task 时调用) */
static void Vision_ModeSwitchStart(void)
{
    s_vision_mode_tick = HAL_GetTick();
    s_vision_settling  = 1;
    s_vision_drop_cnt  = 0;
}

/**
 * @brief  读取一条新的 K230 消息到本地缓冲
 * @retval 1=有新消息, 0=无
 * @note   优先取“机械臂阻塞期间攒下”的队列, 再取中断里的最新一行;
 *         换任务静默期内的所有行会被丢掉(见上面 VISION_MODE_SETTLE_MS)
 */
static uint8_t Mission_GetNewLine(char *dst, uint16_t maxlen)
{
    /* ⭐ 换任务静默期: 把上一个任务换模式前发出的残留帧全部丢掉 */
    if (s_vision_settling) {
        if ((HAL_GetTick() - s_vision_mode_tick) < VISION_MODE_SETTLE_MS) {
            s_vision_drop_cnt = (uint8_t)(s_vision_drop_cnt + s_k230_q_count);
            s_k230_q_head  = 0;
            s_k230_q_tail  = 0;
            s_k230_q_count = 0;
            if (g_k230_new_data_flag) {
                g_k230_new_data_flag = 0;
                s_vision_drop_cnt++;
            }
            return 0;
        }
        s_vision_settling = 0;
        MLOG("视觉: 换任务静默期结束, 丢弃了 %u 条上个任务的旧数据",
             (unsigned)s_vision_drop_cnt);
    }

    if (s_k230_q_count > 0) {
        memcpy(dst, s_k230_q[s_k230_q_head], maxlen - 1);
        dst[maxlen - 1] = '\0';
        s_k230_q_head = (uint8_t)((s_k230_q_head + 1) % K230_QUEUE_DEPTH);
        s_k230_q_count--;
        s_k230_last_rx_tick = HAL_GetTick();
        s_k230_rx_lines++;
        return 1;
    }

    if (!g_k230_new_data_flag) {
        return 0;
    }
    g_k230_new_data_flag = 0;
    memcpy(dst, (const void *)g_k230_rx_line, maxlen - 1);
    dst[maxlen - 1] = '\0';
    s_k230_last_rx_tick = HAL_GetTick();
    s_k230_rx_lines++;
    return 1;
}

/**
 * @brief  距上次真正读到 K230 一行数据过去了多少 ms
 * @note   ⭐ 专门给各种“超时/放弃”日志用: 一眼就能分辨
 *           ① K230 一直没发数据(超声波很大)  → 查 K230 端(卡住/看不到目标/异常退出)
 *           ② K230 一直在发但内容不对(消声很小) → 查方向映射/判定阀值
 *         从未收到过时返回 0xFFFFFF(约 16.7s)当作“很久”。
 */
static uint32_t K230_RxSilenceMs(void)
{
    if (s_k230_rx_lines == 0) {
        return 0xFFFFFFu;
    }
    return HAL_GetTick() - s_k230_last_rx_tick;
}

/**
 * @brief  ⭐⭐ 2026-10-11 新增: 航向校正前的“小角度跳过”判断
 * @param  target_deg 目标绝对航向(度)
 * @retval 1 = 偏差 ≤ TURN_SKIP_DEG, 【不要】再发转向指令; 0 = 需要真正转向
 * @note   为什么要有它(1~2° 的校正"转不动还容易失控"), 见 TURN_SKIP_DEG 的说明。
 *         用 Chassis_GetYaw()(已过跳变检查), 所以坏读数不会让它误判成"已正"。
 */
static uint8_t Turn_SkipIfTiny(float target_deg)
{
    float e = target_deg - Chassis_GetYaw();

    while (e > 180.0f)  e -= 360.0f;
    while (e < -180.0f) e += 360.0f;

    if (TURN_SKIP_DEG > 0.0f && e <= TURN_SKIP_DEG && e >= -TURN_SKIP_DEG) {
        MLOG("航向校正: 偏差 %+.2f° 已在 ±%.2f° 内 → 跳过本次转向(见 TURN_SKIP_DEG)",
             (double)e, (double)TURN_SKIP_DEG);
        return 1u;
    }
    return 0u;
}

/**
 * @brief  ⭐ 2026-10-11 新增: 按"停车方式"宏设定【紧接着的一次平移】的提前减速
 * @param  mode 0 = 平滑减速(不覆盖, 长段按 CH_SLOWDOWN_LONG_* 自动拉长);
 *              1 = 直接急停(本段完全不减速, 全速到位即断输出);
 *              2 = 只用固定 CH_SLOWDOWN_DIST_COUNT(≈30mm)的小减速
 * @note   见 ROUTE_8B_RIGHT_STOP_MODE 处的说明。
 *         ⚠️ 必须【紧接着】发移动指令 —— 这个覆盖只生效一次, 发一次移动就消耗掉。
 */
static void Route_ApplyStopMode(int mode)
{
    if (mode == 1) {
        MLOG("停车方式: 本段【直接急停】(不做提前减速, 全速到位即停)");
        Chassis_SetNextMoveSlowdown(0, 0.0f);
    } else if (mode == 2) {
        MLOG("停车方式: 本段只用固定 %d 计数(≈%.0fmm)的小减速(不按比例拉长)",
             (int)CH_SLOWDOWN_DIST_COUNT,
             (double)CH_SLOWDOWN_DIST_COUNT / CH_COUNTS_PER_MM);
        Chassis_SetNextMoveSlowdown((int32_t)CH_SLOWDOWN_DIST_COUNT, CH_SLOWDOWN_MIN_COUNT);
    }
    /* mode 0: 不设置 ⇒ 走默认(短段 30mm / 长段按比例拉长) */
}

/**
 * @brief  ⭐⭐ 2026-10-11 新增(用户实测: "打靶结束之后转弯 90° 时航向校准不准"):
 *         大转角(90°)专用的【阻塞式转正 + 停稳 + 压正】
 * @param  target_deg 目标绝对航向(度), 例: RESCUE_HEADING_DEG = -90°
 * @retval 1 = 已经足够正(两次都没动); 0 = 做过至少一次转向
 * @note   为什么要这套(而不是像别处那样"发个非阻塞转向 + 转移条件等完成"):
 *         ① 转向环的完成判据允许角速度到 CH_TURN_RATE_THRESHOLD(0.2°/周期 = 10°/s),
 *            判"完成"后其实是 Chassis_Stop() 断输出【滑行】—— 90° 这种大转角带上来的
 *            角速度最大, 滑过的量也最大(实测 1~3°);
 *         ② 各处的"航向校正"走 Heading_AlignTo → TURN_SKIP_DEG = 2° 以内【直接跳过】,
 *            所以那 1~3° 的残留经常【没人再管】, 直接被带进整段救援走位。
 *         做法: 转到位 → 原地停稳 RESCUE_TURN_SETTLE_MS(让滑行/晃动结束) → 再压正一次,
 *            而且压正这一步用【更紧的跳过门槛】RESCUE_TURN_TIGHT_DEG(1°), 不被 2° 放过。
 *         ⚠️ 用 Mission_Coop_Wait 等(不是 HAL_Delay): 期间照刷陀螺仪 yaw、照收 K230 行。
 *         ⚠️ 每次转向的等待上限 = RESCUE_TURN_TIMEOUT_MS, 超时会强制停车(不会卡死)。
 */
#define RESCUE_TURN_SETTLE_MS        300    /* 两次转正之间的原地停稳(ms); 0 = 不等待 */
#define RESCUE_TURN_TIMEOUT_MS       6000   /* 每次转正的等待上限(ms) */
#define RESCUE_TURN_TIGHT_DEG        1.0f   /* 压正这一步的"够正"门槛(度), 不跟 TURN_SKIP_DEG(2°) */

/* 前置声明: 定义在下面"转向辅助"那一段(Chassis_WaitTurnDone 附近) */
static uint8_t Chassis_WaitTurnDone(uint32_t timeout_ms);

static uint8_t Turn_AlignTightBlocking(float target_deg)
{
    float e = target_deg - Chassis_GetYaw();
    uint8_t moved = 0;

    while (e > 180.0f)  e -= 360.0f;
    while (e < -180.0f) e += 360.0f;

    /* 基准一定要同步(后面平移的航向保持用它), 不管转不转 */
    Chassis_SetHeadingRef(target_deg);

    if (e <= RESCUE_TURN_TIGHT_DEG && e >= -RESCUE_TURN_TIGHT_DEG) {
        MLOG("大转角压正: 偏差 %+.2f° 已在 ±%.2f° 内 → 不用转(只同步基准)",
             (double)e, (double)RESCUE_TURN_TIGHT_DEG);
        return 1u;
    }

    /* 第 1 次转正 */
    Chassis_Rotate_To(target_deg);
    Chassis_WaitTurnDone(RESCUE_TURN_TIMEOUT_MS);
    MLOG("大转角转正(1/2)完成: yaw=%.2f° (目标 %.2f°)", (double)Chassis_GetYaw(), (double)target_deg);
    moved = 1u;

    /* 停稳: 让滑行/车身晃动结束, 再压正一次消掉残余 */
    Mission_Coop_Wait(RESCUE_TURN_SETTLE_MS);
    e = target_deg - Chassis_GetYaw();
    while (e > 180.0f)  e -= 360.0f;
    while (e < -180.0f) e += 360.0f;
    if (e > RESCUE_TURN_TIGHT_DEG || e < -RESCUE_TURN_TIGHT_DEG) {
        Chassis_Rotate_To(target_deg);
        Chassis_WaitTurnDone(RESCUE_TURN_TIMEOUT_MS);
        MLOG("大转角压正(2/2)完成: yaw=%.2f° (目标 %.2f°, 停稳后才压的正)",
             (double)Chassis_GetYaw(), (double)target_deg);
    } else {
        MLOG("大转角压正(2/2): 停稳后偏差 %+.2f° 已够正, 不再转", (double)e);
    }
    return moved;
}

/**
 * @brief  ⭐ 2026-10-11 新增: "挪一小步"的【冲击保护】—— 短距离贴边移动要慢、要早减速
 * @param  mm       本步的步长(mm): 0 = 直接返回(不挪这一步, 也就别设覆盖)
 * @param  cap_mmps 本段最高速度(mm/s); 0 = 不限
 * @param  ramp_mm  本段减速起始距离(mm); 0 = 用默认(短段 30mm)
 * @note   用户要求(转完 90° 后车头前进 75mm 那一段): "防止它冲太快出界"。
 *         位置环的"提前减速"只削末段速度, 削不掉"起步就冲" ⇒ 所以这里同时
 *         压【速度上限】(Chassis_SetNextMoveSpeedCap) + 拉长【减速区】。
 *         ⚠️ 两个覆盖都【只生效一次】, 而且是"下一次移动"消耗 —— 所以:
 *            ① 必须在紧接着的移动指令【之前】调用;
 *            ② mm == 0(不挪步)时必须把挂着的覆盖清掉, 否则会漏给后面真正的移动。
 */
static void Route_ImpactGuard(int32_t mm, float cap_mmps, int32_t ramp_mm)
{
    if (mm == 0) {
        /* 不挪步: 清掉可能挂着的覆盖, 免得漏给下一个移动指令 */
        Chassis_SetNextMoveSlowdown(-1, 0.0f);
        Chassis_SetNextMoveSpeedCap(0.0f);
        return;
    }
    if (ramp_mm > 0) {
        Chassis_SetNextMoveSlowdown((int32_t)((float)ramp_mm * CH_COUNTS_PER_MM),
                                   CH_SLOWDOWN_MIN_COUNT);
    }
    if (cap_mmps > 0.0f) {
        Chassis_SetNextMoveSpeedCap(cap_mmps);
    }
    MLOG("冲击保护: 本步 %dmm —— 限速 %dmm/s, 减速起始 %dmm",
         (int)mm, (int)cap_mmps, (int)ramp_mm);
}

/* ---- 前置声明(定义在文件下方"机械臂 ID1"那一段, 那里才有 s_id1_offset/s_id1_pos) ---- */
static int32_t Arm_Id1GetOffset(void);
static int32_t Arm_Id1GetLogicalPos(void);
static void    Arm_Id1Step(uint8_t pose_idx, int32_t delta, uint16_t move_ms,
                           int32_t pos_min, int32_t pos_max, uint16_t speed_min, uint8_t acc);

/* ---- 桶横向"轻微误差用 ID1 修"的状态量(见 BUCKET_LR_ID1_ENABLE) ---- */
static int8_t   s_bucket_id1_sign     = (int8_t)BUCKET_LR_ID1_SIGN;  /* 当前生效的方向符号 */
static int32_t  s_bucket_id1_last_px  = 0;    /* 上一次用 ID1 修时的误差(判方向反了) */
static uint8_t  s_bucket_id1_moved    = 0;    /* 上一次是否真的动过 ID1 */
static uint8_t  s_bucket_id1_flipped  = 0;    /* 是否已经自动翻过方向(只翻一次) */
/* ⭐⭐ 2026-10-11(用户要求): 【锁存】—— 与救援 s_rescue_id1_lock 同一套做法:
 *   在"摄像头对准桶"那一刻把 ID1 的数值存下来, 放球动作直接用它(见 Arm_BucketPlaceId1)。
 *   ⚠️ 原来是"放球姿态 = 标定值 + 累计偏移"(s_bucket_id1_off_valid 那套), 已改成
 *      与救援完全一致的【锁存】, 旧变量已删除(不要再引用它)。 */
static int32_t  s_bucket_id1_lock       = 0;   /* 锁存值(ID1 逻辑位置) */
static int32_t  s_bucket_id1_lock_base  = 0;   /* 锁存那一刻的基准值(BUCKET_LOOK 表值),
                                                * 用来算"对准修正量 = 锁存 − 基准" */
static uint8_t  s_bucket_id1_lock_valid = 0;   /* 1 = 锁存过(放球带着它); 放完球清 0 */

/**
 * @brief  ⭐⭐ 2026-10-11 新增(用户要求): 桶【轻微横向误差】改用底座 ID1 修
 * @param  lr_px      本帧的横向像素误差(+ = 目标偏画面左)
 * @param  tag        日志用标签("BUCKET")
 * @param  p_cooldown 冷却时间输出(与底盘步一致)
 * @retval 1 = 已经用 ID1 修了一步(底盘不动); 0 = ID1 到边界/关掉了 ⇒ 调用方回退走底盘
 * @note   为什么要这套: 见 BUCKET_LR_ID1_ENABLE 处的说明(死区导致"个别轮子转")。
 *         机制: 修正量累计在【ID1 相对基准的偏移】里(s_id1_offset, Arm_Id1Step 自动维护),
 *               对准时 = BUCKET_LOOK(40) + 偏移, 放球时 = PLACE_PRE(10) + 同一个偏移
 *               (见 Arm_BucketPlaceId1) —— 这样"对准摆了多少、放球就跟着摆多少", 两者
 *               之间的标定差(30 码)不会被破坏。
 *         ⚠️ ID1 的【绝对】位置被限幅在 [BUCKET_LR_ID1_POS_MIN, BUCKET_LR_ID1_POS_MAX]
 *            (0~40 码, 用户定的安全范围), 到边界就停并返回 0 ⇒ 调用方回退走底盘,
 *            不会卡死; 也正因为是绝对限幅, 永远不会出现"ID1 转到 4095"那种越界。
 *         ⚠️ 转 ID1 期间【底盘必须停着】: 本函数不动底盘, 调用方也【不要】再发移动指令,
 *            否则摄像头/臂在动、车也在动, 误差会互相打架。
 */
static uint8_t Bucket_LrFineWithId1(int32_t lr_px, const char *tag, uint32_t *p_cooldown)
{
    float   f;
    int32_t d;
    int32_t base  = (int32_t)s_arm_pose_table[ARM_POSE_BUCKET_LOOK][0];  /* = 40 */
    int32_t cur   = base + Arm_Id1GetOffset();     /* 对准时的 ID1 绝对位置 */
    int32_t v_new = cur;
    int32_t real;

    /* 方向: 画面偏左 → 按 BUCKET_LR_ID1_SIGN 决定 ID1 往哪转
     * (符号反了不要紧: 下面 AUTOFLIP 会在"越修越大"时自动取反一次) */
    f = (float)lr_px / BUCKET_LR_ID1_PX_PER_CODE * (float)s_bucket_id1_sign;
    d = (int32_t)((f >= 0.0f) ? (f + 0.5f) : (f - 0.5f));
    if (d == 0) {
        d = (lr_px > 0) ? 1 : -1;      /* 误差极小也别"一步不动", 至少给 1 码 */
    }
    if (d >  (int32_t)BUCKET_LR_ID1_MAX_STEP) d =  (int32_t)BUCKET_LR_ID1_MAX_STEP;
    if (d < -(int32_t)BUCKET_LR_ID1_MAX_STEP) d = -(int32_t)BUCKET_LR_ID1_MAX_STEP;

    /* 预测限幅后的实际位移: 到边界(0 或 40)就没法再动了 ⇒ 交给调用方走底盘 */
    v_new += d;
    if (v_new < (int32_t)BUCKET_LR_ID1_POS_MIN) v_new = (int32_t)BUCKET_LR_ID1_POS_MIN;
    if (v_new > (int32_t)BUCKET_LR_ID1_POS_MAX) v_new = (int32_t)BUCKET_LR_ID1_POS_MAX;
    real = v_new - cur;
    if (real == 0) {
        MLOG("视觉[%s][横向/ID1]: ID1 已到范围边界(%ld, 允许 %d~%d) 转不动了 -> 回退用底盘平移",
             tag, (long)cur, (int)BUCKET_LR_ID1_POS_MIN, (int)BUCKET_LR_ID1_POS_MAX);
        return 0u;
    }

#if BUCKET_LR_ID1_AUTOFLIP
    /* 方向自检: 上一步转过 ID1 之后 |误差| 反而变大 ⇒ 方向反了, 取反一次 */
    if (!s_bucket_id1_flipped && s_bucket_id1_moved &&
        (abs(lr_px) > (abs(s_bucket_id1_last_px) + (int32_t)BUCKET_LR_ID1_FLIP_MIN_PX))) {
        s_bucket_id1_sign    = (int8_t)(-s_bucket_id1_sign);
        s_bucket_id1_flipped = 1;
        s_bucket_id1_moved   = 0;
        MLOG("视觉[%s][横向/ID1]: ⚠ 上一步转完 |误差| 从 %ldpx 变大到 %ldpx ⇒ 判为方向反了, "
             "BUCKET_LR_ID1_SIGN 自动取反为 %+d(本帧先不动作, 下一帧按新方向重来)",
             tag, (long)labs((long)s_bucket_id1_last_px), (long)abs(lr_px), (int)s_bucket_id1_sign);
        return 1u;   /* 吃掉这一帧: 不动作, 下一帧按新方向算 */
    }
#endif

    MLOG("视觉[%s][横向/ID1]: 误差 %ldpx 属于【轻微】(不够走底盘的最小步 %dmm) -> "
         "底盘不动, ID1 转 %+ld 码 (%ld -> %ld, 累计偏移 %+ld, 范围 %d~%d)",
         tag, (long)lr_px, (int)BUCKET_LR_CHASSIS_MIN_MM,
         (long)real, (long)cur, (long)(cur + real), (long)Arm_Id1GetOffset(),
         (int)BUCKET_LR_ID1_POS_MIN, (int)BUCKET_LR_ID1_POS_MAX);
    Arm_Id1Step(ARM_POSE_BUCKET_LOOK, real, BUCKET_LR_ID1_MOVE_MS,
                (int32_t)BUCKET_LR_ID1_POS_MIN, (int32_t)BUCKET_LR_ID1_POS_MAX,
                TARGET_FINE_SPEED_MIN, TARGET_FINE_ACC);
    s_bucket_id1_last_px = lr_px;
    s_bucket_id1_moved   = 1;
    *p_cooldown = HAL_GetTick() + FINE_TUNE_COOLDOWN_MS;
    return 1u;
}

/**
 * @brief  兼容参考工程的转向接口
 * @note   本工程 Chassis_Rotate 为【相对旋转】; 参考工程 Turn_Angle 为绝对角度。
 *         角度符号/绝对值需按实际场地标定, 微小角度(|a|<1)按"航向校正占位"处理。
 *         ⭐ 2026-10-11: 校 0° 之前先过 Turn_SkipIfTiny() —— 偏差 ≤ TURN_SKIP_DEG
 *            就不发转向(那个量级的校正在这台车上转不动, 见 TURN_SKIP_DEG)。
 */
static void Turn_Angle_Compat(float angle)
{
    if (angle > -1.0f && angle < 1.0f) {
        /* 分段后的航向校正: 旋转到绝对 0°(车头方向), 用 JY61P yaw 闭环 */
        if (Turn_SkipIfTiny(0.0f)) {
            return;
        }
        Chassis_Rotate_To(0.0f);
    } else {
        Chassis_Rotate(angle);
    }
}

/**
 * @brief  ⭐⭐ 2026-10-11 新增: 阻塞等“转向环收工”, 超时【强制停车】
 * @param  timeout_ms 等待上限(ms)
 * @retval 1 = 转向环正常判到位; 0 = 超时(已 Chassis_Stop())
 * @note   为什么必须在这里 Stop(这是修实车"弯形移动"的关键):
 *           转向环的完成判据是 Chassis_Task_Is_Complete() = (!s_turn_open && !s_moving),
 *           超时说明它【没转完】—— 此时若不关, s_turn_open 仍是 1、剩余角还挂着,
 *           后面任何平移都不取消它 ⇒ 车会"边走边转", 实测右移走成一条弧线。
 *           实车日志铁证(排爆后那次校正超时之后紧接着右移):
 *             MISSION 排爆后: 第2次(压正)完成 ... -> 开始右移
 *             STEER   on=1 err=126.00 P=189.00 out=192.00 integ=500.0   ← 环还开着
 *           ⚠️ 用 Mission_Coop_Wait 等(不是 HAL_Delay): 期间照刷陀螺仪 yaw、照收 K230 行。
 *           ⚠️ 超时后只是"停住并往下走"(不重试), 日志会打出剩余角/航向,
 *              出现它 ⇒ 检查陀螺仪读数与 CH_TURN_ABORT_EXTRA_DEG 那套保护。
 */
static uint8_t Chassis_WaitTurnDone(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();

    while (!Chassis_Task_Is_Complete() && (HAL_GetTick() - t0) < timeout_ms) {
        Mission_Coop_Wait(20);
    }
    if (!Chassis_Task_Is_Complete()) {
        MLOG("⚠ 转向等待超时 %lums: 剩余角 %.1f°, yaw=%.1f°(目标 0/基准) "
             "→ 强制停车(不关的话后面平移会“边走边转”走成弧线)",
             (unsigned long)(HAL_GetTick() - t0),
             (double)Chassis_GetTurnRemaining(), (double)Chassis_GetYaw());
        Chassis_Stop();
        return 0u;
    }
    return 1u;
}

/**
 * @brief  航向校准(绝对角度版): 把车头转到指定绝对航向, 并同步航向基准
 * @param  heading_deg 目标绝对航向(度): 正 = 逆时针/左转, 负 = 顺时针/右转
 * @note   与 Turn_Angle_Compat(0.1f) 的区别: 那个固定转到绝对 0°
 *         (前面所有阶段车头朝向都是 0°); 而救援阶段车头常驻
 *         RESCUE_HEADING_DEG(-90°), 所以这里显式给出目标角。
 *         ⚠️ 必须同时改【航向基准】(Chassis_SetHeadingRef):
 *            平移时“航向保持”用的就是这个基准, 不改的话下一段
 *            Chassis_Move_Right 会按旧基准(0°)把车硬拽回原朝向,
 *            相当于 90° 掉头白做、而且横移会被拧成弧线。
 *         用法: 进入动作里调一次(非阻塞), 转移条件里等
 *               Chassis_Task_Is_Complete() 即可。
 *         ⭐ 2026-10-11: 偏差 ≤ TURN_SKIP_DEG 时只同步基准、不发转向(见 TURN_SKIP_DEG)。
 */
static void Heading_AlignTo(float heading_deg)
{
    Chassis_SetHeadingRef(heading_deg);     /* 基准一定要同步(平移时航向保持用它) */
    if (Turn_SkipIfTiny(heading_deg)) {
        return;
    }
    Chassis_Rotate_To(heading_deg);
}

/**
 * @brief  打靶走位途中: 航向校正【之前】的“挪最小一步”(阻塞版, 可正可负)
 * @param  mm 步长(mm): >0 = 车头前进 | <0 = 车头后退 | 0 = 直接返回(不挪这一步)
 * @note   ⭐ 2026-10-07 新增, 动机见 ROUTE_12_A_MINSTEP_MM 处的说明。
 *         ⭐ 2026-10-08: 改成可正可负(与 Route_MinStep 同一套符号约定)。
 *         ⭐ 2026-10-09: 改成【由调用方传步长】—— 4 处各有自己的固定步长宏,
 *            第 2 / 第 4 处还能选“按偏航角”; 调用点分别传
 *            ROUTE_12_A_MINSTEP_MM / Route_A2_MinStepMM() /
 *            ROUTE_12_B_MINSTEP_MM / Route_B2_MinStepMM()。
 *            (签名与 Route_MinStep 对齐, 以后新增调用点直接传自己的值即可。)
 *         ⭐ 总开关 ROUTE_12_MINSTEP_ENABLE(当前 = 1): 置 0 时本函数体为空
 *            —— 调用点一行都不用改, 4 处都不挪。
 *         只做“挪一步”这一段, 【不发】转向指令 —— 转向仍由调用方紧接着发,
 *         这样 CORRECT_* 各自原来的“非阻塞发转向 + 转移条件里等
 *         Chassis_Task_Is_Complete()”写法完全不变, 只是前面多挪了一小步。
 *         ⚠️ 必须【阻塞等到位】才返回: 这一步还没走完就发转向, 转向会把这一段
 *            覆盖掉(Chassis_Rotate 内部 s_moving=false)。
 *         ⚠️ 用 Mission_Coop_Wait 等(不是 HAL_Delay): 期间照刷陀螺仪 yaw、照收 K230 行,
 *            否则航向保持/转向环读到冻结角度。3.5s 超时兜底(与底盘自己的单段超时对齐)。
 */
static void Route_BackMinStep(int32_t mm)
{
#if ROUTE_12_MINSTEP_ENABLE
    uint32_t t0;
    float yaw_before;

    if (mm == 0) {
        return;   /* 步长 0: 不挪这一步(方案二的“区间内”就是走这条路) */
    }
    yaw_before = Chassis_GetYaw();   /* 横移那段攒下的航向偏差(挪之前) */
    t0 = HAL_GetTick();
    if (mm > 0) {
        Chassis_Move_Forward(mm);
    } else {
        Chassis_Move_Backward(-mm);
    }
    while (!Chassis_Task_Is_Complete() && (HAL_GetTick() - t0) < 3500u) {
        Mission_Coop_Wait(20);
    }
    /* 这一步【必须】自己打日志: 底盘的 MOVE 行是中断里攒、由
     * Chassis_FlushPendingLog() 打印的, 而那个函数本工程【没有任何地方调用】
     * (既有问题, 不是这次改的) ⇒ 这段挪的距离不会自己出现在日志里。
     * 读这两个 yaw:
     *   挪之前 = 这段右移一共攒了多少航向偏差(判断“偏上”的根因, 期望越小越好);
     *   挪之后 = 这一步自己又把车头带偏/纠回了多少(航向保持一直在纠, 所以
     *            它通常已经吃掉一部分) —— 剩下的就交给紧接着的转正动作。
     * ⭐ 日志里带“车头前进/车头后退”字样, 实车一眼就能核对方向有没有给反。 */
    MLOG("打靶走位: 校正前先挪 %dmm(%s; 耗时 %lums; yaw %.1f° -> %.1f°)",
         (int)mm, (mm > 0) ? "车头前进" : "车头后退",
         (unsigned long)(HAL_GetTick() - t0),
         (double)yaw_before, (double)Chassis_GetYaw());
#else
    /* ⭐ 2026-10-09: ROUTE_12_MINSTEP_ENABLE = 0 ⇒ 4 处都不挪这一小步, 直接转正。
     * 前后漂移若还需要补, 就走 ROUTE_12_RIGHT_BACK_COMP 那条“全程均匀补”
     * (两条一般只开一条)。 */
    (void)mm;   /* 开关关掉时本参数用不到: 显式吃掉, 免得报“未使用参数” */
#endif
}

/* ---- ⭐⭐ 2026-10-09 新增: “挪一小步”的【方案选择】(第 2 / 第 4 处用) -------------
 * 见 ROUTE_12_A2_STEP_MODE / ROUTE_12_B2_STEP_MODE 处的说明: 每处两个方案, 各自独立。
 * 这两个函数就是“按当前偏航角算步长”的入口, 供那两个 CORRECT_* 的进入动作调用;
 * 第 1 / 第 3 处只有固定值, 调用点直接传 ROUTE_12_A_MINSTEP_MM / _B_MINSTEP_MM。 */

/**
 * @brief  方案二: 按【当前偏航角】选“挪一小步”的步长
 * @param  pos_mm yaw > +门限(车头偏左)时用它的值(约定为负数 = 车头后退)
 * @param  neg_mm yaw < −门限(车头偏右)时用它的值(约定为正数 = 车头前进)
 * @param  mid_mm |yaw| ≤ 门限(车头基本正)时用它的值(约定 0 = 不挪)
 * @return 选中的步长(mm)
 * @note   门限 = ROUTE_12_YAW_LIMIT_DEG(只看绝对值); 判据 = Chassis_GetYaw()
 *         (打靶走位阶段航向基准是 0°, 所以读数本身就是偏差)。
 *         ⚠️ 两处都选方案一(或总开关关闭)时它就没被调用 ⇒ 显式标注 unused,
 *            免得编译器报“定义但未使用”(两种开关组合都要能干净编译)。
 */
static int32_t __attribute__((unused)) Route_MinStepMM_ByYaw(int32_t pos_mm, int32_t neg_mm, int32_t mid_mm)
{
    float limit = ROUTE_12_YAW_LIMIT_DEG;
    float yaw   = Chassis_GetYaw();

    if (limit < 0.0f) {
        limit = -limit;   /* 门限只看大小(写成 -1.1 也照样管两个方向) */
    }
    if (yaw >  limit) return pos_mm;
    if (yaw < -limit) return neg_mm;
    return mid_mm;
}

/**
 * @brief  第 2 处(STATE_12_PART1_CORRECT_A2) 这次该挪多少(mm)
 * @note   方案一(ROUTE_12_A2_STEP_MODE = 0, ★当前实车选择): 固定值
 *         ROUTE_12_A2_MINSTEP_MM(-15);
 *         方案二(= 1): 按当前偏航角在 ROUTE_12_A2_YAW_{POS,NEG,MID}_MM 里选。
 */
static int32_t Route_A2_MinStepMM(void)
{
#if ROUTE_12_A2_STEP_MODE
    int32_t mm = Route_MinStepMM_ByYaw(ROUTE_12_A2_YAW_POS_MM,
                                       ROUTE_12_A2_YAW_NEG_MM,
                                       ROUTE_12_A2_YAW_MID_MM);
    MLOG("打靶走位[方案二·第2处]: yaw=%.2f° (门限 ±%.2f°) ⇒ 挪 %+dmm",
         (double)Chassis_GetYaw(), (double)ROUTE_12_YAW_LIMIT_DEG, (int)mm);
    return mm;
#else
    return ROUTE_12_A2_MINSTEP_MM;   /* 方案一: 固定步长(稳定纠正) */
#endif
}

/**
 * @brief  第 4 处(STATE_12_PART1_CORRECT_B2) 这次该挪多少(mm)
 * @note   方案一(ROUTE_12_B2_STEP_MODE = 0, ★当前实车选择): 固定值
 *         ROUTE_12_B2_MINSTEP_MM(-15);
 *         方案二(= 1): 按当前偏航角在 ROUTE_12_B2_YAW_{POS,NEG,MID}_MM 里选。
 *         ⚠️ 与第 2 处【完全独立】: 改这边的开关/取值不影响上面那处。
 */
static int32_t Route_B2_MinStepMM(void)
{
#if ROUTE_12_B2_STEP_MODE
    int32_t mm = Route_MinStepMM_ByYaw(ROUTE_12_B2_YAW_POS_MM,
                                       ROUTE_12_B2_YAW_NEG_MM,
                                       ROUTE_12_B2_YAW_MID_MM);
    MLOG("打靶走位[方案二·第4处]: yaw=%.2f° (门限 ±%.2f°) ⇒ 挪 %+dmm",
         (double)Chassis_GetYaw(), (double)ROUTE_12_YAW_LIMIT_DEG, (int)mm);
    return mm;
#else
    return ROUTE_12_B2_MINSTEP_MM;   /* 方案一: 第 4 处自己的固定步长 */
#endif
}

/**
 * @brief  打靶走位: 带“向后分量”的右移(走位链的 4 个小段统一从这走)
 * @param  mm 右移距离(mm, 正数 = 向右)
 * @note   把 ROUTE_12_RIGHT_BACK_COMP 的比例交给底盘 ⇒ 整段右移是斜着走的,
 *         边走边把“往上漂”的量按比例补掉(原理见 ROUTE_12_RIGHT_BACK_COMP 说明)。
 *         该宏填 0 时等价于普通 Chassis_Move_Right。
 */
static void Target_MoveRight(int32_t mm)
{
    Chassis_Move_Right_WithBack(mm, ROUTE_12_RIGHT_BACK_COMP);
}

/**
 * @brief  救援段: 航向纠正【之前】的“挪最小一步”(阻塞版, 可正可负)
 * @param  mm >0 = 车头前进 | <0 = 车头后退 | 0 = 直接返回(关闭这一步)
 * @note   ⭐ 2026-10-07 新增, 动机/取值/方向见 ROUTE_RESCUE_AFTER_TURN_MM / ROUTE_RESCUE_FWD_MM 处的说明。
 *         作用状态(各处航向纠正) —— 车头 0° 的: STATE_11A_HEADING_CORRECT(排爆后,
 *         ★2026-10-09 新增) / STATE_12_PART1_CORRECT_A2 / _CORRECT_B2;
 *         车头 -90° 的(阶段四): STATE_15_TURN_FOR_HOSTAGE(②) /
 *         STATE_15D_RESCUE_RECORRECT(③'右移前, ★2026-10-09 新增, 本处填 0 = 只校不挪) /
 *         STATE_15B2_RESCUE_MID_CORRECT(④'右移中途, ★2026-10-09 新增) /
 *         STATE_17A_RESCUE_HEADING_CORRECT(⑨) / STATE_18A_RESCUE_HEADING_CORRECT(⑪)。
 *         ⭐ 2026-10-10: ROUTE_RESCUE_GRAB_BACK_MM(到人质处、识别前先退一步)【不再】走本函数
 *            —— 它已经挪进 STATE_16_RESCUE_RIGHT_A 的进入动作, 在那里自己阻塞等到位。
 *         只做“挪一步”这一段, 【不发】转向指令 —— 转向仍由调用方紧接着发,
 *         这样各状态原来的“非阻塞发转向 + 转移条件里等
 *         Chassis_Task_Is_Complete()”写法完全不变, 只是前面多走了一小步。
 *         ⚠️ 必须【阻塞等到位】才返回: 这一步还没走完就发转向, 转向会把它
 *            覆盖掉(Chassis_Rotate 内部 s_moving=false)。
 *         ⚠️ 用 Mission_Coop_Wait 等(不是 HAL_Delay): 期间照刷陀螺仪 yaw、照收 K230 行,
 *            否则航向保持/转向环读到冻结角度。3.5s 超时兜底(与底盘自己的单段超时对齐)。
 */
static void Route_MinStep(int32_t mm)
{
    uint32_t t0;
    float yaw_before;
    int32_t cnt_before;
    int32_t cnt_after;
    int32_t moved_mm;

    if (mm == 0) {
        return;   /* 关掉这一步: 直接回“到位就直接转正”的老行为 */
    }
    yaw_before = Chassis_GetYaw();   /* 平移那段攒下的航向偏差(挪之前) */
    /* ⭐⭐ 2026-10-11 新增(用户反馈"转弯后看不到车往前走"): 发指令前记一份四轮
     *   平均位置, 走完再相减 → 日志里直接给出【实测走了多少毫米】。
     *   为什么要: 光看"耗时 900ms" 分不清"指令发了没动"和"动了几十毫米没看清"
     *   (底盘自己的 MOVE 日志不会打印, 见下)。四轮取平均 = 平移净位移;
     *   若实测远小于请求值, 那才是真的没走(该查限速/死区/打滑/被转向环顶掉)。 */
    cnt_before = (Chassis_GetWheelCount(0) + Chassis_GetWheelCount(1) +
                  Chassis_GetWheelCount(2) + Chassis_GetWheelCount(3)) / 4;
    t0 = HAL_GetTick();
    if (mm > 0) {
        Chassis_Move_Forward(mm);
    } else {
        Chassis_Move_Backward(-mm);
    }
    while (!Chassis_Task_Is_Complete() && (HAL_GetTick() - t0) < 3500u) {
        Mission_Coop_Wait(20);
    }
    cnt_after = (Chassis_GetWheelCount(0) + Chassis_GetWheelCount(1) +
                 Chassis_GetWheelCount(2) + Chassis_GetWheelCount(3)) / 4;
    moved_mm = (int32_t)((float)(cnt_after - cnt_before) / CH_COUNTS_PER_MM
                         + ((cnt_after >= cnt_before) ? 0.5f : -0.5f));
    /* 这一步【必须】自己打日志: 底盘的 MOVE 行是中断里攒、由
     * Chassis_FlushPendingLog() 打印的, 而那个函数本工程【没有任何地方调用】
     * (既有问题, 不是这次改的) ⇒ 这一段挪的距离不会自己出现在日志里。
     * 读这两个 yaw: 挪之前 = 这段平移攒了多少航向偏差; 挪之后 = 这一步
     * 被航向保持纠掉/带偏了多少(剩下的交给紧接着的转正动作)。
     * ⭐ 日志里带“车头前进/车头后退”字样, 实车一眼就能核对方向有没有给反;
     *    “实测”是四轮计数反算出来的真实位移, 正常应 ≈ 请求值(差 3~5mm 属死区)。 */
    MLOG("救援: 纠正前先挪 %dmm(%s; 实测 %+dmm; 耗时 %lums; yaw %.1f° -> %.1f°)",
         (int)mm, (mm > 0) ? "车头前进" : "车头后退", (int)moved_mm,
         (unsigned long)(HAL_GetTick() - t0),
         (double)yaw_before, (double)Chassis_GetYaw());
    /* 实测明显不足(不足请求的 70%)= 指令发了但车几乎没走, 单独立一条醒目的警告,
     * 免得淹没在上面那行里(常见原因: 被转向环的差速顶掉 / 轮子打滑 / 顶到边界)。 */
    if (moved_mm < 0) moved_mm = -moved_mm;
    {
        int32_t want = (mm < 0) ? -mm : mm;
        if (moved_mm * 10 < want * 7) {
            MLOG("⚠ 救援: 这一步只走了 %dmm(请求 %dmm) —— 检查限速/死区/打滑, 或上传日志核对",
                 (int)moved_mm, (int)want);
        }
    }
}

/* ================= 机械臂动作序列 ================= */

/* ---- ⭐ 2026-10-08 新增: “进容差”确认帧计数 + 最近一帧残差(结束对准时打日志用) ----
 * 作用: ① s_align_done_cnt 实现 ALIGN_DONE_FRAMES 的连续确认;
 *       ② s_align_last_* 让“结束对准那一刻到底差多少”出现在日志里:
 *          收到 OK / 超时放弃 / 进容差 三条路都会把它打出来,
 *          一眼就能区分“真对准了”和“没对准也执行了”。
 * ⚠️ 这些变量【定义在机械臂动作序列之前】: 因为抓球/放桶的 Y→ID2 补偿
 *    (BombFbCapture, 见下)也要读 s_align_last_*, 而它比 Vision_FineAlignProcess
 *    更靠前 —— 放前面才能被两边都引用到。 */
static uint8_t s_align_done_cnt   = 0;
static int32_t s_align_last_lr    = 0;
static int32_t s_align_last_fb    = 0;
static uint8_t s_align_last_valid = 0;     /* 本轮是否读到过 D 帧 */

/** @brief 新一轮精对准开始时复位“确认帧计数 / 最近残差” */
static void AlignDwell_Reset(void)
{
    s_align_done_cnt   = 0;
    s_align_last_lr    = 0;
    s_align_last_fb    = 0;
    s_align_last_valid = 0;
}

/* ---- ⭐ 2026-10-09: 抓球/放桶的 Y 轴 → 机械臂 ID2 补偿 ----
 * s_bomb_fb_px : X 精对准成功那一刻抓到的 Y 误差(px), 由状态机在对准成功时拷贝;
 * s_bomb_fb_valid : 1 = 该值有效(本轮精对准确实读到过 D 帧) */
static int32_t s_bomb_fb_px    = 0;
static uint8_t s_bomb_fb_valid = 0;

/**
 * @brief  把状态机在对准成功时抓到的 Y 误差拷贝进来(供抓取/放置前补偿用)
 * @note   必须在【对准成功、准备抓取/放置】的那个瞬间调; 之后 BOMB_SETTLE
 *         只做计时等待、不再读 K230 行, 所以这里拷贝下来的值到抓取前都不会变。
 *         没读到过 D 帧(s_align_last_valid=0, 例如 K230 直接回了 OK)时,
 *         置 valid=0 → 不补偿, 保证不会拿旧任务的误差乱补。
 */
static void BombFbCapture(void)
{
    s_bomb_fb_valid = s_align_last_valid;
    s_bomb_fb_px    = s_align_last_fb;
}

/**
 * @brief  摆到指定姿态, 并把 Y 误差补偿叠加到 ID2 上再下发(球/桶用)
 * @param  pose_idx 目标姿态(BALL_PRE/CLOSE 或 PLACE_PRE/OPEN)
 * @param  cfg      本任务的参数(球 = s_fb_arm_ball, 桶 = s_fb_arm_bucket)
 * @note   与 Arm_GotoPose 的唯一差别: ID2 多叠一个补偿码并限幅。
 *         补偿码为 0(误差太小/未开启/无效)时行为与 Arm_GotoPose 完全一致。
 *         误差值 = s_bomb_fb_px(对准成功那一刻由 BombFbCapture 拷进来的)。
 */
static void Arm_GotoPoseYComp(uint8_t pose_idx, const FbArmCfg_t *cfg)
{
    const uint16_t *src = ArmPose_Ptr(pose_idx);
    uint16_t        pose[SERVO_COUNT];
    uint8_t         i;

    if (src == NULL) {
        MLOG("机械臂: 姿态编号非法 %d", (int)pose_idx);
        return;
    }
    for (i = 0; i < SERVO_COUNT; i++) {
        pose[i] = src[i];
    }
    if (s_bomb_fb_valid) {
        FbArmCompApply(cfg, s_bomb_fb_px, pose, pose_idx);
    }
    Arm_GotoPoseBuffer(pose, pose_idx);
}

/**
 * @brief  ⭐⭐ 2026-10-11(用户要求): 同 Arm_GotoPoseYComp, 但 ID1 换成【锁存值 + 手动偏移】
 * @note   只给【放球】那两个姿态用(PLACE_PRE / PLACE_OPEN):
 *         桶精对准阶段可以用小步转 ID1 修轻微横向误差(见 Bucket_LrFineWithId1),
 *         对准成功那一刻把这个 ID1 锁存下来(s_bucket_id1_lock), 放球时用它
 *         ⇒ "对准时底座转到哪, 放球就在哪"(与救援 Arm_GotoRescuePoseKeepId1 同一套做法)。
 *         没锁存过(测试模式 / 超时直接放球)时退回姿态表标定值, 与老行为一致。
 */
static uint16_t Arm_BucketPlaceId1(uint8_t pose_idx);   /* 定义在下面"机械臂 ID1"那一段 */

static void Arm_GotoPoseYCompId1(uint8_t pose_idx, const FbArmCfg_t *cfg)
{
    const uint16_t *src = ArmPose_Ptr(pose_idx);
    uint16_t        pose[SERVO_COUNT];
    uint8_t         i;

    if (src == NULL) {
        MLOG("机械臂: 姿态编号非法 %d", (int)pose_idx);
        return;
    }
    for (i = 0; i < SERVO_COUNT; i++) {
        pose[i] = src[i];
    }
    pose[0] = Arm_BucketPlaceId1(pose_idx);    /* [0] = ID1 底座(锁存值 + 手动偏移) */
    if (s_bomb_fb_valid) {
        FbArmCompApply(cfg, s_bomb_fb_px, pose, pose_idx);
    }
    MLOG("机械臂: %s 的 ID1 = 锁存 %ld %+d(手动偏移) %+d(标定差) = %d",
         ArmAction_GetName(pose_idx),
         (long)(s_bucket_id1_lock_valid ? s_bucket_id1_lock : 0),
         (int)BUCKET_PLACE_ID1_OFFSET,
         (int)((s_bucket_id1_lock_valid && !BUCKET_PLACE_ID1_USE_LOCK)
                   ? ((int32_t)src[0] - s_bucket_id1_lock_base) : 0),
         (int)pose[0]);
    Arm_GotoPoseBuffer(pose, pose_idx);
}

/**
 * @brief  排爆第一步: 抓小球(BALL_PRE → 夹紧 → 抬起)
 * @note   调用前机械臂应已在 ARM_POSE_BALL_LOOK(看球), 且底盘已视觉对准。
 *         ⭐ 2026-10-09: BALL_PRE/CLOSE 会按“X 精对准成功那一刻的 Y 误差”
 *         对 ID2 做前后补偿(见 BombFbCapture / Arm_GotoPoseYComp)。
 */
void Arm_Start_Bomb_Grab(void)
{
    MLOG("机械臂: 排爆抓球(到球前→夹紧→抬起)");
    Arm_GotoPoseYComp(ARM_POSE_BALL_PRE,   &s_fb_arm_ball);
    Arm_GotoPoseYComp(ARM_POSE_BALL_CLOSE, &s_fb_arm_ball);
    Arm_GotoPose(ARM_POSE_BALL_LIFT);
}

/**
 * @brief  排爆第二步: 放球(PLACE_PRE → 松开 → 抬起)
 * @note   调用前机械臂应已在 ARM_POSE_BUCKET_LOOK(看桶), 且底盘已对准球桶
 *         ⭐⭐ 2026-10-11: ID1 用【该姿态标定值 + 桶对准累计的 ID1 偏移】
 *         (见 Arm_BucketPlaceId1) —— 若对准时用 ID1 修过轻微横向误差,
 *          放球动作要跟着修同样的量, 否则前面白修。
 */
void Arm_Start_Bomb_Place(void)
{
    MLOG("机械臂: 排爆放球(到桶前→松开→抬起)");
    Arm_GotoPoseYCompId1(ARM_POSE_PLACE_PRE,  &s_fb_arm_bucket);
    Arm_GotoPoseYCompId1(ARM_POSE_PLACE_OPEN, &s_fb_arm_bucket);
    /* ⭐ PLACE_LIFT 实测会剥蹭: 拆成两步 —— 先 ID1/ID2/ID3 转到位, 再动 ID4/ID5 */
    Arm_GotoPoseSplit(ARM_POSE_PLACE_LIFT, SERVO_MASK_ARM_BODY);
}

/* =====================================================================
 * ⭐ 扫码“多角度扫描” (2026-10-06 新增)
 * ---------------------------------------------------------------------
 * 为什么: 只用一个固定姿态扫码, 遇到反光/角度偏/K230 视野边缘就扫不到。
 *         用腕部 ID4 微调出 3 个角度轮着扫, 命中率高很多。
 * 怎么做: 进入 STATE_2 先摆 ARM_POSE_SCAN(其余 4 个舵机保持不动),
 *         之后每 SCAN_MOVE_MS + SCAN_HOLD_MS 用掩码【只改 ID4】换一个角度:
 *             ID4 = SCAN + 0  →  SCAN - SCAN_ID4_DELTA  →  SCAN + SCAN_ID4_DELTA
 *         每个角度停 SCAN_HOLD_MS 让 K230 拍照识别; 期间它若回 SCAN_OK
 *         就立刻结束本状态(不等三个角度走完)。
 * 非阻塞: 由 STATE_2 的转移检查每周期驱动(Arm_Scan_Poll), 不占主循环。
 * ===================================================================== */
static const int16_t s_scan_id4_off[3] = { 0, -SCAN_ID4_DELTA, +SCAN_ID4_DELTA };
static uint8_t  s_scan_idx = 0;            /* 已发出的角度序号; >=3 = 已扫完 */
static uint32_t s_scan_hold_until = 0;     /* 下一个角度最早什么时候可以发 */

/** @brief 扫码开始: 摆好 SCAN 姿态, 并把多角度扫描复位到第 1 个角度 */
void Arm_Scan_Begin(void)
{
    Arm_GotoPose(ARM_POSE_SCAN);           /* 整表下发(阻塞), 其余 4 个舵机就位 */
    s_scan_idx        = 0;
    s_scan_hold_until = HAL_GetTick();     /* 允许立刻发第 1 个角度 */
    MLOG("扫码: 开始多角度扫描 (ID4 偏移 %+d/%+d/%+d, 共 3 个角度, 每个停 %dms)",
         (int)s_scan_id4_off[0], (int)s_scan_id4_off[1], (int)s_scan_id4_off[2],
         (int)SCAN_HOLD_MS);
}

/**
 * @brief 扫码多角度扫描推进 (非阻塞; 放在 STATE_2 的转移检查里每周期调用)
 * @note  三个角度都发完后(s_scan_idx >= 3)直接返回, 之后就保持最后一个角度
 *        继续等 K230 回 SCAN_OK。
 */
void Arm_Scan_Poll(void)
{
    uint16_t pose[SERVO_COUNT];
    const uint16_t *scan = s_arm_pose_table[ARM_POSE_SCAN];
    int32_t v;

    if (s_scan_idx >= 3u) {
        return;                            /* 三个角度都扫过了 */
    }
    if ((int32_t)(HAL_GetTick() - s_scan_hold_until) < 0) {
        return;                            /* 还在转动/停留中 */
    }

    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        pose[i] = scan[i];                 /* 其余 4 个舵机仍填 SCAN 姿态值 */
    }
    v = (int32_t)scan[3] + (int32_t)s_scan_id4_off[s_scan_idx];
    if (v < 0)    v = 0;
    if (v > 4095) v = 4095;
    pose[3] = (uint16_t)v;                 /* [3] = ID4 腕部 */

    MLOG("扫码: 角度 %u/3 (ID4=%d, 偏移 %+d), 停 %dms 给 K230 识别",
         (unsigned)(s_scan_idx + 1u), (int)pose[3],
         (int)s_scan_id4_off[s_scan_idx], (int)SCAN_HOLD_MS);

#if MISSION_TEST_NO_ARM
    MLOG("扫码: MISSION_TEST_NO_ARM=1, 跳过舵机驱动");
#else
    Servos_SetPositionsMasked(pose, SERVO_MASK_ID4, SCAN_MOVE_MS);
#endif

    s_scan_idx++;
    s_scan_hold_until = HAL_GetTick() + (uint32_t)SCAN_MOVE_MS + (uint32_t)SCAN_HOLD_MS;

    if (s_scan_idx >= 3u) {
        MLOG("扫码: 3 个角度扫完, 停在最后一个角度继续等 SCAN_OK");
    }
}

/** @brief 打靶: 摆到激光发射位(调用后由状态机开激光) */
void Arm_Start_Target_Fire(void)
{
    MLOG("机械臂: 摆到激光发射位");
    Arm_GotoPose(ARM_POSE_TARGET_FIRE);
}

/** @brief 打靶: 发射完把大臂抬起 */
void Arm_Start_Target_Lift(void)
{
    MLOG("机械臂: 打完靶, 大臂抬起");
    /* ⭐ 2026-10-06: TARGET_LIFT 是全场负载最重的动作(见 s_arm_pose_time 注释),
     *    而它之前刚经历“抓球→放球→抰TARGET_LOOK→精对准”一连串大电流动作。
     *    实测“抬完大臂后臂薓下去” → 疑似峰值电流/过热。
     *    所以改成【分两步摆】:
     *      第1步 掩码 SERVO_MASK_ARM_BODY: 先让 ID1/ID2/ID3(负载最重的三个关节)
     *             转到位并停稳(避免了五个舵机同时启动的电流峰值);
     *      第2步 剩下的 ID4/ID5 再动(这时大臂已停稳, 负载小得多)。
     *    判定: 若这样就不薓了 → 基本确定是供电/峰值电流问题;
     *          若照样薓     → 往舵机扭矩/过热/机械结构方向查。
     *    ⚠️ 代价: 比原来多花几秒(两步各自的运动时间 + 保持时间)。验证完可以
     *       改回 Arm_GotoPose(ARM_POSE_TARGET_LIFT) 省时间。 */
    Arm_GotoPoseSplit(ARM_POSE_TARGET_LIFT, SERVO_MASK_ARM_BODY);
}

/* ⭐ 精对准/微调过程中 ID1(底座) / ID4(腕部) 相对【基准姿态】的累计偏移(角度码)。
 * 为什么要有这个量:
 *   ① 打靶 —— 精对准是用“只发 ID1” / “只发 ID4” 的掩码方式微调的, 而最后摆
 *      发射位是【整表下发】, 会把 ID1/ID4 拉回姿态表标定值 ⇒ 对准白做。
 *      所以摆发射位时要把偏移叠加回 TARGET_FIRE(见 Target_GotoFirePose)。
 *   ② 救援 —— 抓取位要根据 ID1 一共转了多少、朝哪边转来三选一
 *      (见 Rescue_GrabPoseFor / Arm_Start_Rescue_Grab)。
 * 定义放在这里(而不是文件后半段)是为了让 Arm_Start_Rescue_Grab() 能用;
 * 且放在条件编译之外, 保证 MISSION_TEST_NO_VISION 下也能编译。
 * ⚠️ 2026-10-10: Rescue_Id1Norm / Rescue_GrabSideFor 为了给“按侧别取抓取偏移”
 *    用, 已上移到 Rescue_GrabId1Code() 之前, 不在这里了。 */
static int32_t s_id1_offset = 0;
static int32_t s_id4_offset = 0;

/* 三个方向 × 三个动作 的姿态号查找表(下标 = Rescue_GrabSideFor 的返回值) */
static const uint8_t s_rescue_grab_pose[3] = {
    ARM_POSE_HOSTAGE_GRAB_L, ARM_POSE_HOSTAGE_GRAB_M, ARM_POSE_HOSTAGE_GRAB_R
};
static const uint8_t s_rescue_close_pose[3] = {
    ARM_POSE_HOSTAGE_GRAB_L_CLOSE, ARM_POSE_HOSTAGE_GRAB_M_CLOSE, ARM_POSE_HOSTAGE_GRAB_R_CLOSE
};
static const uint8_t s_rescue_lift_pose[3] = {
    ARM_POSE_HOSTAGE_GRAB_L_LIFT, ARM_POSE_HOSTAGE_GRAB_M_LIFT, ARM_POSE_HOSTAGE_GRAB_R_LIFT
};

/* 本次救援选中的侧别(0=左 1=中 2=右), 给后面的“抬起”用。
 * 默认 1(中): 万一 Arm_Start_Rescue_Grab 没被调到而先调了 Retract, 也不会乱跑 */
static uint8_t s_rescue_grab_side = 1u;

/**
 * @brief  救援: ① 摆到三选一的抓取位 → ② 原位抱紧
 * @note   ⭐ 拆成两个姿态的原因: 三个方向的抓取姿态差得很大(左/右位 ID1 差
 *         约 400 码 ≈ 35°, ID2/ID3/ID4 也各不相同), “合夹爪”这一个动作
 *         在这三个姿态上并不是同一个位置, 用一套会拉回中间位或蹭到车架。
 *         ⇒ ①GRAB_x / ②GRAB_x_CLOSE / ③GRAB_x_LIFT 各自独立标定。
 *         ⭐⭐⭐ 2026-10-07 改: 下发改用 Arm_GotoRescuePoseKeepId1 ——
 *         【底座 ID1 用“刚刚对准好”的锁存值, 不跟姿态表里的 ID1 走】(用户要求)。
 *         所以这三行的【第 1 列只当兵底用】(没锁过时才会用); 真正决定
 *         抓取姿态的是 ID2~ID5。
 */
void Arm_Start_Rescue_Grab(void)
{
    /* ⭐ 侧别用【归一化后的 ID1】判断(而不是拿绝对角度码直接比):
     *    对准完成时锁存的是绝对角度码(如 1561), 必须先归一化(1561 → -498)
     *    才能看出它偏左还是偏右 —— 见 Rescue_Id1Norm 的注释。
     *    没锁过(测试模式等)就退回用累计偏移 s_id1_offset(它本来就是相对基准的偏移)。 */
    int32_t off = s_rescue_id1_valid ? Rescue_Id1Norm(s_rescue_id1_lock)
                                     : s_id1_offset;

    s_rescue_grab_side = Rescue_GrabSideFor(off);

    if (s_rescue_id1_valid) {
        MLOG("机械臂: 救援抓取 —— 锁存 ID1 = %ld (归一化偏移 %+ld 码, 负=左/正=右) "
             "-> 用 %s (居中判定 ±%d 码)",
             (long)s_rescue_id1_lock, (long)off,
             ArmAction_GetName(s_rescue_grab_pose[s_rescue_grab_side]),
             RESCUE_GRAB_MID_RANGE);
    } else {
        MLOG("机械臂: 救援抓取 —— 未锁存 ID1, 用累计偏移 %+ld 码 -> 用 %s (居中判定 ±%d 码)",
             (long)off, ArmAction_GetName(s_rescue_grab_pose[s_rescue_grab_side]),
             RESCUE_GRAB_MID_RANGE);
    }
    if (off > (int32_t)RESCUE_GRAB_FAR_WARN ||
        off < -(int32_t)RESCUE_GRAB_FAR_WARN) {
        MLOG("机械臂: ⚠ ID1 归一化偏移 %+ld 码已超出三个抓取位的覆盖范围(±%d 码) —— "
             "人质太偏或对准跑飞了, 这次可能抱不准(先查 RESCUE_ID1_LR_SIGN 方向对不对)",
             (long)off, (int)RESCUE_GRAB_FAR_WARN);
    }

    /* ① 摆到三选一的抓取位(底座 ID1 保持对准位不动)
     *    ⭐ 2026-10-10: 抓取位/抱紧位都按 Y(前后)误差补 ID2 里程(见 s_fb_arm_rescue) */
    if (RESCUE_FB_ARM_ENABLE) {
        MLOG("救援: 抓取里程补偿 —— Y(前后)误差 %ldpx%s; 分档 %d/%d/%d px -> %d/%d/%d 码, 补 ID%d",
             (long)(s_rescue_d_valid ? s_rescue_fb_px : 0),
             s_rescue_d_valid ? "" : " (⚠ 没读到过 D 帧 -> 本步不补)",
             (int)RESCUE_FB_ARM_FAR_PX, (int)RESCUE_FB_ARM_MID_PX, (int)RESCUE_FB_ARM_NEAR_PX,
             (int)RESCUE_FB_ARM_FAR_CODE, (int)RESCUE_FB_ARM_MID_CODE, (int)RESCUE_FB_ARM_NEAR_CODE,
             (int)(RESCUE_FB_ARM_SERVO + 1u));
    }
    Arm_GotoRescuePoseKeepId1(s_rescue_grab_pose[s_rescue_grab_side], &s_fb_arm_rescue);
    /* ② 在原位合夹爪抱紧(每个方向一套, 由示教标定; 同样不动 ID1) */
    Arm_GotoRescuePoseKeepId1(s_rescue_close_pose[s_rescue_grab_side], &s_fb_arm_rescue);
}

/**
 * @brief  救援: 抱起人质后抬起(用【本次选中方向】的那一套抬起姿态)
 * @note   ⚠️ 三个方向的抬起姿态分别标定: 左/右位手臂偏得多, 用同一张
 *         HOSTAGE_LIFT 会把底座拉回中间 → 抱着人质硬掰回去, 很容易抖掉。
 *         ⭐ 2026-10-07: 同样用 Arm_GotoRescuePoseKeepId1 —— 抬起时底座 ID1
 *         也保持对准位不动(抱着人质时底座突然转一下最容易把人质甩掉)。
 */
void Arm_Start_Rescue_Retract(void)
{
    MLOG("机械臂: 人质抱起后抬起 —— 用 %s (侧别 %u)",
         ArmAction_GetName(s_rescue_lift_pose[s_rescue_grab_side]),
         (unsigned)s_rescue_grab_side);
    Arm_GotoRescuePoseKeepId1(s_rescue_lift_pose[s_rescue_grab_side], NULL);   /* 抬起位: 不补里程 */
}

/**
 * @brief  救援: 抱起人质后执行【回程姿态】(三个方向共用)
 * @note   ⚠️ 顺序是 抓取位 → 抱紧位 → 抬起位 → 回程姿态, 回程姿态之后
 *         主状态机才发【右移】指令(车头已右转 90°)。带着人质动作, 所以用限幅下发。
 */
void Arm_Start_Rescue_Return(void)
{
    MLOG("机械臂: 切换到回程姿态(收臂, 准备带人质横移)");
    Arm_GotoPoseClamped(ARM_POSE_HOSTAGE_RETURN);
}

/**
 * @brief  任务初始化: 恢复到待机状态
 */
void Mission_Init(void)
{
    g_mission_state = MISSION_STATE_IDLE;
    g_vision_task_in_progress = 0;
    g_qr_code_string[0] = '\0';
    /* ⭐ 重新开始: 丢弃上一次跑残留的位置误差(原来是 Chassis_Stop() 里顺手做的,
     * 现在 Chassis_Stop() 改为保留残差, 所以必须在这里显式复位) */
    Chassis_SyncTarget();
    /* ⭐ 平移时的航向基准 = 0°(与全程各处 "航向校正到 0°" 一致)。
     * 以前是每段平移各自以"当前朝向"为基准 → 校不完的误差被继承, 越跑越偏 */
    Chassis_GyroBias_Stop();      /* ⭐ 2026-10-11: 复位零偏估计(回未起锚状态), 保证干净上电 */
    Chassis_SetHeadingRef(0.0f);
    Chassis_Stop();
    Laser_Off();
    LED_OFF();
#if MISSION_TEST_NO_ARM
    MLOG("机械臂: 已禁用(测试模式), 跳过回初始位");
#else
    /* ⭐ 回初始姿态只发指令, 不在启动流程里死等:
     *   舵机自己会按 SERVO_HOME_MOVE_MS 慢慢走回, 主循环/按键照常响应。
     *   (以前 这里的 HAL_Delay(10000) 会把上电后的 10 秒全卡住) */
    Servos_SetPositions(Servos_GetHomePositions(), SERVO_HOME_MOVE_MS);
#endif
}

/**
 * @brief  启动任务
 */
void Mission_Start(void)
{
    if (g_mission_state == MISSION_STATE_IDLE || g_mission_state == MISSION_STATE_COMPLETE) {
        MLOG("========== 任务开始 ==========");
        g_mission_state = STATE_1_MOVING_TO_QR_SCAN;
    }
}

/* ⭐⭐ 自适应步长状态(自学习比例 a = mm/px) ----------------------------------
 *   a 是把“上一步实际走了多少 mm”除以“误差变了多少 px”实测出来的 ——
 *   喂数据的地方在 Vision_FineAlignProcess 的 [K标定] 那段。
 *   攒够 ALIGN_ADAPT_MIN_SAMPLES 步后, 步长就用它替代 K_GAIN(见
 *   Calc_Move_Distance_Adaptive), 这就是“自适应”的核心。
 *   ⚠️ 故意【不在 KCal_Reset() 里清零】: 这是镜头/车体的物理比例, 上一阶段
 *      (球)学到的值对下一阶段(桶)同样有效, 保留能少走几步弯路。 */
static float   s_adapt_a    = 0.0f;   /* 实测比例估计(mm/px); s_adapt_n==0 时无效 */
static uint8_t s_adapt_n    = 0;      /* 有效样本数(到 200 封顶, 之后只做平滑) */
static uint8_t s_adapt_over = 0;      /* 1 = 上一步冲过头了(本帧误差方向与上一步相反) */

/* ===================== ⭐ 桶“前后(深度)闭环”的状态 =====================
 * 见 BUCKET_FB_CL_* 那一段的说明。只在 cfg->fb_closed_loop=1(目前即桶)时使用。 */
static uint8_t s_fb_cl_steps  = 0;    /* 本轮已经修了几步(兜底②: 步数上限) */
static int32_t s_fb_cl_total  = 0;    /* 累计位移(mm): 正 = 车头前进(兜底②: 位移上限) */
static uint8_t s_fb_cl_moved  = 0;    /* 上一步是否真的动过(判“越修越大”的前提) */
static int32_t s_fb_cl_last_px = 0;   /* 上一步动作时的 |fb|(兜底①: 越修越大就翻极性) */
static int8_t  s_fb_cl_err_sign = 0;  /* 上一步动作依据的 fb 符号(判“跨过中心”用, 与极性无关) */
static uint8_t s_fb_cl_over   = 0;    /* 上一步跨过中心 ⇒ 这一步减半(阻尼) */
static uint8_t s_fb_cl_giveup = 0;    /* 兜底用尽: 本轮不再修前后(直接接受现状) */
/* ⚠️ 下面两个【跨阶段保留】(和 s_adapt_a 一样): 极性/“翻过没有”是物理属性,
 *    上一阶段(球)学到的对下一阶段(桶)同样有效, 保留能少走弯路。 */
static int8_t  s_fb_cl_sign   = 1;    /* 极性: +1 = 默认约定(fb>0 → 车头前进); -1 = 已翻转 */
static uint8_t s_fb_cl_flipped = 0;   /* 兜底①: 是否已经自动翻转过一次(只翻一次) */

/** @brief 桶前后闭环: 新一轮精对准前复位“本轮”的统计(极性/翻转标志刻意保留) */
static void FbCl_Reset(void)
{
    s_fb_cl_steps    = 0;
    s_fb_cl_total    = 0;
    s_fb_cl_moved    = 0;
    s_fb_cl_last_px  = 0;
    s_fb_cl_err_sign = 0;
    s_fb_cl_over     = 0;
    s_fb_cl_giveup   = 0;
}

/** @brief 兜底②: 还有没有“修前后”的预算(步数 / 累计位移都够) */
static uint8_t FbCl_BudgetLeft(void)
{
    int32_t tot = s_fb_cl_total;

    if (s_fb_cl_giveup) {
        return 0;
    }
    if (s_fb_cl_steps >= (uint8_t)BUCKET_FB_CL_MAX_STEPS) {
        return 0;
    }
    if (tot < 0) {
        tot = -tot;
    }
    return (uint8_t)(tot < (int32_t)BUCKET_FB_CL_MAX_TOTAL_MM);
}

/* ⚠️ 只在闭环开着时编译: 关掉时它的唯一调用点(状态机那条日志)也被 #if 去掉了,
 *    留着会变成“declared but never referenced”警告。 */
#if BUCKET_FB_CL_ENABLE
/** @brief 桶前后闭环: 本轮是否“放弃过”前后修正(兜底用尽) —— 只给状态机打日志用 */
static uint8_t FbCl_GaveUp(void)
{
    return s_fb_cl_giveup;
}
#endif

/**
 * @brief  桶前后(深度)轴的【闭环步长】: 自适应比例 + 过冲减半 + 按方向限幅
 * @param  fb_px  前后像素误差(正 = 目标偏远)
 * @retval 本步要走的距离(mm); 方向由调用方按 s_fb_cl_sign 决定
 * @note   比例用自学习的 s_adapt_a(攒够样本)否则 K_GAIN —— 和左右同一套“不标定”思路;
 *         兜底③: 单步上限按方向分开 —— 朝桶那侧(车头前进)用 FWD_MAX_MM 限小,
 *         离开桶那侧(车头后退)用 BACK_MAX_MM。这样即便极性判断错了, 第一步也只走 6mm。
 */
static int32_t Calc_FbClDistance(int32_t fb_px)
{
    float   a   = (s_adapt_n >= ALIGN_ADAPT_MIN_SAMPLES) ? s_adapt_a : (float)K_GAIN;
    int32_t d   = (int32_t)((float)abs(fb_px) * a * (float)ALIGN_STEP_DAMP_PCT / 100.0f);
    int32_t cap = ((fb_px * (int32_t)s_fb_cl_sign) > 0) ? (int32_t)BUCKET_FB_CL_FWD_MAX_MM
                                                         : (int32_t)BUCKET_FB_CL_BACK_MAX_MM;

    if (s_fb_cl_over) {
        d /= 2;                                  /* 上一步跨过中心 ⇒ 减半(阻尼) */
    }
    if (d < (int32_t)BUCKET_FB_CL_MIN_MM) d = (int32_t)BUCKET_FB_CL_MIN_MM;
    if (d > cap) d = cap;
    return d;
}

/**
 * @brief  像素误差 -> 底盘修正距离(【左右】轴, ⭐自适应步长)
 * @param  pixel_error 横向像素误差(+ = 目标偏画面左)
 * @param  tol_px      本阶段容差: |误差| < 它就返回 0(调用方会判“已对准”)
 * @param  min_step_mm 本阶段的【步长下限】(mm): 普通阶段 = ALIGN_STEP_MIN_MM(8),
 *                     ⭐细模式阶段(桶) = ALIGN_STEP_MIN_MM_FINE(当前也是 8) ——
 *                     见该宏的说明(为什么不能太小: 会被死区吃掉 ⇒ 个别轮子动 ⇒ 车身偏移)
 * @param  p_a_used    输出: 本步实际用的比例(mm/px), 只给日志用(可传 NULL)
 * @retval 要走的距离(mm); 0 = 不用动
 * @note   分档 / 自学习 / 冲过头减半 / 限幅的完整说明见上面 ALIGN_STEP_* 那一段注释。
 *         前后轴用后面的 Calculate_Move_Distance_FB() —— 它多一层缩放 + 
 *         自己的一套 min/max, 因为前后离目标太近, 必须比左右保守。
 */
static int32_t Calc_Move_Distance_Adaptive(int pixel_error, int tol_px,
                                           int32_t min_step_mm, float *p_a_used)
{
    int32_t a_px = abs(pixel_error);
    float   a;
    int32_t d;

    if (a_px < tol_px) {
        return 0;
    }
    /* ① 比例: 攒够实测样本就用“自学习”值, 否则用宏里的 K_GAIN */
    if (s_adapt_n >= ALIGN_ADAPT_MIN_SAMPLES) {
        a = s_adapt_a;
    } else {
        a = (float)K_GAIN;
    }
    /* ② 按误差大小分档 */
    if (a_px > ALIGN_STEP_BIG_PX) {
        d = (int32_t)ALIGN_STEP_MAX_MM;                 /* 差得远: 直接大步赶过去 */
    } else {
        d = (int32_t)((float)a_px * a * (float)ALIGN_STEP_DAMP_PCT / 100.0f);
    }
    /* ③ 上一步冲过头了 → 减半(抑制来回摆) */
    if (s_adapt_over) {
        d /= 2;
    }
    /* ④ 限幅: ⭐ 下限用【本阶段自己的】(细模式阶段允许 3mm, 普通阶段 8mm)。
     *    ⚠️ 细模式的下限(3mm)成立的前提是该阶段走的是"小到位死区"入口
     *       (Chassis_Move_*Fine, 见 AlignAxisCfg_t::fine_arrival) —— 否则 3mm 会被
     *       4.5mm 的大死区吃掉, 车基本不动。 */
    if (min_step_mm < 1) min_step_mm = 1;                  /* 防呆: 别给 0/负 */
    if (d < min_step_mm) d = min_step_mm;
    if (d > (int32_t)ALIGN_STEP_MAX_MM) d = (int32_t)ALIGN_STEP_MAX_MM;

    if (p_a_used != NULL) {
        *p_a_used = a;
    }
    return d;
}

/* =====================================================================
 * 视觉对准通用辅助(去重: 球/桶/靶/形状 复用同一套发指令/精对准逻辑)
 * 各任务状态机持有各自计时变量, 通过指针传入。
 * ===================================================================== */

/* ⭐ 本次精对准“一共把车挪了多少”(mm), 正 = 左移 / 前进。
 * 为什么要记: Chassis_Move_* 全是【增量式】目标(s_target += dc), 每段结束时
 *   chassis_freeze() 只是“丢掉残差”(目标=当前位置), 并【不会】把车开回原位。
 *   ⇒ 对准时横移/纵移走的位移是真实位置偏移, 会原样被后续每一段路线继承
 *      (后续 ROUTE_* 宏全是“从当前位置再走固定距离”)。
 *   把这个累计量打到蓝牙日志里, 就能看出对准漂了多少、要不要在
 *   BOMB_*_RETURN_MM 或后续路线宏里补回来。 */
static int32_t s_align_shift_strafe = 0;   /* 正 = 左移累计(mm) */
static int32_t s_align_shift_fwd    = 0;   /* 正 = 前进累计(mm) */

/* ⭐ 精对准累计偏移 s_id1_offset / s_id4_offset 的【定义】已上移到
 * Arm_Start_Rescue_Grab() 之前(救援要按 ID1 偏移量三选一抓取位)。
 * 这里只留用途说明:
 *   打靶精对准是用“只发 ID1” / “只发 ID4” 的掩码方式微调的, 而最后摆发射位
 *   ARM_POSE_TARGET_FIRE 是【整表下发】—— 会把 ID1/ID4 拉回姿态表里的标定值
 *   (TARGET_FIRE: ID1=2052 / ID4=2175), 精对准白做。所以摆发射位时要把这两个
 *   偏移叠加到 TARGET_FIRE 对应舵机上(见 Target_GotoFirePose)。 */

/* =====================================================================
 * 精对准“轴向 / 方向 / 步长”配置 (每个阶段一套)
 * ---------------------------------------------------------------------
 * allow_lr : 1 = 允许修【车体左右】(0 = 本阶段不修横向)
 * allow_fb : 1 = 允许修【车体前后】(0 = 本阶段不修前后)
 * x_is_fb  : 1 = 画面 X 轴对应的是车体【前后】(救援: 底座转≈90°)
 *            0 = 画面 X 轴就是车体左右
 * inv_lr   : 1 = 车体左右的修正方向取反 (桶: 底座转≈180°, 画面镜像)
 * ---- 前后(F/B)轴的独立步长 (2026-10-04 新增) ---------------------------
 * fb_scale_pct : 前后步长 = 按像素算出的步长 × 本比例(%), 100=和左右一样大
 * fb_min_mm    : 前后单步最小距离(mm)
 * fb_max_mm    : 前后单步最大距离(mm)
 *   为什么: 左右是“把目标摆正”, 前后是“要不要再往前贴”; 桶/人质就在眼前,
 *   用和左右一样大的步长(最多 MAX_MOVE_MM=60mm) 很容易冲过头甚至撞上去。
 *   ⚠️ 本组参数只在 allow_fb=1(方案二) 时参与运算。
 * ---------------------------------------------------------------------
 * 方向是怎么定下来的(和上面“视觉方向映射”那张表对应):
 *   代码里“车体左右”的默认方向是: lr_px<0 → 右移, lr_px>0 → 左移
 *   (这个方向在【球姿态】下实测是对的); “车体前后”默认是:
 *   fb_px<0 → 后退, fb_px>0 → 前进。
 *   ⇒ 球/靶 直接用默认; 桶 要 inv_lr=1 把画面镜像转回来;
 *     救援 要 x_is_fb=1, 让画面的横向(X)去驱动车身前后。
 * ⚠️ 实测哪一段“越修越偏”, 就把该阶段的 inv_lr / x_is_fb 取反对调。
 * ⚠️ 实测哪一段“前后冲太多”, 就调该阶段那行的后三个数(或改 BUCKET_FB_*)。
 * ===================================================================== */
typedef struct {
    uint8_t allow_lr;       /* 1 = 允许修【车体左右】 */
    uint8_t allow_fb;       /* 1 = 允许修【车体前后】 */
    uint8_t x_is_fb;        /* 1 = 画面X 对应车体前后(救援) */
    uint8_t inv_lr;         /* 1 = 左右修正方向取反(桶: 画面镜像) */
    uint8_t fb_scale_pct;   /* 前后步长比例(%): 100=同左右, 50=一半 */
    uint8_t fb_min_mm;      /* 前后单步最小距离(mm) */
    uint8_t fb_max_mm;      /* 前后单步最大距离(mm) */
    uint8_t tol_px;         /* ⭐ 本阶段“判定已对准”的像素容差。
                             * 球/桶 = ALIGN_TOLERANCE(★2026-10-11 起 = 15, 与 K230
                             *         那边抓球/放桶的窗口一致);
                             * 救援 = 25(不跟这次改动 —— 它对应的 K230 任务窗口没变,
                             *        所以这两项在下面显式写 25, 不走宏);
                             * 打靶 = TARGET_ALIGN_TOLERANCE(10) —— K230 那边只把
                             * 打完靶的窗口收紧到了 10px, 本机必须跟它一致, 否则本机
                             * 会提前认为“够准了不再动舵机”而 K230 一直在发 D 帧,
                             * 两边干等到 12s 自超时(只回 OK 不点激光)。 */
    uint8_t fb_closed_loop; /* ⭐⭐ 2026-10-10: 前后轴走【闭环】(而不是固定步长补偿)。
                             * 1 = 用自适应比例步长(自学习 a + 过冲减半) + 3 个兜底
                             *     (方向自动纠正 / 步数与累计位移上限 / 朝桶侧步长限小),
                             *     前后“够准”的容差也放宽成 BUCKET_FB_CL_TOL_PX;
                             * 0 = 老行为(用下面 fb_* 那套固定缩放步长)。
                             * ★ 2026-10-10: 目前【没有阶段用它】= 全 0
                             *   (桶已改回“只修左右 + 臂补 ID2”, 见 BUCKET_FB_CL_ENABLE = 0)。 */
    uint8_t fine_arrival;   /* ⭐⭐ 2026-10-11: 本阶段的修正微步是否用【小到位死区】
                             * (走 Chassis_Move_*Fine() = CH_ALIGN_FINE_THRESHOLD_COUNT
                             *  ≈1.8mm, 而不是默认的 CH_POS_THRESHOLD_COUNT ≈4.5mm)。
                             * 1 = 用(★目前只有桶): 8mm 的微步会真的走掉 ≈6~7mm,
                             *     四轮都出力、车真的平移;
                             * 0 = 老行为(大死区): 小步会被死区吃掉大半, 看起来是
                             *     “个别轮子空转几毫米、车原地蹭”(见 Chassis.h 的说明)。
                             * ⚠️ 底盘总开关 CH_ALIGN_FINE_ENABLE = 0 时本项无效。 */
} AlignAxisCfg_t;

#if ALIGN_AXIS_SCHEME_1
/* ---- 方案一(当前): 排爆/打靶 只修横向; 救援 只修前后 ----
 * ⚠️ 桶【不在这里】: 桶单独由一个开关决定(见下面 BUCKET_USE_SCHEME_2, ★当前 = 1) */
static const AlignAxisCfg_t s_align_ball   = { 1, 0, 0, 0, FB_SCALE_PCT, FB_MIN_MM, FB_MAX_MM,
                                               ALIGN_TOLERANCE, 0, 0 };
#if MISSION_DEBUG_VISION_TASK
/* 靶: ⚠️ 打靶已改版为“只转底座 ID1”, 不再做 D:x,y 精对准。
 * 本项只给 MISSION_DEBUG_VISION_TASK=2 的单独调试用, 平时包在 #if 里,
 * 免得触发 -Wunused-const-variable 警告。
 * ⚠️ 容差用 TARGET_ALIGN_TOLERANCE(20): K230 那边任务2 的窗口就是 20px */
static const AlignAxisCfg_t s_align_target = { 1, 0, 0, 0, FB_SCALE_PCT, FB_MIN_MM, FB_MAX_MM,
                                               TARGET_ALIGN_TOLERANCE, 0, 0 };
#endif
/* ⚠️ 救援【不跟】2026-10-11 那次容差改动(球/桶 25→15): 它对应的是 K230 的另一个
 *    任务窗口(没变), 所以这里显式写 25, 不走 ALIGN_TOLERANCE 宏。 */
static const AlignAxisCfg_t s_align_rescue = { 0, 1, 1, 0, FB_SCALE_PCT, FB_MIN_MM, FB_MAX_MM,
                                               25, 0, 0 };
#else
/* ---- 方案二: 前后左右都修(方向映射仍按上表); 桶的前后步长单独限小 ---- */
static const AlignAxisCfg_t s_align_ball   = { 1, 1, 0, 0,
                                               FB_SCALE_PCT, FB_MIN_MM, FB_MAX_MM,
                                               ALIGN_TOLERANCE, 0, 0 };
#if MISSION_DEBUG_VISION_TASK
static const AlignAxisCfg_t s_align_target = { 1, 1, 0, 0,
                                               FB_SCALE_PCT, FB_MIN_MM, FB_MAX_MM,
                                               TARGET_ALIGN_TOLERANCE, 0, 0 };  /* 仅调试用 */
#endif
/* ⚠️ 救援: 同上面方案一那段说明 —— 显式 25, 不跟本次容差改动 */
static const AlignAxisCfg_t s_align_rescue = { 1, 1, 1, 0,
                                               FB_SCALE_PCT, FB_MIN_MM, FB_MAX_MM,
                                               25, 0, 0 };
#endif

/* ⭐⭐ 2026-10-10: 桶的精对准方案(与全局开关 ALIGN_AXIS_SCHEME_1 无关):
 *   BUCKET_USE_SCHEME_2 = 1: 桶: allow_lr=1 且 allow_fb=1 —— 左右 + 前后【都修】,
 *                          但前后走的是【闭环】(见 fb_closed_loop 与 BUCKET_FB_CL_*)。
 *                       = 0(★当前): 桶【只修横向】—— 与【抓球】完全同款的写法
 *                          (球也是“只修左右”, 深度交给臂补 ID2;
 *                           见 BOMB_FB_ARM_ENABLE_BUCKET)。
 * 字段含义(见 AlignAxisCfg_t): { allow_lr, allow_fb, x_is_fb, inv_lr, fb_*,
 *                                tol_px, fb_closed_loop }
 *   inv_lr = 1: 桶那个姿态(底座 57 ≈ 180°)下画面左右是【镜像】的, 横向方向要取反。
 * ⚠️ 选了闭环(fb_closed_loop = 1)时, 下面那三个 BUCKET_FB_*(固定缩放步长)【不参与
 *    运算】—— 闭环用的是 BUCKET_FB_CL_* 那一套(含朝桶侧限小、步数/位移上限)。
 * ⚠️ 桶的深度修正【只能有一个执行者】: BUCKET_FB_CL_ENABLE(底盘闭环) /
 *    BUCKET_AFTER_ALIGN_BACK_ENABLE(底盘分档后退) / BOMB_FB_ARM_ENABLE_BUCKET(臂补),
 *    三者只能开一个(前两个之间已有 #error 互斥检查)。
 *    ★ 当前执行者 = 【臂补】(BOMB_FB_ARM_ENABLE_BUCKET = 1), 另外两个都是 0。 */
#define BUCKET_USE_SCHEME_2         0

/* ⭐⭐ 2026-10-11 新增(用户要求): 桶【对准微步】用底盘那个"小到位死区" ----------
 *   1(★当前) = 桶的对准微步走 Chassis_Move_*Fine()(死区 ≈1.8mm 而不是 ≈4.5mm):
 *              8mm 的微步会真的走掉 ≈6~7mm, 四个轮子都出力 ⇒ 车真的平移过去,
 *              不会出现"个别轮子空转几毫米、车原地蹭"(详见 Chassis.h 的
 *              CH_ALIGN_FINE_THRESHOLD_COUNT 说明)。
 *   0        = 老行为(用大死区)。
 * ⚠️ 只影响【桶】(球/打靶/救援都不变) —— 球那条路实车已验证可用, 不动它。
 * ⚠️ 底盘那边总开关 CH_ALIGN_FINE_ENABLE = 0 时, 本宏等于无效(自动退回大死区)。 */
#define BUCKET_USE_FINE_ARRIVAL     1

static const AlignAxisCfg_t s_align_bucket = { 1, BUCKET_USE_SCHEME_2, 0, 1,
                                               BUCKET_FB_SCALE_PCT,
                                               BUCKET_FB_MIN_MM,
                                               BUCKET_FB_MAX_MM,
                                               ALIGN_TOLERANCE,
                                               BUCKET_FB_CL_ENABLE,
                                               BUCKET_USE_FINE_ARRIVAL };

/**
 * @brief  像素误差 -> 修正距离(【前后】轴专用)
 * @param  pixel_error 前后方向的像素误差
 * @brief  像素误差 -> 修正距离(【前后】轴专用, 【老做法】: 固定缩放步长)
 * @param  cfg         本阶段配置(取 fb_scale_pct / fb_min_mm / fb_max_mm)
 * @note   ⚠️ 2026-10-10: cfg->fb_closed_loop = 0 的阶段走这里 —— 现在【所有阶段】
 *         都走这里(球/桶/救援/调试都用固定缩放步长); 若哪天把 BUCKET_FB_CL_ENABLE
 *         打开, 桶会改走【闭环】Calc_FbClDistance()(见 AlignAxisCfg_t 字段说明)。
 *         与 Calc_Move_Distance_Adaptive() 的差别只有两点:
 *         ① 先乘 fb_scale_pct%(把前后步长整体缩小, 默认 50%);
 *         ② 再用 fb_min_mm / fb_max_mm 限幅(与左右那套 MIN/MAX_MOVE_MM 独立)。
 *         目的: 左右保持合适的手感, 只把“往前贴”的步长压小, 防空桶/撞桶。
 *         为什么必须用独立限幅: 若直接套 MIN_MOVE_MM, 缩小后的值会被
 *           下限又抬回去, 等于没改。
 */
static int32_t Calculate_Move_Distance_FB(int pixel_error, const AlignAxisCfg_t *cfg)
{
    if (abs(pixel_error) < (int)cfg->tol_px) return 0;          /* 本阶段自己的容差 */
    int32_t dist = (int32_t)(abs(pixel_error) * K_GAIN);
    dist = (dist * (int32_t)cfg->fb_scale_pct) / 100;   /* ① 前后整体缩小 */
    if (dist < (int32_t)cfg->fb_min_mm) dist = (int32_t)cfg->fb_min_mm;
    if (dist > (int32_t)cfg->fb_max_mm) dist = (int32_t)cfg->fb_max_mm;
    return dist;
}

/* =====================================================================
 * ⭐ K_GAIN 标定辅助 (2026-10-06 新增, 专治“不会调 K_GAIN”)
 * ---------------------------------------------------------------------
 * K_GAIN 的含义: 画面里 1 个像素的误差, 让底盘走多少 mm。
 *   理想值算法(和 K_GAIN 宏旁边的注释一致):
 *       实际比例 a(mm/px) = 目标人为挪开的距离 D(mm) / K230 回的初始|误差|(px)
 *       建议 K_GAIN ≈ 0.5 × a   (只取一半, 免得每步都冲过头来回荡)
 * ---------------------------------------------------------------------
 * 日志怎么看(每条 D:x,y 都会打):
 *   [球][K标定] 横向: 误差 -42px -> 左移 11mm, 本次比例 0.262 mm/px(未被限幅(可信))
 *   [球][K标定] 横向: 累计已移动 11mm (正=左移)
 *   [球][K标定] 横向: 误差 -9px  -> 左移 10mm, 本次比例 1.111 mm/px(⚠被MIN_MOVE_MM抬到最小步…)
 *      ↑ 提示“被 MIN/MAX 限幅”的那几步【没按 K_GAIN 走】, 别拿它算比例。
 *   对准结束时打总结:
 *   [球][K标定] 横向总结(K230回OK): 初始误差 -42px -> 结束误差 -3px, 累计移动 33mm
 *   [球][K标定] 横向等效比例 a≈0.786 mm/px -> 建议 K_GAIN≈0.393 (当前 0.25)
 *      ↑ 这行只在“最后确实对上了(残余误差<容差)”时才算得准。
 * ---------------------------------------------------------------------
 * 最省事的标定流程:
 *   ① 拿尺子把目标从“画面正中”【人为挪开】一个已知距离 D(比如 50mm);
 *   ② 进对应阶段对准(或用 MISSION_DEBUG_VISION_TASK 单独调那个任务);
 *   ③ 日志里找 [K标定] 那几行 → 算 a → K_GAIN 填 0.5×a → 重编重试。
 *   ⚠️ 看日志里出现的是“横向”还是“前后”: 两个轴的 mm/px 往往不一样,
 *      但本工程只用一个 K_GAIN, 所以以后/后步长为基准的那套参数(FB_SCALE_PCT/
 *      FB_MIN_MM/FB_MAX_MM)去单独配另一个轴。
 * ===================================================================== */
static int32_t s_kcal_lr_first = 0;   /* 本轮横向对准的第一个误差(px) */
static int32_t s_kcal_lr_last  = 0;   /* 本轮横向对准最近一个误差(px) */
static uint8_t s_kcal_lr_seen  = 0;   /* 0=本轮还没读到过横向误差 */
static int32_t s_kcal_fb_first = 0;   /* 本轮前后对准的第一个误差(px) */
static int32_t s_kcal_fb_last  = 0;   /* 本轮前后对准最近一个误差(px) */
static uint8_t s_kcal_fb_seen  = 0;   /* 0=本轮还没读到过前后误差 */

/* ---- ⭐ “单步实测比例”用的状态(最有用的一组, 原理见下面 Vision_FineAlignProcess 里的注释) ---- */
static int32_t s_kcal_lr_shift_ref = 0;    /* 上次读横向误差帧时的累计位移(正=左移) */
static uint8_t s_kcal_lr_ref_valid = 0;
static int32_t s_kcal_fb_shift_ref = 0;    /* 上次读前后误差帧时的累计位移(正=前进) */
static uint8_t s_kcal_fb_ref_valid = 0;
static float   s_kcal_lr_a_sum = 0.0f;     /* 实测 a 的累加(mm/px) */
static uint8_t s_kcal_lr_a_cnt = 0;        /* 有效的实测 a 次数 */
static float   s_kcal_fb_a_sum = 0.0f;
static uint8_t s_kcal_fb_a_cnt = 0;
static uint8_t s_kcal_lr_flip  = 0;        /* 横向移动方向翻转次数(=来回过冲次数) */
static int8_t  s_kcal_lr_sign  = 0;        /* 上一次横向移动的方向(±1) */

/** @brief 新一轮精对准开始时复位标定统计(由 Vision_StartFineAlign 调用) */
static void KCal_Reset(void)
{
    s_kcal_lr_first = 0;
    s_kcal_lr_last  = 0;
    s_kcal_lr_seen  = 0;
    s_kcal_fb_first = 0;
    s_kcal_fb_last  = 0;
    s_kcal_fb_seen  = 0;
    s_kcal_lr_shift_ref = 0;
    s_kcal_lr_ref_valid = 0;
    s_kcal_fb_shift_ref = 0;
    s_kcal_fb_ref_valid = 0;
    s_kcal_lr_a_sum = 0.0f;
    s_kcal_lr_a_cnt = 0;
    s_kcal_fb_a_sum = 0.0f;
    s_kcal_fb_a_cnt = 0;
    s_kcal_lr_flip  = 0;
    s_kcal_lr_sign  = 0;
}

/**
 * @brief  精对准结束时打印 K_GAIN 标定总结
 * @param  tag    日志标签("BALL"/"BUCKET"/"TARGET"/"SHAPE")
 * @param  reason 结束原因(中文, 方便日志里一眼看到)
 * @note   ⚠️ 只用【单步实测 a】求平均 —— 因为“净位移 / 初始误差”在
 *         来回过冲时算出来是错的(走 57mm 再回 31mm, 净位移只有 26mm,
 *         但那 26mm 并不对应初始误差)。
 */
static void KCal_PrintSummary(const char *tag, const char *reason)
{
    if (!s_kcal_lr_seen && !s_kcal_fb_seen) {
        MLOG("视觉[%s][K标定] 本轮没收到任何 D:x,y 误差帧, 无法标定", tag);
        return;
    }

    if (s_kcal_lr_seen) {
        MLOG("视觉[%s][K标定] 横向总结(%s): 初始误差 %ldpx -> 结束误差 %ldpx, 净位移 %ldmm(正=左移), 方向翻转 %u 次",
             tag, reason, (long)s_kcal_lr_first, (long)s_kcal_lr_last,
             (long)s_align_shift_strafe, (unsigned)s_kcal_lr_flip);

        if (s_kcal_lr_a_cnt > 0) {
            float a_avg = s_kcal_lr_a_sum / (float)s_kcal_lr_a_cnt;
            MLOG("视觉[%s][K标定] 横向实测 a 平均 ≈ %.3f mm/px (由 %u 步单步数据得出) -> 建议 K_GAIN ≈ %.3f (当前 %.2f)",
                 tag, (double)a_avg, (unsigned)s_kcal_lr_a_cnt,
                 (double)(a_avg * 0.5f), (double)K_GAIN);
        } else {
            MLOG("视觉[%s][K标定] 横向拿不到有效单步数据(可能每步都被 MIN/MAX 限幅, "
                 "或方向反了一直往外跑); 请把目标人为挪开已知距离 D(mm) 后手算 a=D/|初始误差|",
                 tag);
        }
        if (s_kcal_lr_flip > 0) {
            MLOG("视觉[%s][K标定] ⚠ 横向来回摆了 %u 次 => 当前 K_GAIN=%.2f 偏大(每步都过冲); "
                 "建议调到上面“实测 a”的一半左右",
                 tag, (unsigned)s_kcal_lr_flip, (double)K_GAIN);
        }
    }

    if (s_kcal_fb_seen) {
        MLOG("视觉[%s][K标定] 前后总结(%s): 初始误差 %ldpx -> 结束误差 %ldpx, 净位移 %ldmm(正=前进)",
             tag, reason, (long)s_kcal_fb_first, (long)s_kcal_fb_last,
             (long)s_align_shift_fwd);
        if (s_kcal_fb_a_cnt > 0) {
            float a_avg = s_kcal_fb_a_sum / (float)s_kcal_fb_a_cnt;
            MLOG("视觉[%s][K标定] 前后实测 a 平均 ≈ %.3f mm/px (由 %u 步得出; "
                 "注意步长已乘过 FB_SCALE_PCT=%d%% 缩放)",
                 tag, (double)a_avg, (unsigned)s_kcal_fb_a_cnt, (int)FB_SCALE_PCT);
        } else {
            MLOG("视觉[%s][K标定] 前后没拿到有效单步数据", tag);
        }
    }
}

/* =====================================================================
 * ⭐⭐ 视觉“稳定帧”滤波 (2026-10-07 新增, 用于排爆 / 救援)
 * ---------------------------------------------------------------------
 * 【为什么需要】K230 是连续输出帧的, 单帧可能是误检/抖动(目标一闪、画面糊、
 *   识别框跳一下)。旧代码是【收到第一行就动手】:
 *     · 排爆: 一帧假的 L 就让小车白横移 BOMB_L_ADJUST_MM;
 *     · 救援巡视: 任何一行就中止巡视;
 *     · 救援对准: 一帧假的 C 就直接去抓人质(最危险)。
 * 【怎么做】要求【连续 N 帧是同一个 token】才采纳:
 *     token = C / L / R 本身;  D:<x>,<y> 统一记成 'D';  其它行记成首字符。
 *   ⇒ 3 帧一致 = 单帧误检基本不可能;
 *     而 D 帧的数值每帧都在变, 所以只比“是不是 D” ⇒ “连续 N 帧都还在报误差”
 *     = 目标确实在画面里、K230 在持续跟踪。
 * 【防卡死】等不到连续一致时, 最长等 VISION_STABLE_TIMEOUT_MS 就用【最新一帧】
 *   放行(并打日志说明), 绝不会因为凑不齐而卡住。
 *   ⚠️ 巡视那一处用 timeout = 0(不超时): 凑不齐就说明“没真看到”, 扫完一圈收工。
 * 【调参】帧数越多越稳但越慢(K230 约 2Hz ⇒ 3 帧 ≈ 1.5s):
 *   排爆只有一两次动作, 用 3; 救援要对准很多步, 用 2 免得把对准拖到超时。
 *   不想用就把它置 1(等于回到“收到就动手”)。
 * ===================================================================== */
#define VISION_STABLE_FRAMES_BOMB    3     /* 排爆(球/桶): 连续 3 帧同向才动车 */
#define VISION_STABLE_FRAMES_RESCUE  2     /* 救援对准: 连续 2 帧同向才转 ID1 */
#define VISION_STABLE_FRAMES_SWEEP   2     /* 救援巡视: 连续 2 帧才算“真看到目标” */
#define VISION_STABLE_TIMEOUT_MS     3000  /* 等不到连续一致时的兜底(ms), 0 = 不兜底 */

typedef struct {
    uint8_t  inited;   /* 0 = 本轮还没收到过 */
    uint8_t  cnt;      /* 连续相同帧数 */
    uint8_t  chg;      /* 本轮 token 变化次数(只给日志看“是不是在跳”) */
    char     tok;      /* 最近的 token(= 最新一帧) */
    uint32_t t0;       /* 本轮起始时刻 */
} VisionStable_t;

/* 三个阶段各一个滤波器实例(排爆两个点各一个: 球 / 桶) */
static VisionStable_t s_vs_ball;
static VisionStable_t s_vs_bucket;
static VisionStable_t s_vs_rescue;

/**
 * @brief  清空一个“稳定帧”滤波器(换任务 / 开新一轮时调)
 * @note   它也是采纳后的复位动作, 所以“每动一步”都要重新凑够 need 帧一致。
 */
static void Vision_StableReset(VisionStable_t *st)
{
    st->inited = 0;
    st->cnt    = 0;
    st->chg    = 0;
    st->tok    = ' ';
    st->t0     = 0;
}

#if !MISSION_TEST_NO_VISION
/** @brief 把一行 K230 消息归类成“稳定帧比较用的 token” */
static char Vision_TokenOf(const char *line)
{
    if (strstr(line, "D:") != NULL) {
        return 'D';      /* D:<x>,<y> 数值每帧都变, 只比“是不是 D” */
    }
    return line[0];
}

/**
 * @brief  喂一帧给“稳定帧”滤波器, 判断能不能采纳
 * @param  st         滤波器实例
 * @param  tok        本帧的 token(见 Vision_TokenOf)
 * @param  need       需要连续几帧相同
 * @param  timeout_ms 兜底时间(ms): 期间凑不齐就用最新一帧放行; 0 = 不兜底(一直等)
 * @param  tag        日志前缀("球"/"桶"/"救援"/"巡视")
 * @retval 1 = 可以按这一帧动手(已打日志); 0 = 还没稳, 调用方应继续等下一帧
 * @note   采纳后自动复位, 因此每动一步都要重新凑够 need 帧一致。
 */
static uint8_t Vision_StableFeed(VisionStable_t *st, char tok, uint8_t need,
                                 uint32_t timeout_ms, const char *tag)
{
    uint32_t now = HAL_GetTick();

    if (!st->inited) {
        st->inited = 1;
        st->tok    = tok;
        st->cnt    = 1;
        st->chg    = 0;
        st->t0     = now;
    } else if (tok == st->tok) {
        if (st->cnt < 255u) {
            st->cnt++;
        }
    } else {
        st->tok = tok;
        st->cnt = 1;                 /* 变了就重新数(但 t0 不回退, 超时还是按本轮算) */
        if (st->chg < 255u) {
            st->chg++;
        }
    }

    if (need <= 1u || st->cnt >= need) {
        MLOG("视觉[%s][稳定帧] 连续 %u 帧都是 '%c' -> 采纳",
             tag, (unsigned)st->cnt, st->tok);
        Vision_StableReset(st);
        return 1;
    }
    if (timeout_ms != 0u && (now - st->t0) >= timeout_ms) {
        MLOG("视觉[%s][稳定帧] ⚠ %ums 内没凑够 %u 帧一致(本轮变了 %u 次), "
             "按最新帧 '%c' 放行(防卡死)",
             tag, (unsigned)timeout_ms, (unsigned)need, (unsigned)st->chg, st->tok);
        Vision_StableReset(st);
        return 1;
    }

    MLOG("视觉[%s][稳定帧] '%c' 连续 %u/%u 帧, 继续等(本帧不动手)",
         tag, st->tok, (unsigned)st->cnt, (unsigned)need);
    return 0;
}
#endif /* !MISSION_TEST_NO_VISION */

/**
 * @brief  发送 run_task(带 1s 定时重发, 应对 K230 重启加载模型丢指令)
 * @param  task         K230 任务号(1=球 2=靶 3=桶 4=形状)
 * @param  tag          日志标签(如 "BALL")
 * @param  p_last_send  指向"上次发送时刻"变量(各状态机持有)
 */
static void Vision_SendTask(uint8_t task, const char *tag, uint32_t *p_last_send)
{
    if (!g_vision_task_in_progress) {
        Chassis_Stop();
        /* ⭐ 换任务: 先清掉旧数据, 再开静默期(见 VISION_MODE_SETTLE_MS 注释) */
        K230_FlushAll();
        Vision_ModeSwitchStart();
        /* ⭐ 换任务: “稳定帧”计数也清零(免得拿上个任务的连续计数直接放行) */
        Vision_StableReset(&s_vs_ball);
        Vision_StableReset(&s_vs_bucket);
        Vision_StableReset(&s_vs_rescue);
        MLOG("视觉[%s]: 启动任务 run_task:%d (已清旧帧 + 开换任务静默期)", tag, (int)task);
    }
    K230_Run_Specific_Task(task);
    g_vision_task_in_progress = 1;
    *p_last_send = HAL_GetTick();
}

/**
 * @brief  进入精对准: 发 start_align 并复位计时/冷却
 */
static void Vision_StartFineAlign(const char *tag, const AlignAxisCfg_t *cfg,
                                  uint32_t *p_state_tick,
                                  uint32_t *p_align_tick, uint32_t *p_cooldown)
{
    MLOG("视觉[%s]: 请求精对准(发 start_align)", tag);
    K230_Start_Align();
    K230_FlushAll();
    *p_state_tick = HAL_GetTick();
    *p_align_tick = HAL_GetTick();
    *p_cooldown = 0;
    s_align_shift_strafe = 0;   /* 新一次对准: 清累计偏移 */
    s_align_shift_fwd    = 0;
    KCal_Reset();               /* 新一次对准: 清 K_GAIN 标定统计 */
    AlignDwell_Reset();         /* 新一次对准: 清“确认帧计数 / 最近残差” */
    FbCl_Reset();               /* ⭐ 新一次对准: 清“前后闭环”的本轮统计(极性/翻转标志保留) */
    Chassis_Stop();
    /* 把当前实际生效的参数打出来, 方便对照日志调参 */
    MLOG("视觉[%s][K标定] 本轮参数: 自适应步长(下限%.0fmm > 死区≈%.1fmm, 上限%.0fmm, "
         "打折%d%%, 自学习 a 样本 %u 个), K_GAIN=%.2f mm/px, 容差=%dpx",
         tag, (double)ALIGN_STEP_MIN_MM, (double)CH_DEADZONE_MM,
         (double)ALIGN_STEP_MAX_MM, (int)ALIGN_STEP_DAMP_PCT,
         (unsigned)s_adapt_n, (double)K_GAIN, (int)cfg->tol_px);
}

/**
 * @brief  处理一帧精对准消息("OK" / "D:x,y")
 * @param  line        K230 回传的一行
 * @param  tag         日志标签("BALL"/"BUCKET"/"TARGET"/"SHAPE")
 * @param  p_cooldown  冷却截止时刻(冷却中忽略新帧, 避免指令叠车)
 * @param  p_settle    对准完成后要等到的时刻(停车稳定)
 * @param  cfg         本阶段的轴向/方向配置(见 AlignAxisCfg_t)
 * @retval 1=已对准(OK 或 Close Enough, 已设好 settle 时刻); 0=继续对准
 * @note   修正流程(四步):
 *         ① 把 K230 的【画面误差】(err_x, err_y) 按 cfg 翻译成【车体误差】
 *            (lr_px 左右 / fb_px 前后);
 *         ② 只修 cfg 允许修的轴(方案一限制; 方案二两个轴都修);
 *         ③ 串行修正: 一个轴修完(冷却结束)再修另一个轴, 绝不同时修;
 *         ④ 每步位移累加到 s_align_shift_*, 对准结束时打日志,
 *            方便判断要不要在后续路线宏里补回来。
 *         注意: 左右步长由 Calc_Move_Distance_Adaptive() 算(自适应: 按误差分档
 *               + 自学习实测比例 a + 下限 > 底盘死区 + 上步冲过头就减半,
 *               详见 ALIGN_STEP_* 那段注释);
 *               前后步长由 Calculate_Move_Distance_FB() 算(多乘 fb_scale_pct%,
 *               再用 fb_min_mm/fb_max_mm 限幅 —— 前后离目标近, 单独限小);
 *               不修的轴的误差【不参与】“是否已对准”的判定, 否则会永远卡住。
 */
static uint8_t Vision_FineAlignProcess(char *line, const char *tag,
                                       uint32_t *p_cooldown, uint32_t *p_settle,
                                       const AlignAxisCfg_t *cfg)
{
    if (HAL_GetTick() < *p_cooldown) {
        return 0;   /* 冷却中, 忽略新帧 */
    }
    /* ⭐ 新 K230(yolo_main.py)在【任务2 打靶】对准成功时会先发一行 "FIRE"
     *    (通知“激光已发射”), 紧跟一行 "OK"。这里只记录, 不改变对准流程 */
    if (strncmp(line, "FIRE", 4) == 0) {
        MLOG("视觉[%s]: K230 已发射激光(FIRE) —— 它认为已经对准靶心", tag);
        return 0;
    }
    if (strncmp(line, "OK", 2) == 0) {
        /* ⭐ 2026-10-08: 把“K230 判 OK 那一刻本机看到的残差”打出来 ——
         *    若这里的 px 明显大于本机容差, 说明 K230 那边的窗口更宽、它提前判
         *    成功 ⇒ 本机拿不到纠正机会就直接放下了。要把它(K230 的
         *    ALIGN_TOL_PIX)收到和本机 ALIGN_TOLERANCE 一致。 */
        if (s_align_last_valid) {
            MLOG("视觉[%s]: ⭐ K230 判 OK 时本机最近一帧残差 左右=%ldpx 前后=%ldpx "
                 "(本机容差 %dpx; 差得多就去收紧 K230 那边的窗口)",
                 tag, (long)s_align_last_lr, (long)s_align_last_fb, (int)cfg->tol_px);
        }
        MLOG("视觉[%s]: 已对准(收到 OK) -> 停车稳定 %dms", tag, (int)FINE_TUNE_SETTLE_MS);
        KCal_PrintSummary(tag, "K230回OK");
        Chassis_Stop();
        *p_settle = HAL_GetTick() + FINE_TUNE_SETTLE_MS;
        return 1;
    }
    char *pD = strstr(line, "D:");
    if (pD != NULL) {
        char *pComma = strchr(pD, ',');
        if (pComma) {
            *pComma = '\0';
            int err_x = atoi(pD + 2);
            int err_y = atoi(pComma + 1);
            int lr_px, fb_px;

            /* ① 画面误差 → 车体误差(cfg 为小车轴向配置) */
            if (cfg->x_is_fb) {
                lr_px = err_y;   /* 救援: 画面Y → 车体左右 */
                fb_px = err_x;   /* 救援: 画面X → 车体前后 */
            } else {
                lr_px = err_x;   /* 球/靶/桶: 画面X → 车体左右 */
                fb_px = err_y;
            }
            if (cfg->inv_lr) {
                lr_px = -lr_px;  /* 桶: 底座转≈180°, 画面镜像 → 修正方向取反 */
            }

            /* ⭐ 记下“最近一帧的残差”(换算到车体之后的值): 结束对准时打出来,
             *    便于判断这次到底是“真对准了”还是“没对准就直接执行了” */
            s_align_last_lr    = lr_px;
            s_align_last_fb    = fb_px;
            s_align_last_valid = 1;

            /* ⭐ K_GAIN 标定①: 用“上一步实际走的位移”和“误差的变化量”直接反推真实比例:
             *        a(mm/px) = |上一步位移| / |误差变化量|
             *    这是最可信的一条 —— 因为 K230 是“移动一次→看一眼→发一个 D 帧”,
             *    相邻两帧的误差差就是那一步的效果。
             *    ⚠️ 两者必须【同号】(位移把误差修小了) 才算得对;
             *       异号 = 那一步把误差弄更大了(方向设反), 会把 a 算成负数, 所以单独提示。
             *    ⚠️ 被 MIN/MAX 限幅的那一步也能用!(限幅后我们仍知道实际走了多少 mm) */
            if (cfg->allow_lr) {
                if (s_kcal_lr_ref_valid) {
                    int32_t moved = s_align_shift_strafe - s_kcal_lr_shift_ref;  /* 上一步净位移 */
                    int32_t de    = s_kcal_lr_last - lr_px;                      /* 误差变化量 */
                    if (moved != 0 && de != 0 && ((moved > 0) == (de > 0))) {
                        float a = (float)abs(moved) / (float)abs(de);
                        s_kcal_lr_a_sum += a;
                        s_kcal_lr_a_cnt++;
                        MLOG("视觉[%s][K标定] 上一步走 %ldmm 使横向误差变化 %ldpx "
                             "=> 本次实测 a≈%.3f mm/px",
                             tag, (long)moved, (long)de, (double)a);
                        /* ⭐ 自适应步长: 把这个实测比例并进“自学习”估计(指数平滑)。
                         *    只收合理范围内的值, 防止偶发脏数据把步长带飞。 */
                        if (a >= ALIGN_ADAPT_A_MIN && a <= ALIGN_ADAPT_A_MAX) {
                            if (s_adapt_n == 0) {
                                s_adapt_a = a;
                            } else {
                                s_adapt_a = s_adapt_a * 0.7f + a * 0.3f;
                            }
                            if (s_adapt_n < 200u) {
                                s_adapt_n++;
                            }
                        } else {
                            MLOG("视觉[%s][自适应] ⚠ 实测比例 a=%.3f mm/px 超出合理范围"
                                 "(%.2f~%.2f), 已丢弃(不参与自学习)",
                                 tag, (double)a, (double)ALIGN_ADAPT_A_MIN,
                                 (double)ALIGN_ADAPT_A_MAX);
                        }
                    } else if (moved != 0 && de != 0) {
                        MLOG("视觉[%s][K标定] ⚠ 上一步走 %ldmm 但误差反而变成 %ldpx(变化 %ldpx) "
                             "=> 方向可能设反了, 请检查方向映射/inv_lr",
                             tag, (long)moved, (long)lr_px, (long)de);
                    }
                }
                s_kcal_lr_shift_ref = s_align_shift_strafe;
                s_kcal_lr_ref_valid = 1;
                if (!s_kcal_lr_seen) { s_kcal_lr_seen = 1; s_kcal_lr_first = lr_px; }
                s_kcal_lr_last = lr_px;
            }
            if (cfg->allow_fb) {
                if (s_kcal_fb_ref_valid) {
                    int32_t moved = s_align_shift_fwd - s_kcal_fb_shift_ref;
                    int32_t de    = s_kcal_fb_last - fb_px;
                    if (moved != 0 && de != 0 && ((moved > 0) == (de > 0))) {
                        float a = (float)abs(moved) / (float)abs(de);
                        s_kcal_fb_a_sum += a;
                        s_kcal_fb_a_cnt++;
                        MLOG("视觉[%s][K标定] 上一步走 %ldmm 使前后误差变化 %ldpx "
                             "=> 本次实测 a≈%.3f mm/px",
                             tag, (long)moved, (long)de, (double)a);
                    }
                }
                s_kcal_fb_shift_ref = s_align_shift_fwd;
                s_kcal_fb_ref_valid = 1;
                if (!s_kcal_fb_seen) { s_kcal_fb_seen = 1; s_kcal_fb_first = fb_px; }
                s_kcal_fb_last = fb_px;
            }

            MLOG("视觉[%s][K标定] K230回传画面误差 x=%d y=%d | 换算到车体: 左右=%d 前后=%d px",
                 tag, err_x, err_y, lr_px, fb_px);

            /* ⭐⭐ 前后轴的“门限 / 还有没有预算”一次算好(闭环模式与老模式取值不同):
             *   · 闭环(★当前关闭, 见 BUCKET_FB_CL_ENABLE): 前后“够准”的容差会放宽成
             *     BUCKET_FB_CL_TOL_PX(深度没那么关键);
             *   · 兜底用尽(步数/位移到上限)时, 前后也当“够准” —— 否则会一直等到超时。 */
            int32_t fb_tol_used  = cfg->fb_closed_loop ? (int32_t)BUCKET_FB_CL_TOL_PX
                                                       : (int32_t)cfg->tol_px;
            uint8_t fb_budget_ok = (uint8_t)((!cfg->fb_closed_loop) || FbCl_BudgetLeft());

            /* 是否已对准: 只看【允许修的那些轴】(容差取本阶段自己的 tol_px) */
            if ((!cfg->allow_lr || abs(lr_px) < (int)cfg->tol_px) &&
                (!cfg->allow_fb || abs(fb_px) < fb_tol_used || !fb_budget_ok)) {
                /* ⭐ 2026-10-08: 连续 ALIGN_DONE_FRAMES 帧都进容差才算对准。
                 *    本帧不动作(车已经在容差内), 等下一帧再确认一次 ——
                 *    这样“单帧误检 / 目标在窗口边缘抖一下”不会让车还没摆正就放球。 */
                s_align_done_cnt++;
                if (s_align_done_cnt < (uint8_t)ALIGN_DONE_FRAMES) {
                    MLOG("视觉[%s]: 已进容差(左右=%ldpx 前后=%ldpx), 连续确认 %u/%u 帧…",
                         tag, (long)lr_px, (long)fb_px,
                         (unsigned)s_align_done_cnt, (unsigned)ALIGN_DONE_FRAMES);
                    return 0;
                }
                MLOG("视觉[%s]: 已对准(连续 %u 帧都在容差 %dpx 内; 末帧左右=%ldpx 前后=%ldpx)"
                     " -> 停车稳定 %dms",
                     tag, (unsigned)s_align_done_cnt, (int)cfg->tol_px,
                     (long)lr_px, (long)fb_px, (int)FINE_TUNE_SETTLE_MS);
                KCal_PrintSummary(tag, "误差进容差");
                Chassis_Stop();
                *p_settle = HAL_GetTick() + FINE_TUNE_SETTLE_MS;
                return 1;
            }
            /* 本帧没进容差 ⇒ 下面要发一步修正, “连续确认”重新计数 */
            s_align_done_cnt = 0;

            /* ③ 串行修正: 先左右, 后前后 */
            if (cfg->allow_lr && abs(lr_px) >= (int)cfg->tol_px) {
                int32_t d;
                float   a_used = (float)K_GAIN;

                /* ⭐ 先判断“上一步是不是冲过头了”(本帧误差方向 与 上一步移动方向相反):
                 *    是 → 这一刻的自适应步长减半, 抑制在窗口两边来回摆。
                 *    ⚠️ 顺序很重要: 必须在算步长【之前】判, 否则减半永远慢一拍。 */
                {
                    int8_t sgn_now = (lr_px > 0) ? 1 : -1;

                    if (s_kcal_lr_sign != 0 && sgn_now != s_kcal_lr_sign) {
                        s_adapt_over = 1;
                        s_kcal_lr_flip++;      /* 也用于 K_GAIN 总结里提醒“偏大” */
                    } else {
                        s_adapt_over = 0;
                    }
                    s_kcal_lr_sign = sgn_now;
                }

                /* ⭐⭐ 2026-10-11: 步长下限【按阶段取】——
                 *    细模式(桶, cfg->fine_arrival=1) 用 ALIGN_STEP_MIN_MM_FINE(8mm):
                 *      它走"小到位死区"入口, 8mm 能被完整执行掉(不被死区吃掉);
                 *      ⚠️ 别再调小(3~5mm 会让个别轮子动、车整体不平移 = 车身偏移);
                 *    普通阶段(球/打靶/救援) 用 ALIGN_STEP_MIN_MM(8mm) —— 两者当前相同。 */
                {
                    int32_t min_step = cfg->fine_arrival ? (int32_t)ALIGN_STEP_MIN_MM_FINE
                                                         : (int32_t)ALIGN_STEP_MIN_MM;

                    d = Calc_Move_Distance_Adaptive(lr_px, (int)cfg->tol_px, min_step, &a_used);
                    if (d > 0) {
                        /* ⭐⭐ 2026-10-11(用户要求): 【桶】的横向修正分两档 ——
                         *   需要的位移不够"底盘最小步"(BUCKET_LR_CHASSIS_MIN_MM)时
                         *   底盘【不动】(那种小步会被到位死区吃掉 ⇒ 个别轮子空转、车原地蹭),
                         *   改成让【底座 ID1】转一点(限幅 0~40 码, 见 BUCKET_LR_ID1_*);
                         *   大误差照旧走底盘平移, 而且步长 ≥ 最小步 ⇒ 四轮一起走、真平移。
                         *   ID1 顶到边界 / 该功能关掉时 Bucket_LrFineWithId1 返回 0
                         *   ⇒ 自动回退走下面那套底盘平移, 保证一定能收敛。 */
                        uint8_t handled = 0;

#if BUCKET_LR_ID1_ENABLE
                        if (cfg->fine_arrival &&
                            d < (int32_t)BUCKET_LR_CHASSIS_MIN_MM) {
                            handled = Bucket_LrFineWithId1(lr_px, tag, p_cooldown);
                        }
#endif
                        if (!handled) {
                        /* ⭐⭐ 2026-10-11: cfg->fine_arrival = 1 的阶段(目前 = 桶)走"小死区"入口,
                         *    让这些小步真的走掉(见 Chassis.h 的 CH_ALIGN_FINE_* 说明)。 */
                        if (lr_px < 0) {
                            if (cfg->fine_arrival) Chassis_Move_RightFine(d);
                            else                   Chassis_Move_Right(d);
                            s_align_shift_strafe -= d;
                        } else {
                            if (cfg->fine_arrival) Chassis_Move_LeftFine(d);
                            else                   Chassis_Move_Left(d);
                            s_align_shift_strafe += d;
                        }

                        MLOG("视觉[%s][自适应] 横向: 误差 %ldpx -> %s %ldmm "
                             "(a≈%.3f mm/px %s, 下限%ldmm > 死区%.1fmm%s)",
                             tag, (long)lr_px, (lr_px > 0) ? "左移" : "右移", (long)d,
                             (double)a_used,
                             (s_adapt_n >= ALIGN_ADAPT_MIN_SAMPLES) ? "实测自学习" : "K_GAIN估算",
                             (long)min_step, (double)CH_DEADZONE_MM,
                             s_adapt_over ? ", 上步冲过头已减半" : "");
                        MLOG("视觉[%s][K标定] 横向: 累计已移动 %ldmm (正=左移)",
                             tag, (long)s_align_shift_strafe);
                        *p_cooldown = HAL_GetTick() + FINE_TUNE_COOLDOWN_MS;
                        }
                    }
                }
            } else if (cfg->allow_fb && fb_budget_ok && abs(fb_px) >= fb_tol_used) {
                if (cfg->fb_closed_loop) {
                    /* ============ ⭐ 桶: 前后【闭环】修正 + 3 个自动兜底 ============ */
                    /* 兜底① 方向自动纠正: 上一步动完 |误差| 反而变大 ⇒ 极性反了 */
                    if (BUCKET_FB_CL_FLIP_ENABLE && s_fb_cl_moved &&
                        (abs(fb_px) > (s_fb_cl_last_px + (int32_t)BUCKET_FB_CL_FLIP_MIN_PX))) {
                        if (!s_fb_cl_flipped) {
                            s_fb_cl_sign    = (int8_t)(-s_fb_cl_sign);
                            s_fb_cl_flipped = 1;
                            s_fb_cl_over    = 0;
                            s_fb_cl_err_sign = 0;   /* 换了极性, 过冲判据重新开始 */
                            MLOG("视觉[%s][前后闭环]: ⚠ 上一步动完 |前后误差| 从 %ldpx 变大到 %dpx "
                                 "⇒ 极性反了, 已【自动翻转】(只翻一次; 累计 %+ldmm)",
                                 tag, (long)s_fb_cl_last_px, abs(fb_px), (long)s_fb_cl_total);
                        } else {
                            s_fb_cl_giveup = 1;
                            MLOG("视觉[%s][前后闭环]: ⚠ 翻转后 |前后误差| 仍在变大(%ldpx -> %dpx) "
                                 "⇒ 本轮【放弃前后修正】, 直接按现状去放球(不再折腾)",
                                 tag, (long)s_fb_cl_last_px, abs(fb_px));
                        }
                    }
                    if (!s_fb_cl_giveup) {
                        /* 过冲检测(与极性无关): 本帧误差符号 与 上一步动作依据的符号相反
                         * ⇒ 上一步跨过了中心 ⇒ 这一步减半(阻尼) */
                        int8_t  sgn_now = (fb_px > 0) ? 1 : -1;
                        int32_t d, dir, room;

                        s_fb_cl_over = (uint8_t)((s_fb_cl_err_sign != 0) && (sgn_now != s_fb_cl_err_sign));

                        d   = Calc_FbClDistance(fb_px);
                        dir = ((fb_px > 0) ? 1 : -1) * (int32_t)s_fb_cl_sign;   /* +1 = 车头前进 */

                        /* 兜底②: 先按“累计位移上限”把这一步夹一下(两边都算上) */
                        room = (int32_t)BUCKET_FB_CL_MAX_TOTAL_MM;
                        if (dir > 0) {
                            room -= s_fb_cl_total;
                        } else {
                            room += s_fb_cl_total;
                        }
                        if (room < 0) room = 0;
                        if (d > room) d = room;

                        if (d < (int32_t)BUCKET_FB_CL_MIN_MM) {
                            /* 剩余余量已不足一个最小步(走了也会被死区吃掉) ⇒ 停手 */
                            s_fb_cl_giveup = 1;
                            MLOG("视觉[%s][前后闭环]: 累计位移已到上限(%+ldmm / 上限 %dmm), "
                                 "停止前后修正, 按现状去放球", tag, (long)s_fb_cl_total,
                                 (int)BUCKET_FB_CL_MAX_TOTAL_MM);
                        } else {
                            if (dir > 0) { Chassis_Move_Forward(d);  s_align_shift_fwd += d; }
                            else         { Chassis_Move_Backward(d); s_align_shift_fwd -= d; }

                            s_fb_cl_total   += (dir > 0) ? d : -d;
                            s_fb_cl_steps++;
                            s_fb_cl_moved    = 1;
                            s_fb_cl_last_px  = abs(fb_px);
                            s_fb_cl_err_sign = sgn_now;

                            MLOG("视觉[%s][前后闭环] 第%u/%u 步: 前后 %ldpx -> %s %ldmm "
                                 "(%s; 单步上限 朝桶%d/离开%d mm; 累计 %+ldmm)",
                                 tag, (unsigned)s_fb_cl_steps, (unsigned)BUCKET_FB_CL_MAX_STEPS,
                                 (long)fb_px, (dir > 0) ? "前进" : "后退", (long)d,
                                 s_fb_cl_over ? "上步跨过中心已减半" : "正常",
                                 (int)BUCKET_FB_CL_FWD_MAX_MM, (int)BUCKET_FB_CL_BACK_MAX_MM,
                                 (long)s_fb_cl_total);
                            *p_cooldown = HAL_GetTick() + FINE_TUNE_COOLDOWN_MS;
                        }
                    }
                } else {
                    int32_t d = Calculate_Move_Distance_FB(fb_px, cfg);   /* 前后用独立的小步长 */
                    if (d > 0) {
                        /* ⭐⭐ 2026-10-11: 同横向 —— cfg->fine_arrival 的阶段走"小死区"入口 */
                        if (fb_px < 0) {
                            if (cfg->fine_arrival) Chassis_Move_BackwardFine(d);
                            else                   Chassis_Move_Backward(d);
                            s_align_shift_fwd -= d;
                        } else {
                            if (cfg->fine_arrival) Chassis_Move_ForwardFine(d);
                            else                   Chassis_Move_Forward(d);
                            s_align_shift_fwd += d;
                        }

                        MLOG("视觉[%s][K标定] 前后: 误差 %ldpx -> %s %ldmm (已按比例 %d%% 缩小)",
                             tag, (long)fb_px, (fb_px > 0) ? "前进" : "后退", (long)d,
                             (int)cfg->fb_scale_pct);
                        MLOG("视觉[%s][K标定] 前后: 累计已移动 %ldmm (正=前进)",
                             tag, (long)s_align_shift_fwd);
                        *p_cooldown = HAL_GetTick() + FINE_TUNE_COOLDOWN_MS;
                    }
                }
            }
        }
    }
    return 0;
}

/**
 * @brief  精对准超时检查
 * @retval 1=超时/超过强制时限(放弃对准); 0=继续
 */
static uint8_t Vision_FineAlignTimeout(const char *tag, uint32_t state_tick, uint32_t align_tick)
{
    if (HAL_GetTick() - state_tick > ALIGN_TIMEOUT_MS) {
        MLOG("视觉[%s]: 精对准超时(%dms), 放弃对准直接执行; ⚠ 放弃时残差 %s"
             "左右=%ldpx 前后=%ldpx(容差 %dpx) —— 这里明显超容差就是“没对准也执行了”; "
             "距上次收到K230数据 %lums",
             tag, (int)ALIGN_TIMEOUT_MS, s_align_last_valid ? "" : "(本轮没读到 D 帧) ",
             (long)s_align_last_lr, (long)s_align_last_fb, (int)ALIGN_TOLERANCE,
             (unsigned long)K230_RxSilenceMs());
        KCal_PrintSummary(tag, "超时放弃");
        Chassis_Stop();
        return 1;
    }
    if (HAL_GetTick() - align_tick > FORCE_GRAB_AFTER_MS && g_k230_new_data_flag) {
        MLOG("视觉[%s]: 超过强制时限(%dms)仍未对准, 放弃对准直接执行; ⚠ 放弃时残差 %s"
             "左右=%ldpx 前后=%ldpx(容差 %dpx)",
             tag, (int)FORCE_GRAB_AFTER_MS, s_align_last_valid ? "" : "(本轮没读到 D 帧) ",
             (long)s_align_last_lr, (long)s_align_last_fb, (int)ALIGN_TOLERANCE);
        KCal_PrintSummary(tag, "强制时限放弃");
        Chassis_Stop();
        return 1;
    }
    return 0;
}

#if !MISSION_TEST_NO_VISION
/**
 * @brief  判断一行是不是“接近阶段的纯方向指令” C / L / R, 是则输出方向
 * @param  line K230 回传的一行
 * @param  out  输出方向字符('C'/'L'/'R')
 * @retval 1=是方向指令; 0=不是(调用方应忽略这一行, 继续等下一行)
 * @note   ⭐ 为什么要严格判断:
 *         K230 在【接近阶段】只发 C/L/R, 但队列里可能混进 "OK" / "FIRE" /
 *         "D:<x>,<y>" / "SCAN_OK" 之类的行。旧代码直接取 line[0] 判断, 于是
 *         "FIRE" 的 'F' 会被当成“既不是 L 也不是 R” → 在打靶/救援分支里被
 *         误判成“已经收到 C → 已对准”, 车/臂还没对好就跑去做抓取/发射。
 *         配上 run_task 重发 + 各阶段的超时兜底后, 忽略无关行更安全且不会卡死。
 */
static uint8_t Vision_IsDirLine(const char *line, char *out)
{
    uint16_t i = 1;
    char c = line[0];

    if (c != 'C' && c != 'L' && c != 'R') {
        return 0;
    }
    while (line[i] == ' ' || line[i] == '\t') {
        i++;
    }
    if (line[i] != '\0') {
        return 0;   /* 后面还有别的内容(如 "CMD:..."), 不是纯方向指令 */
    }
    *out = c;
    return 1;
}
#endif /* !MISSION_TEST_NO_VISION */

#if !MISSION_TEST_NO_VISION
/* =====================================================================
 * ⭐ 原地只转底座 ID1 (打靶 / 救援 共用; 2026-10-04 新增, 10-05 泛化)
 * ---------------------------------------------------------------------
 * 为什么不用 Arm_GotoPose / 直接改姿态表:
 *   ① Arm_GotoPose 是“整张姿态表一起写”, 会把 ID2~ID5 也重写一遍;
 *   ② 它会阻塞(运动时间 + 保持时间) ≈ 3~4.5s, 期间主循环收不到 C/L/R,
 *      不适合“转一点 → 看一眼画面 → 再转一点”的逐个修正。
 * 所以这里只用 Servos_SetPositionsMasked(pose, SERVO_MASK_ID1, ...) 下发 ID1:
 *   位置数组里其余 4 个舵机仍填【基准姿态】那一行(因为没选中, 不会动),
 *   只有 [0]=ID1 被改成新值。下发后立即返回, 由调用方用冷却时间等它停稳。
 * ⚠️ 调用前必须已经用 Arm_GotoPose(基准姿态) 把臂摆好, 否则缓存里记的 ID1
 *    起点跟实际对不上(下发瞬间会先跳一下再走)。
 * ===================================================================== */
/* ⭐ s_id1_pos 存的是 ID1 的【逻辑位置】, 不是直接下发的 0~4095:
 *   它允许越过这两个边界(取负值 / 超过 4095), 只在真正写舵机时才“取模 4096”绕回来。
 *   为什么必须这样(基准 2052 > 2048 ⇒ 归一化后是负的逻辑位置):
 *     救援基准 = 2052(逻辑 -2044), 而“目标偏左”的对准位 1561 对应逻辑
 *       -2535 —— 全在【负半轴】, 若按 0~4095 的绝对位置限幅就会被夹到 0。
 *     这套机制同时也允许越过 0/4095 边界继续绕圈(旧标定 252 → 3827 那种
 *       “过零”点位就靠它; 现在的三个位置都不需要过零)。
 *   ⚠️ 所以打靶和救援一样, 限幅都必须是【基准逻辑位置 + 相对偏移】:
 *      打靶基准 TARGET_LOOK 的 ID1 = 2052 > 2048, 归一化后是 -2044 ——
 *      若按绝对 0~4095 限幅, 第一步就会被夹到 0, 底座直接甩半圈到靶子反方向。 */
static int32_t  s_id1_pos      = 0;   /* ID1 当前【逻辑】位置(可越 0/4095 边界) */
static uint8_t  s_id1_pose_idx = 0;   /* 上面那个位置对应的【基准姿态】 */
static uint16_t s_id4_pos      = 0;   /* ID4(腕部) 当前指令位置 —— 打靶修竖直 dy 用 */
static uint8_t  s_id4_pose_idx = 0;   /* 上面那个位置对应的【基准姿态】 */

/** @brief  ⭐ 2026-10-11: 读 ID1【相对基准的累计偏移】(0 = 还没修过; 正负都有)。
 *         给"桶轻微横向误差用 ID1 修"用 —— 那个函数定义在文件上方, 读不到 s_id1_offset,
 *         所以走这个访问器。对齐/放球都用【相对偏移】, 这样姿态表里的标定差不会被破坏。 */
static int32_t Arm_Id1GetOffset(void)
{
    return s_id1_offset;
}

/** @brief  ⭐⭐ 2026-10-11: 读 ID1【逻辑位置】—— 桶对准"锁存"时要用它(与救援锁存同一个量) */
static int32_t Arm_Id1GetLogicalPos(void)
{
    return s_id1_pos;
}

/**
 * @brief  ⭐⭐ 2026-10-11(用户要求): 桶放球姿态的 ID1 = 【锁存的 ID1】+ 手动偏移
 * @param  pose_idx PLACE_PRE / PLACE_OPEN
 * @retval 下发给舵机的 ID1 位置(0~4095)
 * @note   机制与救援的 Arm_GotoRescuePoseKeepId1 完全一样: 桶精对准判定成功那一刻
 *         把 ID1 锁存(s_bucket_id1_lock), 放球时用它 ⇒ "对准时底座转到哪, 放球就在哪"。
 *         ⚠️ 两种取值方式由 BUCKET_PLACE_ID1_USE_LOCK 切(见该宏的说明):
 *            1(★当前) = 锁存值直接当放球的 ID1;
 *            0        = 只搬"对准过程产生的修正量"(锁存值 − 对准基准), 保留姿态表标定差。
 *         ⚠️ 手滑/异常值都会在最后被夹到 0~4095; 而对准阶段 ID1 本身被限在 0~40,
 *            所以锁存值不可能跑飞 ⇒ 不存在"ID1 转到 4095"这类隐患。
 *         ⚠️ 没锁存过(测试模式 / 超时直接放球)时退回姿态表里的标定值, 与老行为一致。
 */
static uint16_t Arm_BucketPlaceId1(uint8_t pose_idx)
{
    int32_t v;
    const uint16_t *src = ArmPose_Ptr(pose_idx);

    if (src == NULL) {
        return 0u;
    }
    if (!s_bucket_id1_lock_valid) {
        return src[0];     /* 没锁过: 用标定值(与老行为一致) */
    }
#if BUCKET_PLACE_ID1_USE_LOCK
    v = s_bucket_id1_lock + (int32_t)BUCKET_PLACE_ID1_OFFSET;
#else
    v = (int32_t)src[0] + (s_bucket_id1_lock - s_bucket_id1_lock_base)
        + (int32_t)BUCKET_PLACE_ID1_OFFSET;
#endif
    if (v < 0)    v = 0;
    if (v > 4095) v = 4095;
    return (uint16_t)v;
}

/**
 * @brief  把 ID1 位置缓存复位到指定基准姿态的底座值 */
static void Arm_Id1Reset(uint8_t pose_idx)
{
    int32_t v = (int32_t)s_arm_pose_table[pose_idx][0];

    s_id1_pose_idx = pose_idx;
    /* ⭐ 归一化到“逻辑位置”: 超过半圈(2048)的基准值记成小负数,
     *    这样“往左转”时可以直接减下去、越过 0 继续绕(见 s_id1_pos 的注释)。
     * ⚠️ 打靶/救援的基准现在都是 2052(> 2048) ⇒ 都会归一化成 -2044,
     *    所以调用 Arm_Id1Step 的限幅必须写【基准 + 相对偏移】
     *    (见 Target_Id1Base / Rescue_Id1Base), 不能写绝对 0~4095。 */
    s_id1_pos      = (v > 2048) ? (v - 4096) : v;
    s_id1_offset   = 0;                 /* 换基准: 相对偏移从 0 重新开始 */
    MLOG("底座ID1: 基准姿态复位为 %s -> 逻辑位置 %d",
         ArmAction_GetName(pose_idx), (int)s_id1_pos);
}

/** @brief 把 ID4(腕部) 位置缓存复位到指定基准姿态的值 —— 打靶修竖直用 */
static void Arm_Id4Reset(uint8_t pose_idx)
{
    s_id4_pose_idx = pose_idx;
    s_id4_pos      = s_arm_pose_table[pose_idx][3];   /* [3]=ID4 腕部 */
    s_id4_offset   = 0;
    MLOG("腕部ID4: 基准姿态复位为 %s -> %d",
         ArmAction_GetName(pose_idx), (int)s_id4_pos);
}

/**
 * @brief  让底座 ID1 相对当前位置转一步(不阻塞)
 * @param  pose_idx 基准姿态(打靶=ARM_POSE_TARGET_LOOK, 救援=ARM_POSE_HOSTAGE_LOOK)
 * @param  delta    角度码增量: 正 = 数值增大, 负 = 数值减小
 * @param  move_ms  本步转动时间(ms)
 * @param  pos_min  限幅下限 —— 【逻辑位置】(可为负, 见下), 防越界堵转
 * @param  pos_max  限幅上限 —— 【逻辑位置】
 * @param  speed_min 本步速度下限(步/秒), 见 ServoArm_CalcSpeedEx():
 *                  传 SERVO_SPEED_MIN_DEF = 老行为; 打靶精对准传
 *                  TARGET_FINE_SPEED_MIN 让微动真的慢下来(见该宏的说明)
 * @param  acc       本步加速度(0~254)
 * @note   换了基准姿态时自动重新对齐缓存; 单步增量远小于 2048, 不会触发
 *         飞特舵机的“最短路径反向甩”问题。
 * ⭐ 2026-10-07: 限幅改成 int32 的【逻辑位置】, 并且下发前“取模 4096”,
 *    于是可以【越过 0/4095 边界继续绕圈】: 打靶/救援的基准都 > 2048(归一化成
 *    负数), 只有“逻辑位置 + 取模下发”才能正确走到(见 s_id1_pos 的注释)。
 * ⚠️⚠️ 调用方传的 pos_min/pos_max 是【逻辑位置】的绝对边界, 必须覆盖基准值:
 *    打靶基准(TARGET_LOOK 的 ID1 = 2052)归一化后是 -2044, 救援基准同理 ——
 *    两边都用 Target_Id1Base()/Rescue_Id1Base() 算出基准再 + 相对偏移传入。
 */
static void Arm_Id1Step(uint8_t pose_idx, int32_t delta, uint16_t move_ms,
                        int32_t pos_min, int32_t pos_max,
                        uint16_t speed_min, uint8_t acc)
{
    uint16_t pose[SERVO_COUNT];
    int32_t  v;
    int32_t  real_delta;
    int32_t  send;

    if (s_id1_pose_idx != pose_idx) {
        Arm_Id1Reset(pose_idx);          /* 换了阶段/基准姿态: 重新对齐 */
    }
    v = s_id1_pos + delta;
    if (v < pos_min) v = pos_min;
    if (v > pos_max) v = pos_max;
    real_delta = v - s_id1_pos;            /* 限幅后真正走了多少 */
    s_id1_pos    = v;
    s_id1_offset += real_delta;            /* ⭐ 累计偏移: 打靶摆发射位/救援选抓取位要用 */

    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        pose[i] = s_arm_pose_table[pose_idx][i];
    }
    /* ⭐ 逻辑位置 → 真正下发的 0~4095: 取模 4096(负值也要能正确绕回来) */
    send = s_id1_pos % 4096;
    if (send < 0) {
        send += 4096;
    }
    pose[0] = (uint16_t)send;

    MLOG("底座ID1[%s] 转 %+ld -> 逻辑位置 %d (下发 %d) (相对基准累计 %+ld)",
         ArmAction_GetName(pose_idx), (long)delta, (int)s_id1_pos, (int)send,
         (long)s_id1_offset);
    Servos_SetPositionsMaskedEx(pose, SERVO_MASK_ID1, move_ms, speed_min, acc);
}

/**
 * @brief  让腕部 ID4 相对当前位置转一步(不阻塞) —— 打靶修【竖直误差 dy】用
 * @param  pose_idx 基准姿态(打靶=ARM_POSE_TARGET_LOOK)
 * @param  delta    角度码增量: 正 = 数值增大, 负 = 数值减小
 * @param  move_ms  本步转动时间(ms)
 * @param  pos_min  限幅下限(角度码), 防越界堵转
 * @param  pos_max  限幅上限(角度码)
 * @param  speed_min 本步速度下限(步/秒); 打靶精对准传 TARGET_FINE_SPEED_MIN
 * @param  acc       本步加速度(0~254)
 * @note   除被选中的舵机不同(ID4)外, 机制与 Arm_Id1Step 完全一样:
 *         只发 ID4(掩码 SERVO_MASK_ID4), 其余舵机位置数组里填基准姿态值
 *         但因为没被选中所以不会动。
 *         ⚠️ 转 ID4 会同时影响画面的横向(横滚耦合), 所以必须与 ID1 【交替】修,
 *            绝不能两个同时发指令。
 */
static void Arm_Id4Step(uint8_t pose_idx, int32_t delta, uint16_t move_ms,
                        uint16_t pos_min, uint16_t pos_max,
                        uint16_t speed_min, uint8_t acc)
{
    uint16_t pose[SERVO_COUNT];
    int32_t  v;
    int32_t  real_delta;

    if (s_id4_pose_idx != pose_idx) {
        Arm_Id4Reset(pose_idx);          /* 换了阶段/基准姿态: 重新对齐 */
    }
    v = (int32_t)s_id4_pos + delta;
    if (v < (int32_t)pos_min) v = (int32_t)pos_min;
    if (v > (int32_t)pos_max) v = (int32_t)pos_max;
    real_delta = v - (int32_t)s_id4_pos;
    s_id4_pos    = (uint16_t)v;
    s_id4_offset += real_delta;

    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        pose[i] = s_arm_pose_table[pose_idx][i];
    }
    pose[3] = s_id4_pos;                 /* [3]=ID4 腕部 */

    MLOG("腕部ID4[%s] 转 %+ld -> 新位置 %d (相对基准累计 %+ld)",
         ArmAction_GetName(pose_idx), (long)delta, (int)s_id4_pos, (long)s_id4_offset);
    Servos_SetPositionsMaskedEx(pose, SERVO_MASK_ID4, move_ms, speed_min, acc);
}

/* ---- 打靶 / 救援 各自的封装: 把各自的参数宏收口在一处, 调用点更短 ---- */
/**
 * @brief  打靶底座 ID1 的【基准逻辑位置】(与 Rescue_Id1Base 同构)
 * @note   取 TARGET_LOOK 那一行的 ID1(现在 = 2052)。
 *         超过半圈(2048)的基准值归一化成小负数(2052 -> -2044), 这样“往左转”
 *         可以直接减下去、越过 0 绕圈(见 s_id1_pos 的注释)。
 * ⚠️⚠️ 限幅必须是【这个基准值 + 相对偏移】。用绝对 0~4095 会把 -2044 夹成 0,
 *        底座就会甩半圈到靶子反方向(见 TARGET_ID1_POS_MIN 的注释)。
 */
static int32_t Target_Id1Base(void)
{
    int32_t v = (int32_t)s_arm_pose_table[ARM_POSE_TARGET_LOOK][0];

    return (v > 2048) ? (v - 4096) : v;
}

/** @brief 打靶: ID1 以 TARGET_LOOK 为基准转一步【精对准微动用: 速度柔和】 */
static void Target_Id1Step(int32_t delta)
{
    int32_t base = Target_Id1Base();

    Arm_Id1Step(ARM_POSE_TARGET_LOOK, delta, TARGET_ID1_MOVE_MS,
                base + (int32_t)TARGET_ID1_POS_MIN,
                base + (int32_t)TARGET_ID1_POS_MAX,
                TARGET_FINE_SPEED_MIN, TARGET_FINE_ACC);
}

/** @brief 打靶: ID1 以 TARGET_LOOK 为基准转一步【粗对准用: 速度参数与以前一致】
 *  @note  与 Target_Id1Step 只差速度下限/加速度: 粗对准那一步是 30 码(≈53px),
 *         用通用下限 200 步/秒时约 150ms 走完 —— 保持原样(不动它的手感);
 *         精对准的几码微动才需要柔(见 TARGET_FINE_SPEED_MIN 的说明)。 */
static void Target_Id1CoarseStep(int32_t delta)
{
    int32_t base = Target_Id1Base();

    Arm_Id1Step(ARM_POSE_TARGET_LOOK, delta, TARGET_ID1_MOVE_MS,
                base + (int32_t)TARGET_ID1_POS_MIN,
                base + (int32_t)TARGET_ID1_POS_MAX,
                SERVO_SPEED_MIN_DEF, SERVO_ACC_DEF);
}

/**
 * @brief  按当前横向误差大小选一个“合适的” ID1 步长(自适应)
 * @param  err_x  K230 回的横向误差(px)
 * @retval 本步应该转的角度码(已含方向)
 * @note   分档理由/实测数据见上面 TARGET_ID1_STEP_BIG_PX 处的注释。
 */
static int32_t Target_Id1StepFor(int err_x)
{
    int32_t a    = abs(err_x);
    int32_t base;

    if (a > TARGET_ID1_STEP_BIG_PX) {
        base = TARGET_ID1_STEP;                 /* 大步: 快 */
    } else if (a > TARGET_ID1_STEP_MID_PX) {
        base = TARGET_ID1_STEP / 2;             /* 中步 */
    } else {
        base = TARGET_ID1_STEP_FINE;            /* 小步: 不会再跨过中心 */
    }
    /* 方向与原来一致: 画面偏左(err_x>0) → 按 LR_SIGN 转 */
    return (err_x > 0) ? (base * (int32_t)TARGET_ID1_LR_SIGN)
                       : (-base * (int32_t)TARGET_ID1_LR_SIGN);
}

/**
 * @brief  救援底座 ID1 的【基准位置】(转成“可过零”的逻辑值)
 * @note   取 HOSTAGE_LOOK 那一行的 ID1(现在 = 2052 = 目标居中位)。
 *         超过半圈(2048)的基准值记成小负数, 这样“继续往左”时可以直接减下去、
 *         越过 0 绕一圈(见 s_id1_pos 的注释); 正常范围时就是原值不变。
 */
static int32_t Rescue_Id1Base(void)
{
    int32_t v = (int32_t)s_arm_pose_table[ARM_POSE_HOSTAGE_LOOK][0];

    return (v > 2048) ? (v - 4096) : v;
}

/** @brief 救援: ID1 以 HOSTAGE_LOOK 为基准转一步
 *  ⚠️ 限幅是【相对基准的偏移】写成 base + RESCUE_ID1_POS_MIN/MAX;
 *     按现在的标定: 2052 + (-578) = 1474(下限), 2052 + (+524) = 2576(上限)
 *     ⇒ 左右两个对准位(1561 / 2503)各留约 80 码余量。 */
static void Rescue_Id1Step(int32_t delta)
{
    int32_t base = Rescue_Id1Base();

    Arm_Id1Step(ARM_POSE_HOSTAGE_LOOK, delta, RESCUE_ID1_MOVE_MS,
                base + (int32_t)RESCUE_ID1_POS_MIN,
                base + (int32_t)RESCUE_ID1_POS_MAX,
                SERVO_SPEED_MIN_DEF, SERVO_ACC_DEF);
}

/**
 * @brief  按当前横向误差大小选一个“合适的”救援 ID1 步长(自适应, 与打靶同构)
 * @param  err_x  K230 回的横向误差(px), 正 = 目标偏画面左
 * @retval 本步应该转的角度码(已含方向)
 * @note   分档理由 / 怎么定准见上面 RESCUE_ID1_STEP_FINE 处的注释。
 *         方向与粗对准一致: x>0 与接近阶段的 'L' 同向 ⇒ 用 RESCUE_ID1_LR_SIGN。
 */
static int32_t Rescue_Id1StepFor(int err_x)
{
    int32_t a    = abs(err_x);
    int32_t base;

    if (a > RESCUE_ID1_STEP_BIG_PX) {
        base = RESCUE_ID1_STEP;                 /* 大步: 快 */
    } else if (a > RESCUE_ID1_STEP_MID_PX) {
        base = RESCUE_ID1_STEP / 2;             /* 中步 */
    } else {
        base = RESCUE_ID1_STEP_FINE;            /* 小步: 不会再跨过中心 */
    }
    return (err_x > 0) ? (base * (int32_t)RESCUE_ID1_LR_SIGN)
                       : (-base * (int32_t)RESCUE_ID1_LR_SIGN);
}

/* =====================================================================
 * ⭐ 救援搜目标: ID1 慢速巡视 (实现; 思路/调参见上面 RESCUE_SWEEP_* 处的注释)
 * ===================================================================== */

/* 巡视点位(⭐ 直接写【标定好的角度码】, 归一化交给 Rescue_Id1Norm 做)
 * ⭐ 2026-10-07 改成 5 个点(用户要求): 中间 → 右 → 中间 → 左 → 中间
 *     中间 = RESCUE_ID1_MID_POS   (2059) → 归一化     0
 *     右   = RESCUE_ID1_RIGHT_POS (2503) → 归一化  +444
 *     左   = RESCUE_ID1_LEFT_POS  (1561) → 归一化  -498
 *   每扫完一侧都回中间一次: ① 中间是基准, 回去后相机横向参考最正;
 *   ② 避免从最右一步甩到最左(942 码)造成画面拖影; ③ 最后停在中间,
 *   所以扫完一圈后 s_id1_offset 正好回 0。 */
static const int32_t s_rescue_sweep_code[5] = {
    RESCUE_ID1_MID_POS,     /* 1) 中 2059 */
    RESCUE_ID1_RIGHT_POS,   /* 2) 右 2503 */
    RESCUE_ID1_MID_POS,     /* 3) 中 2059 */
    RESCUE_ID1_LEFT_POS,    /* 4) 左 1561 */
    RESCUE_ID1_MID_POS      /* 5) 中 2059 */
};

/**
 * @brief  把 ID1 转到指定的【绝对角度码】位置(只动 ID1, 不阻塞等待)
 * @param  code    目标角度码(0~4095): 中间 = 2059 / 右 = 2503 / 左 = 1561
 * @param  move_ms 本步转动时间(巡视要慢, 所以可单独指定)
 * @note   ⭐ 内部先用 Rescue_Id1Norm() 把“角度码”归一化成“相对基准(2059)的偏移”,
 *         再走最短路径:
 *             2059 →    0     (不用动)
 *             2503 → +444     (往右转 444 码)
 *             1561 → -498     (往左转 498 码)
 *         ⚠️ 归一化同时防住“跨 0/4095 圈边界时方向反掉”(旧标定 252 → 3827 就是:
 *            直接相减得 +3575 码 —— 看着像“往右转几乎一整圈”)。
 *         相对基准的偏移量照样累加到 s_id1_offset, 所以巡视走完一圈
 *         (最后一个点 = 2059) 时偏移正好回到 0。
 *         和 Rescue_Id1Step 的区别: 这个是【绝对点位】(内部反算 delta),
 *         并且能指定本步转动时间。
 */
static void Rescue_Id1Goto(int32_t code, uint16_t move_ms)
{
    int32_t base;
    int32_t off;
    int32_t want;
    int32_t delta;
    int32_t now_code;
    int32_t want_code;

    /* 基准没对齐过就先对齐, 否则下面的减法是在两套坐标系里算, 会转飞 */
    if (s_id1_pose_idx != ARM_POSE_HOSTAGE_LOOK) {
        Arm_Id1Reset(ARM_POSE_HOSTAGE_LOOK);
    }
    off   = Rescue_Id1Norm(code);          /* ⭐ 角度码 → 归一化偏移 */
    base  = Rescue_Id1Base();
    want  = base + off;
    delta = want - s_id1_pos;

    now_code  = ((s_id1_pos % 4096) + 4096) % 4096;   /* 当前实际下发的角度码 */
    want_code = ((want % 4096) + 4096) % 4096;        /* 本点位对应的角度码   */

    if (delta == 0) {
        MLOG("[救援] 巡视点位: ID1 %d (归一化偏移 %+ld, 已在位不用动)",
             (int)now_code, (long)off);
        return;      /* 已经在位: 不用发指令(否则会多一次无意义的下发) */
    }
    MLOG("[救援] 巡视点位: ID1 %d -> %d (归一化偏移 %+ld 码, %s)",
         (int)now_code, (int)want_code, (long)off,
         (off > 0) ? "偏右" : ((off < 0) ? "偏左(过零绕行)" : "中间"));
    Arm_Id1Step(ARM_POSE_HOSTAGE_LOOK, delta, move_ms,
                base + (int32_t)RESCUE_ID1_POS_MIN,
                base + (int32_t)RESCUE_ID1_POS_MAX,
                SERVO_SPEED_MIN_DEF, SERVO_ACC_DEF);
}

/**
 * @brief  ID1 慢速巡视一圈(阻塞; 一旦收到 K230 数据就立刻停下来)
 * @param  out_line 输出: 中止时收到的那一行
 * @param  out_size out_line 的字节数
 * @retval 1 = 巡视途中收到了 K230 数据(已写入 out_line)
 *         0 = 扫完一圈啥都没收到(已回到中间)
 * @note   ⚠️ 会阻塞约 5×(MOVE+HOLD) ≈ 11.5s; 期间用 Mission_Coop_Wait 等,
 *         照刷陀螺仪 yaw、照收 K230 行(否则收不到中止条件)。
 */
static uint8_t Rescue_SweepOnce(char *out_line, uint16_t out_size)
{
    /* ⭐ 本次巡视用的“稳定帧”滤波器: 状态【跨点位】保留 ——
     *    同一个目标在两个点位连着被看到 2 次也算数(不是每换一个点又重新数) */
    VisionStable_t vs;

    Vision_StableReset(&vs);
    MLOG("[救援] ID1 巡视开始(5 个点): 中 %d -> 右 %d -> 中 %d -> 左 %d -> 中 %d "
         "(归一化偏移: 中 0 / 右 %+ld / 左 %+ld; 每点转 %dms + 停 %dms, 只动 ID1; "
         "需连续 %d 帧一致才中止巡视)",
         (int)s_rescue_sweep_code[0], (int)s_rescue_sweep_code[1],
         (int)s_rescue_sweep_code[2], (int)s_rescue_sweep_code[3],
         (int)s_rescue_sweep_code[4],
         (long)Rescue_Id1Norm(RESCUE_ID1_RIGHT_POS),
         (long)Rescue_Id1Norm(RESCUE_ID1_LEFT_POS),
         (int)RESCUE_SWEEP_MOVE_MS, (int)RESCUE_SWEEP_HOLD_MS,
         (int)VISION_STABLE_FRAMES_SWEEP);

    for (uint8_t k = 0; k < 5u; k++) {
        uint32_t span = (uint32_t)RESCUE_SWEEP_MOVE_MS + (uint32_t)RESCUE_SWEEP_HOLD_MS;
        uint32_t t0;

        Rescue_Id1Goto(s_rescue_sweep_code[k], RESCUE_SWEEP_MOVE_MS);
        t0 = HAL_GetTick();
        while ((HAL_GetTick() - t0) < span) {
            if (Mission_GetNewLine(out_line, out_size)) {
                /* ⭐ 稳定帧: 连续 N 帧同一个 token 才算“真看到目标”,
                 *    单帧误检【不中止巡视】(旧代码是任何一行就中止)。
                 *    ⚠️ timeout 传 0: 凑不齐就一直扫(反正扫完一圈回去继续等) */
                if (Vision_StableFeed(&vs, Vision_TokenOf(out_line),
                                      VISION_STABLE_FRAMES_SWEEP, 0u, "巡视")) {
                    MLOG("[救援] 巡视到第 %u 个点(ID1 %d, 归一化偏移 %+ld 码)时 K230 稳定回应: %s "
                         "-> 中止巡视, 转正常对准",
                         (unsigned)(k + 1u), (int)s_rescue_sweep_code[k],
                         (long)Rescue_Id1Norm(s_rescue_sweep_code[k]), out_line);
                    return 1;
                }
                continue;   /* 还没稳: 继续在本点位等, 不中止巡视 */
            }
            Mission_Coop_Wait(20);
        }
    }
    MLOG("[救援] ID1 巡视结束(一圈都没有回应), 已回到中间继续等 K230");
    return 0;
}

/** @brief 打靶: ID4(腕部) 以 TARGET_LOOK 为基准转一步(修竖直误差 dy)
 *  @note  速度用精对准的柔性参数(腕部在最外端, 抽搐式微动晃得最明显) */
static void Target_Id4Step(int32_t delta)
{
    Arm_Id4Step(ARM_POSE_TARGET_LOOK, delta,
                TARGET_ID4_MOVE_MS, TARGET_ID4_POS_MIN, TARGET_ID4_POS_MAX,
                TARGET_FINE_SPEED_MIN, TARGET_FINE_ACC);
}

/**
 * @brief  按当前竖直误差大小选一个 ID4 步长(自适应, 与 ID1 同构)
 * @param  err_y  K230 回的竖直误差(px)
 * @retval 本步应该转的角度码(已含方向)
 * @note   分档理由 / 实测反馈见上面 TARGET_ID4_STEP 处的注释。
 */
static int32_t Target_Id4StepFor(int err_y)
{
    int32_t a    = abs(err_y);
    int32_t base;

    if (a > TARGET_ID4_STEP_FAR_PX) {
        base = TARGET_ID4_STEP_FAR;             /* 大步: 快 */
    } else if (a > TARGET_ID4_STEP_FINE_PX) {
        base = TARGET_ID4_STEP;                 /* 中步 */
    } else {
        base = TARGET_ID4_STEP_FINE;            /* 细步: 不会再跨过中心 */
    }

    /* 方向与原来一致: y>0(目标偏画面下) → 按 DY_SIGN 转 */
    return (err_y > 0) ? (base * (int32_t)TARGET_ID4_DY_SIGN)
                       : (-base * (int32_t)TARGET_ID4_DY_SIGN);
}

#if 0   /* ⚠️ 2026-10-06 【已停用, 保留备用】摆激光发射位
         * 现在收到 K230 的 OK 后直接抬大臂(TARGET_LIFT), 不再摆 TARGET_FIRE,
         * 所以这个函数暂时用不到。
         * 以后如果要恢复: 把 #if 0 改成 #if 1, 并在 TARGET_PERFORM 里
         * 用 Target_GotoFirePose() 代替 Arm_Start_Target_Lift() 的前一步。 */
/**
 * @brief  摆到“打靶发射位”, 并把精对准期间 ID1/ID4 的偏差补回来
 * @note   ⚠️⚠️ 为什么不能直接 Arm_GotoPose(ARM_POSE_TARGET_FIRE):
 *           Arm_GotoPose 会把【整张姿态表】写下去, ID1/ID4 会被拉回姿态表里的
 *           标定值(TARGET_FIRE: ID1=2052 / ID4=2175)。而精对准为了对准靶心,
 *           刚把 ID1 转了(水平)、ID4 转了(竖直) —— 不补偏差的话前面全白对,
 *           激光依旧会照偏。
 *         这里把 s_id1_offset / s_id4_offset 叠加到 TARGET_FIRE 对应舵机上再下发,
 *         其余 3 个舵机(ID2/ID3/ID5)照旧用 TARGET_FIRE 的标定值。
 *         阻塞时长 = TARGET_FIRE 自己的运动时间 + 保持时间(与其他姿态一致)。
 */
static void Target_GotoFirePose(void)
{
    uint16_t pose[SERVO_COUNT];
    int32_t  v1, v4;

    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        pose[i] = s_arm_pose_table[ARM_POSE_TARGET_FIRE][i];
    }
    v1 = (int32_t)pose[0] + s_id1_offset;
    v4 = (int32_t)pose[3] + s_id4_offset;
    /* ⚠️ 这里的 v1 是【绝对】位置(TARGET_FIRE 的 ID1 标定值 2052 + 偏移),
     *    所以限幅用绝对 0~4095, 不能用已经改成【相对基准偏移】的
     *    TARGET_ID1_POS_MIN/MAX(-2048/+2047)。 */
    if (v1 < 0) v1 = 0;
    if (v1 > 4095) v1 = 4095;
    if (v4 < (int32_t)TARGET_ID4_POS_MIN) v4 = (int32_t)TARGET_ID4_POS_MIN;
    if (v4 > (int32_t)TARGET_ID4_POS_MAX) v4 = (int32_t)TARGET_ID4_POS_MAX;
    pose[0] = (uint16_t)v1;
    pose[3] = (uint16_t)v4;

#if MISSION_TEST_NO_ARM
    MLOG("[靶] 发射位(含对准偏差 ID1%+ld / ID4%+ld) —— MISSION_TEST_NO_ARM=1, 跳过",
         (long)s_id1_offset, (long)s_id4_offset);
#else
    MLOG("[靶] 摆发射位: TARGET_FIRE + 对准偏差 (ID1 %+ld -> %d, ID4 %+ld -> %d)",
         (long)s_id1_offset, (int)pose[0], (long)s_id4_offset, (int)pose[3]);
    Servos_SetPositions(pose, s_arm_pose_time[ARM_POSE_TARGET_FIRE]);
    Mission_Coop_Wait(s_arm_pose_time[ARM_POSE_TARGET_FIRE] + Arm_HoldMs(ARM_POSE_TARGET_FIRE));
#endif
}
#endif  /* 0: 摆发射位(已停用) */
#endif /* !MISSION_TEST_NO_VISION */

/**
 * @brief  视觉辅助任务(任务1 排爆 / 任务2 打靶 / 任务3 救援 共用)
 *
 * ⚠️ 任务2(打靶)已改版: 不走下面的“粗对准①②”, 而是“只转底座 ID1”。
 *    详细流程见函数内“任务2”那一段注释。
 *
 * 每个任务的通用套路(三个阶段):
 *   ① “粗对准”: MCU 发 run_task:<n>，K230 回一个字符
 *              C = 已在目标正中 / L = 目标偏画面左 / R = 目标偏画面右
 *              偏了就按宏“盲走”一小段(距离由 *_ADJUST_MM 宏定)
 *              ⚠️ 画面左/右 对应 车体的哪个方向, 跟【当前机械臂姿态】有关!
 *                 详见文件头“⭐ 视觉方向映射”表 + 各任务的 AlignAxisCfg_t
 *   ② “精对准”: MCU 发 start_align，K230 不断回 D:<x>,<y> (像素误差)
 *               MCU 按 K_GAIN 换算成毫米, 一段一段修正到 |误差| < ALIGN_TOLERANCE
 *               只修正本阶段“允许修”的轴(由 s_align_* 配置), 其它轴忽略
 *               K230 回 OK 或超时 也算对准结束
 *   ③ “执行”:   停车稳定 FINE_TUNE_SETTLE_MS 后, 让机械臂做对应动作
 *               ⭐ 例外: 桶(BUCKET)在“左右对准完成”之后会先往后退
 *                  BUCKET_AFTER_ALIGN_BACK_{SMALL,BIG}_MM(按前后误差分档, 10 或 15mm),
 *                  再停车稳定 —— 见 BOMB_BUCKET_BACK_OFF 子状态。
 *
 * ⭐ 改方向时只需要动三处:
 *      (a) 文件头“视觉方向映射”注释表
 *      (b) ALIGN_AXIS_SCHEME_1 选择方案一/方案二
 *      (c) 对应的 s_align_ball / bucket / target / rescue 初始化
 *      —— 看车迷路了不用去改下面的大段 switch。
 */
static void Handle_Vision_Alignment(uint8_t expected_task_number)
{
    typedef enum {
        BOMB_IDLE,                 /* 发 run_task:1(球), 等 C/L/R */
        BOMB_WAIT_INITIAL_MOVE,    /* 等球接近盲走完成 */
        BOMB_REQUEST_FINE_TUNE,    /* 发 start_align(球) */
        BOMB_WAIT_FINE_ALIGN,      /* D/OK 精对准球 */
        BOMB_SETTLE,               /* 对准球后停稳 */
        BOMB_PERFORM_GRAB,         /* 抓球 + 盲走回放置区 */
        BOMB_ADJUST_FOR_PLACE,     /* 等盲走完成 */
        BOMB_REQUEST_BUCKET_DIR,   /* 发 run_task:3(桶), 等 C/L/R */
        BOMB_WAIT_BUCKET_DIR_MOVE, /* 等桶接近盲走完成 */
        BOMB_REQUEST_BUCKET_FINE,  /* 发 start_align(桶) */
        BOMB_WAIT_BUCKET_ALIGN,    /* D/OK 精对准桶 */
        BOMB_BUCKET_BACK_OFF,      /* ⭐ 桶左右对准完成后: 按前后误差后退 10/15mm, 等它走完 */
        BOMB_SETTLE_PLACE,         /* 对准桶后停稳 */
        BOMB_PERFORM_PLACE,        /* 放球 */
        BOMB_COMPLETE
    } BombSubState_t;

    /* 任务2: 打靶(2026-10-05 适配新 K230) —— 底盘不动, ID1 修横向 / ID4 修竖直;
     *
     *   ① TARGET_IDLE        发 run_task:2 → K230 回 C/L/R(接近阶段) → 粗转 ID1
     *   ② TARGET_ID1_MOVING  等粗转那一步走完
     *   ③ TARGET_FINE_IDLE   ⭐ 收到 C 后发 start_align, K230 进入 ALIGN 状态;
     *                        之后按 D:<x>,<y> 分别微调 ID1(横向)/ID4(竖直);
     *                        收到 OK 就结束(K230 会先发一行 FIRE 通知已打激光)
     *   ④ TARGET_FINE_MOVING 等微调那一步走完
     *   ⑤ TARGET_PERFORM     收到 OK 后: 抬起大臂 → 收回手臂(不再摆发射位)
     *   ⑥ TARGET_COMPLETE    回主状态机
     *   (TARGET_USE_FINE_ALIGN=0 时跳过 ③④, 收到 C 直接进 ⑤) */
    typedef enum {
        TARGET_IDLE,            /* 发 run_task:2(靶), 等 K230 回 C/L/R */
        TARGET_ID1_MOVING,      /* 刚转了一步 ID1, 等它走完(TARGET_ID1_MOVE_MS) */
        TARGET_FINE_IDLE,       /* 已发 start_align: 等 D:<x>,<y> / OK */
        TARGET_FINE_MOVING,     /* 刚按 D 挪了一步舵机(ID1 或 ID4), 等它走完 */
        TARGET_HOLD,            /* ⭐ 收到 OK: 保持当前对准姿态 TARGET_FIRE_HOLD_MS
                                 *    (让 K230 的激光稳稳打在靶上), 再抬臂 */
        TARGET_PERFORM,         /* 保持完: 抬起大臂 → 收回手臂 */
        TARGET_COMPLETE
    } TargetSubState_t;

    /* 任务3: 救援(停下给视觉→对准→抓取; K230 任务号 = 4 = K230_TASK_RESCUE)
     * 两种对准方案由 RESCUE_SCHEME_ID1 切换(见文件头“救援任务参数”) */
    typedef enum {
        RESCUE_IDLE,            /* 发 run_task:4(形状), 等 K230 回 C/L/R */
        RESCUE_WAIT_ADJUST,     /* [方案一] 视觉给 L/R → 车前进 / 右移一小段, 等它走完 */
        RESCUE_WAIT_ALIGN,      /* [方案一] 精对准(D:/OK, 只修前后) */
        RESCUE_ID1_MOVING,      /* [方案二] 刚转了一步 ID1, 等它走完(RESCUE_ID1_MOVE_MS) */
        RESCUE_SETTLE,          /* 对准后停车稳定 */
        RESCUE_PERFORM,         /* 执行营救动作(HOSTAGE_PRE → CLOSE → LIFT) */
        RESCUE_COMPLETE
    } RescueSubState_t;

    static BombSubState_t bomb_sub_state = BOMB_IDLE;
    static TargetSubState_t target_sub_state = TARGET_IDLE;
    static RescueSubState_t rescue_sub_state = RESCUE_IDLE;
    static char bomb_path_taken = ' ';
    static char bucket_path_taken = ' ';
#if !MISSION_TEST_NO_VISION
    static char target_path_taken = ' ';
    static char rescue_path_taken = ' ';
    /* ⭐ 打靶粗对准(C/L/R)的“过冲”检测: 上一步的方向(+1=L / -1=R / 0=还没有)
     *   和“方向翻转”次数 —— 翻转 = 上一步跨过了画面中心。
     *   用途: 翻转后改用细步 TARGET_COARSE_STEP_MIN, 见那段宏的说明。 */
    static int8_t   target_coarse_dir  = 0;
    static uint16_t target_coarse_flip = 0;
#endif
    static uint32_t state_start_tick = 0;
    static uint32_t align_start_time = 0;
    /* (原 laser_start_time 已删: 激光改由 K230 控制, 单片机不再计时) */
    static uint32_t cooldown_until = 0;   /* 脉冲驱动冷却截止时刻(ms) */
    static uint32_t settle_until = 0;     /* 停车稳定等待截止时刻(ms) */
    static uint32_t vision_cmd_tick = 0;  /* 视觉指令发送时刻(run_task 定时重发) */
    static uint32_t rescue_idle_tick = 0; /* 救援视觉等待起点(超时盲走用) */

    /* ---- 打靶专用: 底座 ID1 原地对准的状态量 ---- */
    static uint8_t  target_id1_inited = 0;    /* 0=本次打靶还没开始 L/R 对准 */
    static uint8_t  target_heard = 0;         /* 0=还没收到过 K230 任何回应
                                               * (只有这个阶段才需要定时重发 run_task,
                                               *  收到回应后就不再打扰 K230) */
    static uint16_t target_id1_steps = 0;     /* 本次已转了多少步(防卡死①) */
    static uint32_t target_id1_tick = 0;      /* 本次 L/R 对准起始时刻(防卡死②) */
    static uint32_t target_id1_move_tick = 0; /* 本步 ID1 开始转动时刻 */
    static uint16_t target_fine_steps = 0;    /* ⭐ 精对准横向(ID1)已微调了多少步(防卡死①) */
    static uint32_t target_fine_tick = 0;     /* ⭐ 精对准起始时刻(防卡死②) */
    static uint32_t target_fine_cmd_tick = 0; /* ⭐ 上次发 start_align 的时刻(重试用) */
    static uint32_t target_fire_tick = 0;     /* ⭐ 收到 K230 "FIRE" 的时刻(0=还没收到; 用来做宽限收尾) */
    static uint8_t  target_fine_xok_warned = 0; /* 两轴都已达标但K230仍在发D: 只提示一次(防刷屏) */
    static uint8_t  target_freeze_warned   = 0; /* ⭐ 2026-10-11: 进"冻结带"只提示一次(防刷屏) */
    static uint16_t target_fine_move_ms = 0;    /* ⭐ 本步要等多久(ID1/ID4 的转动时间不同) */
    static uint16_t target_id4_steps = 0;       /* ⭐ 精对准竖直(ID4)已微调了多少步(防卡死) */
    static int32_t  target_last_dy = 0;         /* ⭐ 上一步 ID4 动作后的竖直误差(用来算 ID4 步长) */
    static uint8_t  target_last_dy_valid = 0;
    static int32_t  target_last_dy_step = 0;    /* ⭐ 上一步 ID4 【实际】转了多少码(步长自适应, 必须记实值) */
    /* ⭐⭐ 2026-10-10 精对准稳定性: 记“上一次横向/竖直决策时误差的符号”。
     *   下一次误差符号与之相反 ⇒ 上一步跨过了画面中心 ⇒ 改用微步。
     *   用途/原理见 TARGET_FINE_MICRO_ENABLE 处的说明。 */
    static int32_t  target_prev_ex = 0;
    static uint8_t  target_prev_ex_valid = 0;
    static int32_t  target_prev_ey = 0;
    static uint8_t  target_prev_ey_valid = 0;
    /* ⭐⭐ 2026-10-11(用户要求): "丢靶"自恢复的状态量 —— 见 TARGET_LOST_SEARCH_ENABLE */
    static uint16_t target_lost_cnt     = 0;   /* 已做过几次"丢靶搜索" */
    static uint32_t target_lost_tick    = 0;   /* 上次搜索的时刻 */
    static int8_t   target_lost_dir     = 1;   /* 本次搜索方向(交替: +1 码值增大/上抬, -1 下低) */
    static uint8_t  target_fine_d_seen  = 0;   /* 精对准期间【收到过 D 帧】= 摄像头本来能看到 */

    /* ---- 救援专用(方案二): 底座 ID1 原地对准的状态量 ---- */
    static uint8_t  rescue_id1_inited = 0;      /* 0=本次救援还没开始 L/R 对准 */
    static uint8_t  rescue_heard      = 0;      /* 0=还没收到过 K230 任何回应 */
    static uint16_t rescue_id1_steps  = 0;      /* 本次已转了多少步(防卡死①) */
    static uint32_t rescue_id1_tick   = 0;      /* 本次 L/R 对准起始时刻(防卡死②) */
    static uint32_t rescue_id1_move_tick = 0;   /* 本步 ID1 开始转动时刻 */
    static uint8_t  rescue_swept      = 0;      /* 0=本轮救援还没做 ID1 巡视;
                                                 *   1=巡视阶段已结束(可能扫了多圈, 见
                                                 *   RESCUE_SWEEP_MAX_ROUNDS) */
    static uint8_t  rescue_fine_req   = 0;      /* 1=已发 start_align(进入 D 精对准阶段),
                                                 *   见 RESCUE_USE_FINE_ALIGN */
    static uint8_t  rescue_have_first = 0;      /* 1=巡视截下了一行, 下一轮先用它 */
    static char     rescue_first_line[K230_LINE_MAX]; /* 巡视中止时截下的那一行 */
    /* ⭐ 救援精调(ID1)步长标定用: 与打靶的 target_last_dy* 同构 */
    static int32_t  rescue_last_dx = 0;         /* 上一步 ID1 动作前/后的横向误差(px) */
    static uint8_t  rescue_last_dx_valid = 0;
    static int32_t  rescue_last_dx_step = 0;    /* 上一步 ID1 【实际】转了多少码(步长自适应, 必须记实值) */

    char line[K230_LINE_MAX];

    /* ================= 任务1: 排爆(抓小球 -> 对准桶放置) ================= */
    if (expected_task_number == 1) {
        switch (bomb_sub_state) {
            case BOMB_IDLE:
#if MISSION_TEST_NO_VISION
                /* 测试: 无 K230, 模拟路径 C(直行靠近) */
                bomb_path_taken = 'C';
                MLOG("[球] 接近方向(测试模拟): C -> 直行靠近");
                if (BOMB_C_APPROACH_MM) Chassis_Move_Forward((int32_t)BOMB_C_APPROACH_MM);
                state_start_tick = HAL_GetTick();
                bomb_sub_state = BOMB_WAIT_INITIAL_MOVE;
#else
                if (Mission_GetNewLine(line, sizeof(line))) {
                    char dir;
                    if (!Vision_IsDirLine(line, &dir)) {
                        /* 忽略 OK/FIRE/D:../SCAN_OK 等无关行, 继续等 C/L/R */
                        MLOG("[球] 忽略无关行: %s (继续等 C/L/R)", line);
                        break;
                    }
                    /* ⭐ 稳定帧: 连续 VISION_STABLE_FRAMES_BOMB 帧同一个方向才真的动车
                     *    —— 单帧误检不动车(这一阶段一动就是 BOMB_*_ADJUST_MM 毫米) */
                    if (!Vision_StableFeed(&s_vs_ball, Vision_TokenOf(line),
                                           VISION_STABLE_FRAMES_BOMB,
                                           VISION_STABLE_TIMEOUT_MS, "球")) {
                        break;   /* 还没稳: 这帧不作数, 继续等 */
                    }
                    g_vision_task_in_progress = 0;
                    bomb_path_taken = dir;
                    MLOG("[球] 接近方向: %c  (L=目标偏画面左 / C=居中 / R=偏右)", bomb_path_taken);
                    /* C/L/R = 目标相对画面中心的横向偏差 → 左右平移补偿
                     * (本车是右移进入排爆区的, 所以不是前进/后退)
                     * ⚠️ 距离宏为 0 时【整条指令都不发】:
                     *    Chassis_Move_*(0) 会把 s_moving 置位、重置 PID、
                     *    重算超时并重新打开航向环, 并不是空操作。 */
                    if (bomb_path_taken == 'C') {
                        if (BOMB_C_APPROACH_MM) Chassis_Move_Forward((int32_t)BOMB_C_APPROACH_MM);
                    } else if (bomb_path_taken == 'L') {
                        if (BOMB_L_ADJUST_MM) Chassis_Move_Left(BOMB_L_ADJUST_MM);
                    } else if (bomb_path_taken == 'R') {
                        if (BOMB_R_ADJUST_MM) Chassis_Move_Right(BOMB_R_ADJUST_MM);
                    }
                    state_start_tick = HAL_GetTick();
                    bomb_sub_state = BOMB_WAIT_INITIAL_MOVE;
                } else if (!g_vision_task_in_progress ||
                           HAL_GetTick() - vision_cmd_tick >= VISION_CMD_RESEND_MS) {
                    Vision_SendTask(K230_TASK_BALL, "BALL", &vision_cmd_tick);
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
                MLOG("[球] 精对准(测试模拟): 已对准");
                Chassis_Stop();
                settle_until = HAL_GetTick() + FINE_TUNE_SETTLE_MS;
                bomb_sub_state = BOMB_SETTLE;
#else
                Vision_StartFineAlign("BALL", &s_align_ball, &state_start_tick, &align_start_time, &cooldown_until);
                bomb_sub_state = BOMB_WAIT_FINE_ALIGN;
#endif
                break;

            case BOMB_WAIT_FINE_ALIGN:
                if (Vision_FineAlignTimeout("BALL", state_start_tick, align_start_time)) {
                    s_bomb_fb_valid = 0;    /* ⭐ 超时=没对准成功, 不做 Y 补偿(防旧值误补) */
                    bomb_sub_state = BOMB_PERFORM_GRAB;
                    break;
                }
                if (Mission_GetNewLine(line, sizeof(line))) {
                    if (Vision_FineAlignProcess(line, "BALL", &cooldown_until, &settle_until,
                                                &s_align_ball)) {
                        BombFbCapture();        /* ⭐ 对准成功: 抓下这一刻的 Y 误差, 抓取时补 ID2 */
                        bomb_sub_state = BOMB_SETTLE;
                    }
                }
                break;

            case BOMB_SETTLE:
                if (HAL_GetTick() >= settle_until) {
                    bomb_sub_state = BOMB_PERFORM_GRAB;
                }
                break;

            case BOMB_PERFORM_GRAB:
                MLOG("阶段: 抓取小球");
                Arm_Start_Bomb_Grab();                 /* 抓夹到小球前 → 夹紧 → 抬起 */
                Arm_GotoPose(ARM_POSE_BUCKET_CARRY);   /* 摆成“携带姿态”, 端着球便于底盘大范围移动 */
                /* 抓完小球后盲走回放置区: 反向平移回去(路径 C 不需要回程) */
                if (bomb_path_taken == 'L') {
                    if (BOMB_L_RETURN_MM) Chassis_Move_Right(BOMB_L_RETURN_MM);
                } else if (bomb_path_taken == 'R') {
                    if (BOMB_R_RETURN_MM) Chassis_Move_Left(BOMB_R_RETURN_MM);
                }
                state_start_tick = HAL_GetTick();
                bomb_sub_state = BOMB_ADJUST_FOR_PLACE;
                break;

            case BOMB_ADJUST_FOR_PLACE:
                /* 路径 C 没有盲走段, 直接摆“看桶姿态”; L/R 等盲走到位后再摆。
                 * 摆完“看桶姿态”才发 run_task:3, 保证 K230 从摄像头里能看到球桶 */
                if (bomb_path_taken != 'L' && bomb_path_taken != 'R') {
                    Arm_GotoPose(ARM_POSE_BUCKET_LOOK);
                    bomb_sub_state = BOMB_REQUEST_BUCKET_DIR;
                    break;
                }
                if (Chassis_Task_Is_Complete() || (HAL_GetTick() - state_start_tick > BLIND_MOVE_TIMEOUT_MS)) {
                    Arm_GotoPose(ARM_POSE_BUCKET_LOOK);
                    bomb_sub_state = BOMB_REQUEST_BUCKET_DIR;
                }
                break;

            /* ---- 排爆第二步: 对准桶(放球) ---- */
            case BOMB_REQUEST_BUCKET_DIR:
#if MISSION_TEST_NO_VISION
                /* 测试: 无 K230, 模拟桶已在正中心(C) → 不补偿, 直接进精对准 */
                bucket_path_taken = 'C';
                MLOG("[桶] 接近方向(测试模拟): C -> 不横移");
                state_start_tick = HAL_GetTick();
                bomb_sub_state = BOMB_WAIT_BUCKET_DIR_MOVE;
#else
                if (Mission_GetNewLine(line, sizeof(line))) {
                    char dir;
                    if (!Vision_IsDirLine(line, &dir)) {
                        MLOG("[桶] 忽略无关行: %s (继续等 C/L/R)", line);
                        break;
                    }
                    /* ⭐ 稳定帧: 连续 3 帧同一个方向才动车(单帧误检不横移) */
                    if (!Vision_StableFeed(&s_vs_bucket, Vision_TokenOf(line),
                                           VISION_STABLE_FRAMES_BOMB,
                                           VISION_STABLE_TIMEOUT_MS, "桶")) {
                        break;   /* 还没稳: 这帧不作数, 继续等 */
                    }
                    g_vision_task_in_progress = 0;
                    bucket_path_taken = dir;
                    MLOG("[桶] 接近方向: %c  (L=目标偏画面左 / C=居中 / R=偏右)", bucket_path_taken);
                    /* ⭐ 桶阶段方向(见文件头“视觉方向映射”):
                     *   机械臂已转到 BUCKET_LOOK(底座≈2243, 比球姿态多转≈180°),
                     *   画面左右相对车体【镜像】, 所以:
                     *     L(画面左) → 小车【右】移
                     *     R(画面右) → 小车【左】移
                     *     C(已居中) → 【不往前走】(车头已抵桶边缘, 再往前会过冲) */
                    if (bucket_path_taken == 'C') {
                        /* 不动, 直接交给精对准 */
                    } else if (bucket_path_taken == 'L') {
                        /* ⭐ 2026-10-11: BUCKET_USE_FINE_ARRIVAL=1 时走"小到位死区"入口
                         *    —— 20mm 的粗对准也让它真的走掉 18mm 左右, 四轮都出力 */
                        if (BUCKET_L_ADJUST_MM) {
                            if (BUCKET_USE_FINE_ARRIVAL) Chassis_Move_RightFine(BUCKET_L_ADJUST_MM);
                            else                         Chassis_Move_Right(BUCKET_L_ADJUST_MM);
                        }
                    } else if (bucket_path_taken == 'R') {
                        if (BUCKET_R_ADJUST_MM) {
                            if (BUCKET_USE_FINE_ARRIVAL) Chassis_Move_LeftFine(BUCKET_R_ADJUST_MM);
                            else                         Chassis_Move_Left(BUCKET_R_ADJUST_MM);
                        }
                    }
                    state_start_tick = HAL_GetTick();
                    bomb_sub_state = BOMB_WAIT_BUCKET_DIR_MOVE;
                } else if (!g_vision_task_in_progress ||
                           HAL_GetTick() - vision_cmd_tick >= VISION_CMD_RESEND_MS) {
                    Vision_SendTask(K230_TASK_BUCKET, "BUCKET", &vision_cmd_tick);
                }
#endif
                break;

            case BOMB_WAIT_BUCKET_DIR_MOVE:
                if (Chassis_Task_Is_Complete() || (HAL_GetTick() - state_start_tick > BLIND_MOVE_TIMEOUT_MS)) {
                    bomb_sub_state = BOMB_REQUEST_BUCKET_FINE;
                }
                break;

            case BOMB_REQUEST_BUCKET_FINE:
#if MISSION_TEST_NO_VISION
                MLOG("[桶] 精对准(测试模拟): 已对准");
                Chassis_Stop();
                settle_until = HAL_GetTick() + FINE_TUNE_SETTLE_MS;
                bomb_sub_state = BOMB_SETTLE_PLACE;
#else
                /* ⭐⭐ 2026-10-11(用户要求): 桶的横向修正分两档 —— 轻微误差改用【底座 ID1】
                 *    修(见 BUCKET_LR_ID1_ENABLE / Bucket_LrFineWithId1)。这里把 ID1
                 *    缓存对齐到 BUCKET_LOOK(基准 40, 落在安全范围 0~40 内)、并复位方向
                 *    自检状态; 偏移清零后重新累计。 */
                Arm_Id1Reset(ARM_POSE_BUCKET_LOOK);
                s_bucket_id1_sign     = (int8_t)BUCKET_LR_ID1_SIGN;
                s_bucket_id1_moved    = 0;
                s_bucket_id1_flipped  = 0;
                s_bucket_id1_last_px  = 0;
                /* ⭐⭐ 2026-10-11: 本次的锁存值先清掉(对准成功那一刻再存, 见
                 *    BOMB_WAIT_BUCKET_ALIGN), 免得用到上一轮的旧值 */
                s_bucket_id1_lock       = 0;
                s_bucket_id1_lock_base  = (int32_t)s_arm_pose_table[ARM_POSE_BUCKET_LOOK][0];
                s_bucket_id1_lock_valid = 0;
                MLOG("[桶] 横向修正: 大误差走底盘(最小步 %dmm), 轻微误差转 ID1 "
                     "(ID1 允许范围 %d~%d, 基准 %d)",
                     (int)BUCKET_LR_CHASSIS_MIN_MM,
                     (int)BUCKET_LR_ID1_POS_MIN, (int)BUCKET_LR_ID1_POS_MAX,
                     (int)s_arm_pose_table[ARM_POSE_BUCKET_LOOK][0]);
                Vision_StartFineAlign("BUCKET", &s_align_bucket, &state_start_tick, &align_start_time, &cooldown_until);
                bomb_sub_state = BOMB_WAIT_BUCKET_ALIGN;
#endif
                break;

            case BOMB_WAIT_BUCKET_ALIGN:
                if (Vision_FineAlignTimeout("BUCKET", state_start_tick, align_start_time)) {
                    s_bomb_fb_valid = 0;    /* ⭐ 超时=没对准成功, 不做 Y 补偿(防旧值误补) */
                    bomb_sub_state = BOMB_PERFORM_PLACE;
                    break;
                }
                if (Mission_GetNewLine(line, sizeof(line))) {
                    if (Vision_FineAlignProcess(line, "BUCKET", &cooldown_until, &settle_until,
                                                &s_align_bucket)) {
                        BombFbCapture();        /* ⭐ 对准成功: 抓下这一刻的 Y 误差 → 放桶时补 ID2 */
                        /* ⭐⭐ 2026-10-11(用户要求): 【锁存 ID1】—— 就在"摄像头对准桶"这一刻
                         *    把底座 ID1 的数值存下来, 之后放球动作直接用它(与救援抓取
                         *    s_rescue_id1_lock 完全同一个做法, 见 Arm_BucketPlaceId1)。
                         *    ⚠️ 对准阶段 ID1 被限幅在 BUCKET_LR_ID1_POS_*(0~40),
                         *       所以锁存值一定落在安全范围内, 不会越界。 */
                        s_bucket_id1_lock       = Arm_Id1GetLogicalPos();
                        s_bucket_id1_lock_valid = 1;
                        MLOG("[桶] 锁存 ID1 = %ld (对准基准 %ld ⇒ 本次对准修正量 %+ld; "
                             "放球动作直接用它 + 手动偏移 %+d, 方式 = %s)",
                             (long)s_bucket_id1_lock, (long)s_bucket_id1_lock_base,
                             (long)(s_bucket_id1_lock - s_bucket_id1_lock_base),
                             (int)BUCKET_PLACE_ID1_OFFSET,
                             BUCKET_PLACE_ID1_USE_LOCK ? "锁存值直接用" : "只搬修正量(保留标定差)");
                        /* ⭐⭐ 2026-10-10(用户要求): 桶改回【抓球那一套】——
                         *    ① 精对准只修左右(BUCKET_USE_SCHEME_2 = 0 / s_align_bucket);
                         *    ② 前后(深度)【不动底盘】, 而是把这帧 Y 误差交给臂: 放桶时
                         *       Arm_Start_Bomb_Place() 里的 Arm_GotoPoseYComp 会按
                         *       BOMB_FB_ARM_ENABLE_BUCKET 把误差换成 ID2 补偿码, 与抓球的
                         *       BALL_PRE/BALL_CLOSE 是同一条路径(只是姿态不同);
                         *    ③ 底盘分档后退(BUCKET_AFTER_ALIGN_BACK_*)保留但关闭 ——
                         *       它和臂补作用在同一个自由度上, 同时开会互相抵消。 */
                        {
                            int32_t fb    = s_bomb_fb_valid ? s_bomb_fb_px : 0;
                            uint8_t is_big = (uint8_t)(labs((long)fb) >
                                                       (long)BUCKET_AFTER_ALIGN_BACK_BIG_PX);
                            int32_t back  = is_big ? BUCKET_AFTER_ALIGN_BACK_BIG_MM
                                                   : BUCKET_AFTER_ALIGN_BACK_SMALL_MM;

#if BUCKET_FB_CL_ENABLE
                            MLOG("[桶] 左右+前后对准完成 (前后误差 %ldpx, 闭环容差 %dpx) -> %s",
                                 (long)fb, (int)BUCKET_FB_CL_TOL_PX,
                                 FbCl_GaveUp() ? "⚠ 前后闭环中途放弃过(见上面 ⚠ 日志)"
                                               : "前后已由闭环修完");
#else
                            MLOG("[桶] 左右对准完成 (前后误差 %ldpx) -> 深度交给臂: %s",
                                 (long)fb,
                                 BOMB_FB_ARM_ENABLE_BUCKET
                                     ? "放桶时按 Y 误差补 ID2(与抓球同款)"
                                     : "⚠ 臂补也关着(BOMB_FB_ARM_ENABLE_BUCKET=0), 前后无人修");
#endif

                            if (BUCKET_AFTER_ALIGN_BACK_ENABLE && (back > 0)) {
                                MLOG("[桶] (老做法)前后误差 %ldpx[%s] (门限 %dpx) "
                                     "-> 后退 %dmm(扣死区后实际约 %dmm) 再停稳放球",
                                     (long)fb, is_big ? "大" : "小",
                                     (int)BUCKET_AFTER_ALIGN_BACK_BIG_PX,
                                     (int)back, (int)(back - CH_DEADZONE_MM));
                                Chassis_Move_Backward(back);
                                state_start_tick = HAL_GetTick();   /* 这一步的兜底超时起点 */
                                bomb_sub_state = BOMB_BUCKET_BACK_OFF;
                            } else {
                                bomb_sub_state = BOMB_SETTLE_PLACE;
                            }
                        }
                    }
                }
                break;

            /* ⭐ 等“对准完之后往后退的那一步”走完 → 停稳 → 放球。
             *    ⚠️ 只有【老做法】才会进到这里(BUCKET_AFTER_ALIGN_BACK_ENABLE = 1);
             *       现在的默认配置(前后走闭环 BUCKET_FB_CL_ENABLE = 1)是
             *       对准完 → 直接 BOMB_SETTLE_PLACE, 不进本状态。
             *    走完会【重新计停稳时间】, 免得边走边放/车还在晃就松爪。 */
            case BOMB_BUCKET_BACK_OFF:
                if (Chassis_Task_Is_Complete() ||
                    (HAL_GetTick() - state_start_tick > BLIND_MOVE_TIMEOUT_MS)) {
                    settle_until = HAL_GetTick() + FINE_TUNE_SETTLE_MS;
                    bomb_sub_state = BOMB_SETTLE_PLACE;
                }
                break;

            case BOMB_SETTLE_PLACE:
                if (HAL_GetTick() >= settle_until) {
                    bomb_sub_state = BOMB_PERFORM_PLACE;
                }
                break;

            case BOMB_PERFORM_PLACE:
                /* 放球: PLACE_PRE(移到放置位) → PLACE_OPEN(夹爪松开) →
                 *       PLACE_LIFT(大臂抬起, 已拆成 ID1/2/3 再 ID4/5 两步防剐蹭) */
                MLOG("阶段: 放置小球");
                Arm_Start_Bomb_Place();
                s_bucket_id1_lock_valid = 0;   /* ⭐ 2026-10-11: 锁存值已用完, 清掉(下次桶对准重新锁) */
                s_bucket_id1_moved      = 0;
                s_bucket_id1_flipped    = 0;
                /* ⭐ 放完球、大臂抬起之后, 直接把臂转到“准备识别靶子”姿态
                 *    (ARM_POSE_TARGET_READY)。之后 STATE_12_PART1_MOVE_A 让小车
                 *    右移; 走完 4 段右移+航向校正, 到 STATE_12_PART1_MOVE_C 才摆成
                 *    TARGET_LOOK。这里是【先摆完停稳、再让车走】(Arm_GotoPose 内部已
                 *    阻塞运动时间+保持时间), 不是车臂并行。 */
                Arm_GotoPose(ARM_POSE_TARGET_READY);
                bomb_sub_state = BOMB_COMPLETE;
                break;

            case BOMB_COMPLETE:
                MLOG("任务1(排爆)完成");
                bomb_sub_state = BOMB_IDLE;
                g_mission_state++;
                break;

            default: break;
        }
    }
    /* ================= 任务2: 打靶 (底盘不动; ID1 修横向, ID4 修竖直) =====
     * 执行到本函数时: 小车已走完 “4 段右移(每段之间航向校正) → 最后一次校正 + 停稳”,
     *                 臂已摆到 ARM_POSE_TARGET_LOOK(摄像头对准靶子)
     *
     * 两个轴的分工(2026-10-06 新增竖直轴):
     *     水平误差 dx  → 底座 ID1 小步转 (TARGET_ID1_STEP 码/步)
     *     竖直误差 dy  → 腕部 ID4 小步转 (TARGET_ID4_STEP 码/步)
     *     两个轴【交替】修: K230 每帧只发绝对值大的那个轴上的 D, 我们收到哪个修哪个;
     *     同时修会互相干扰(转腕部会带动画面横向), 所以一次只动一个舵机。
     *     ⚠️ 为什么要修 dy: K230 判“对准成功”要求 |dx|<50 且 |dy|<50,
     *        只有它判成功才会 fire_start() 点激光。只修 dx 的话 K230 永远等不到成功。
     *
     *   ① TARGET_IDLE        发 run_task:2 → K230 回 C / L / R
     *                        (run_task 只在“还没收到过任何回应”前每 1s 重发,
     *                         收到回应后就不再打扰 K230)
     *                        L → 底座 ID1 向【左】转 TARGET_COARSE_STEP 码
     *                        R → 底座 ID1 向【右】转 TARGET_COARSE_STEP 码
     *                        (⭐ 2026-10-08: 原来固定 60 码 ≈ 107px 一步, 比 K230 的
     *                         ±40px 窗口还大 ⇒ 在窗口两边来回摆、永不收敛;
     *                         现在常规 30 码 ≈ 53px, 并且检测到方向翻转时换细步 12 码)
     *                        C → 粗对准完成, 按下页 TARGET_USE_FINE_ALIGN 选择:
     *                            =1(当前): 发 start_align 进 ②/③ 精对准
     *                            =0       : 直接认为对准, 去 TARGET_PERFORM
     *                        ⚠️ 全程【一条底盘指令都不发】, 小车原地不动
     *                        ⚠️ 这里【不把 g_vision_task_in_progress 清 0】:
     *                           因为要反复回到本状态, 清 0 会导致每次都走
     *                           “Vision Start”分支(重发 run_task + 清空队列)
     *   ② TARGET_ID1_MOVING  等粗对准那一步走完(TARGET_ID1_MOVE_MS)。期间把 K230
     *                        攒下的行【全部丢弃】, 保证下一帧看到的是“转完之后”
     *                        的画面 —— 否则会拿转之前的旧误差连续累加 → 冲过头
     *   ③ TARGET_FINE_IDLE   等 K230 回 D:<x>,<y> / OK:
     *                        |x| ≥ 容差 → ID1 转一步; 否则 |y| ≥ 容差 → ID4 转一步
     *                        (x>0 = 目标偏画面左, 与接近阶段的 'L' 同向;
     *                         y>0 = 目标偏画面下, 方向由 TARGET_ID4_DY_SIGN 统管)
     *                        收到 OK 即完成 —— K230 在发射激光时会先发一行 "FIRE"
     *                        ⭐ 必须发 start_align 让 K230 进入 ALIGN 状态,
     *                           它才会点激光(yolo_main.py 只在 ALIGN 里 fire_start)
     *   ④ TARGET_FINE_MOVING 等刚才那一步舵机走完(target_fine_move_ms: ID1 与 ID4 不同),
     *                        期间的旧帧丢掉; ⚠️ 但里面的 OK/FIRE 不能丢(只发一次)
     *                        ⭐⭐ 2026-10-10 【精对准稳定性】(详见 TARGET_FINE_* 宏):
     *                           · 进 ③ 之前先静置 TARGET_FINE_START_SETTLE_MS(借本状态丢帧);
     *                           · 本状态的等待 = 转动时间 + TARGET_FINE_SETTLE_MS;
     *                           · 微动改走 Servos_SetPositionsMaskedEx() 的柔性速度
     *                             (TARGET_FINE_SPEED_MIN), 不再被通用下限
     *                             SERVO_SPEED_MIN(200 步/秒)抽成 25ms 的“抽搐”;
     *                           · 误差符号翻转(上一步跨过中心) ⇒ 换微步 *_STEP_MICRO。
     *   ⑤ TARGET_PERFORM     收到 OK 后的收尾【不再摆发射位】, 只做两段:
     *                          1) 抬起大臂 → ARM_POSE_TARGET_LIFT
     *                          2) 手臂收回 → ARM_POSE_SCAN_RESET
     *                          每段都阻塞“该姿态运动时间 + 保持时间”
     *                          (激光由 K230 在判成功那一刻自己点亮, 而判成功时
     *                           摄像头就正对靶心 = 当前的 TARGET_LOOK 姿态)
     *   ⑥ TARGET_COMPLETE    回主状态机 → 右移 ROUTE_12_P2_A_MM → 航向校正 → 救援
     *
     * 🛡 防卡死: 粗对准转满 TARGET_ID1_STEP_MAX 步 / 超过 TARGET_ID1_TIMEOUT_MS,
     *            或精对准横向满 TARGET_ID1_STEP_MAX 步、竖直满 TARGET_ID4_STEP_MAX 步
     *            或总时长超限仍没收到 OK (含 K230 完全不回应)
     *            → 强制当作已对准, 保证能往下走
     */

    else if (expected_task_number == 2) {
        switch (target_sub_state) {
            case TARGET_IDLE:
#if MISSION_TEST_NO_VISION
                /* 测试: 无 K230, 模拟“已对准” → 直接抬臂收尾 */
                MLOG("[靶] 接近方向(测试模拟): C -> 直接抬臂收尾");
                target_sub_state = TARGET_PERFORM;
#else
                /* 本次打靶第一次进来: 复位计数/计时, 并把 ID1 缓存对齐到 TARGET_LOOK
                 * (此时 STATE_12_PART1_MOVE_C 已经执行过 Arm_GotoPose(TARGET_LOOK)) */
                if (!target_id1_inited) {
                    target_id1_inited = 1;
                    target_heard      = 0;
                    target_id1_steps  = 0;
                    target_id1_tick   = HAL_GetTick();
                    target_fine_steps = 0;
                    target_fine_tick  = 0;
                    target_fine_cmd_tick = 0;
                    target_fine_xok_warned = 0;
                    target_freeze_warned   = 0;
                    target_fire_tick  = 0;
                    target_fine_move_ms = TARGET_ID1_MOVE_MS;
                    target_id4_steps = 0;
                    target_last_dy_valid = 0;
                    target_last_dy_step  = 0;
                    target_lost_cnt    = 0;     /* ⭐ 2026-10-11: 丢靶自恢复状态复位 */
                    target_lost_tick   = 0;
                    target_lost_dir    = 1;
                    target_fine_d_seen = 0;
                    target_coarse_dir  = 0;    /* ⭐ 粗对准过冲检测复位 */
                    target_coarse_flip = 0;
                    Arm_Id1Reset(ARM_POSE_TARGET_LOOK);    /* 横向轴(ID1)基准 */
                    Arm_Id4Reset(ARM_POSE_TARGET_LOOK);    /* ⭐ 竖直轴(ID4)基准 */
                    MLOG("[靶] 对准开始: 基准位置 ID1=%d, ID4=%d",
                         (int)s_id1_pos, (int)s_id4_pos);
                }

                /* 🛡防卡死: 步数或时长任一超限 → 强制走 C 流程 */
                if (target_id1_steps >= TARGET_ID1_STEP_MAX ||
                    (HAL_GetTick() - target_id1_tick) > TARGET_ID1_TIMEOUT_MS) {
                    MLOG("[靶] 底座ID1粗对准强制结束(已转%u步, 用时%lums, 距上次收到K230数据 %lums) -> 当作已对准",
                         (unsigned)target_id1_steps,
                         (unsigned long)(HAL_GetTick() - target_id1_tick),
                         (unsigned long)K230_RxSilenceMs());
                    target_sub_state = TARGET_PERFORM;
                    break;
                }

                if (Mission_GetNewLine(line, sizeof(line))) {
                    char dir;
                    if (!Vision_IsDirLine(line, &dir)) {
                        /* 忽略 OK / FIRE / D:.. / SCAN_OK 等无关行, 继续等 C/L/R */
                        MLOG("[靶] 忽略无关行: %s (继续等 C/L/R)", line);
                        break;
                    }
                    target_heard = 1;   /* 已与 K230 建立联系: 之后不再定时重发 */
                    target_path_taken = dir;
                    MLOG("[靶] 接近方向: %c  (L=目标偏画面左 / C=居中 / R=偏右)", target_path_taken);
                    /* 画面偏差 → 底座 ID1 小步偏转(车不动):
                     *   L → ID1 数值减小(向左) ; R → ID1 数值增大(向右)
                     * (方向由 TARGET_ID1_LR_SIGN 统管, 实测反了只改那个宏)
                     * ⭐⭐ 2026-10-08: 步长改用 TARGET_COARSE_STEP(30 码 ≈ 53px),
                     *   不再用 60 码(≈107px) —— 那是“粗定位不收敛”的根因:
                     *   一步跨过整个 ±40px 窗口 ⇒ 只会在两边来回摆。
                     *   另外做“过冲检测”: 方向翻转 = 上一步跨过了中心 ⇒ 换细步。 */
                    if (target_path_taken == 'L' || target_path_taken == 'R') {
                        int8_t  dir_sign = (target_path_taken == 'L') ? 1 : -1;
                        int32_t step     = TARGET_COARSE_STEP;
                        if (target_coarse_dir != 0 && dir_sign != target_coarse_dir) {
                            target_coarse_flip++;
                            step = TARGET_COARSE_STEP_MIN;
                            MLOG("[靶] 粗对准: ⚠ 方向翻转(第%u次, 上一步跨过中心) -> 改用细步 %ld 码",
                                 (unsigned)target_coarse_flip, (long)step);
                        }
                        target_coarse_dir = dir_sign;

                        Target_Id1CoarseStep((int32_t)dir_sign *
                                             (int32_t)TARGET_ID1_LR_SIGN * step);
                        MLOG("[靶] 粗对准: 转 ID1 %+ld 码 (%c, 第%u步)",
                             (long)((int32_t)dir_sign * (int32_t)TARGET_ID1_LR_SIGN * step),
                             target_path_taken, (unsigned)(target_id1_steps + 1u));
                        target_id1_steps++;
                        target_id1_move_tick = HAL_GetTick();
                        target_sub_state = TARGET_ID1_MOVING;
                    } else {
                        /* 'C': 粗对准完成 */
#if TARGET_USE_FINE_ALIGN
                        /* ⭐ 发 start_align 让 K230 进入 ALIGN 状态做精对准。
                         *    只有走这一步, K230 对准后才会 "FIRE"(点激光) + "OK"。
                         *    先 flush 掉接近阶段残留的 C/L/R, 免得被当成精对准结果 */
                        MLOG("[靶] 收到C(共粗转%u步) -> 发 start_align 进入精对准",
                             (unsigned)target_id1_steps);
                        target_fine_steps = 0;
                        target_fine_tick  = HAL_GetTick();
                        target_fine_cmd_tick = HAL_GetTick();
                        target_prev_ex_valid = 0;   /* ⭐ 方向翻转检测: 新一轮从头开始记 */
                        target_prev_ey_valid = 0;
                        /* ⭐ 2026-10-11: 丢靶自恢复也从头开始 —— 计时从"开始等它进 ALIGN"
                         *    起算, 但"是否丢靶"还要等收到过 D 帧才算(target_fine_d_seen)。 */
                        target_lost_cnt    = 0;
                        target_lost_tick   = HAL_GetTick();
                        target_lost_dir    = 1;
                        target_fine_d_seen = 0;
                        K230_Start_Align();
                        K230_FlushAll();
#if TARGET_FINE_START_SETTLE_MS > 0
                        /* ⭐⭐【稳定性③】刚停下(扫描/粗转刚结束)整条手臂还在摆动,
                         *    这一段先静置, 借 TARGET_FINE_MOVING 的“丢帧”逻辑把期间
                         *    收到的 D 帧全部丢掉 —— 这样第一次微调用的是“稳下来之后”
                         *    的画面, 不会一开始就按抖动的误差去转(越转越晃)。 */
                        MLOG("[靶] 进精对准先静置 %dms(丢掉抖动期的帧) 再开始微调",
                             (int)TARGET_FINE_START_SETTLE_MS);
                        target_fine_move_ms  = TARGET_FINE_START_SETTLE_MS;
                        target_id1_move_tick = HAL_GetTick();
                        target_sub_state = TARGET_FINE_MOVING;
#else
                        target_sub_state = TARGET_FINE_IDLE;
#endif
#else
                        MLOG("[靶] 收到C(共粗转%u步) -> 直接摆发射位(未开精对准)",
                             (unsigned)target_id1_steps);
                        target_sub_state = TARGET_PERFORM;
#endif
                    }
                } else if (!target_heard &&
                           (!g_vision_task_in_progress ||
                            HAL_GetTick() - vision_cmd_tick >= VISION_CMD_RESEND_MS)) {
                    Vision_SendTask(K230_TASK_TARGET, "TARGET", &vision_cmd_tick);
                }
#endif
                break;

            case TARGET_ID1_MOVING:
                /* 等 ID1 转完。期间把 K230 攒下的旧行【全部丢掉】(故意不保留最新),
                 * 这样回到 TARGET_IDLE 时拿到的一定是“转完之后”采集的新帧 */
                while (Mission_GetNewLine(line, sizeof(line))) {
                    /* 丢弃旧帧, 防止拿旧误差连续累加 */
                }
                if ((HAL_GetTick() - target_id1_move_tick) >= TARGET_ID1_MOVE_MS) {
                    target_sub_state = TARGET_IDLE;
                }
                break;

            /* ⚠️ 下面【两个状态是纯视觉的】, 必须和 !MISSION_TEST_NO_VISION 同条件:
             *    它们要用 Target_Id1StepFor / Target_Id1Step / Target_Id4StepFor /
             *    Target_Id4Step 和 Arm_Id1Reset/Arm_Id4Reset —— 而这整套“原地只转
             *    底座/腕部”的工具函数都写在 MissionControl.c 上方的
             *    `#if !MISSION_TEST_NO_VISION` 块里。
             *    不包起来的话, 把 MISSION_TEST_NO_VISION 置 1(纯底盘测试模式)时会报:
             *      error: implicit declaration of function 'Target_Id1StepFor'
             *    (NO_VISION=1 时 TARGET_IDLE 直接跳 TARGET_PERFORM → 这两个状态根本到不了) */
#if !MISSION_TEST_NO_VISION
            case TARGET_FINE_IDLE:
                /* 已发 start_align, 等 K230 的 D:<x>,<y> / OK。
                 * ⭐⭐ 2026-10-11(对齐 K230 新代码 yolo_main.py):
                 *   · 偏了就发 D:<x>,<y> —— 【两个轴都报真实值】(旧版只报误差大的那一轴);
                 *     节奏: 每帧至少隔 ALIGN_SEND_IVL(1.0s), 发完静默 ALIGN_SETTLE(1.2s),
                 *           若两帧误差完全相同则要等 ALIGN_RETRY_IVL(3.0s) 才重发。
                 *   · 判成功 = |dx| 和 |dy| 都 < TARGET_ALIGN_TOL_PIX(11px) 且连续稳定
                 *     0.4s ⇒ 那一刻它先 fire_start() 点激光 + 发 "FIRE", 紧接着发 "OK"。
                 *   · 它一直对不上时: ALIGN_SELF_TIMEOUT(14s) 自超时 —— 也是【先发射
                 *     再发 FIRE+OK】(⚠️ 和旧版不同: 旧版超时只回 OK、不点激光)。 */
                /* ⭐ K230 发完 FIRE 后既不回 OK、也不再发误差 —— 给它宽限时间,
                 * 到点就当打靶完成, 免得白等到 TARGET_ID1_TIMEOUT_MS(20s) 才收尾。 */
                if (target_fire_tick != 0 &&
                    (HAL_GetTick() - target_fire_tick) >= K230_FIRE_GRACE_MS) {
                    MLOG("[靶] K230 发过 FIRE 但 %dms 内没等到 OK -> 当作打靶完成",
                         (int)K230_FIRE_GRACE_MS);
                    target_sub_state = TARGET_HOLD;
                    break;
                }

                /* 🛡防卡死: ID1/ID4 步数或总时长任一超限 -> 强制走 C 流程 */
                if (target_fine_steps >= TARGET_ID1_STEP_MAX ||
                    target_id4_steps >= TARGET_ID4_STEP_MAX ||
                    (HAL_GetTick() - target_fine_tick) > TARGET_ID1_TIMEOUT_MS) {
                    MLOG("[靶] 精对准强制结束(横向%u步/竖直%u步, 用时%lums, 距上次收到K230数据 %lums) -> 抬臂收尾",
                         (unsigned)target_fine_steps, (unsigned)target_id4_steps,
                         (unsigned long)(HAL_GetTick() - target_fine_tick),
                         (unsigned long)K230_RxSilenceMs());
                    target_sub_state = TARGET_PERFORM;
                    break;
                }
                if (Mission_GetNewLine(line, sizeof(line))) {
                    if (strncmp(line, "FIRE", 4) == 0) {
                        /* ⭐⭐ 2026-10-11(用户实测反馈: "摄像头瞄准靶心之后 ID4 还在抖, 激光出靶子")：
                         *  ⚠️ 原来收到 FIRE 只是记个时间戳, 【继续处理后面的 D 帧】⇒ 激光已经亮着
                         *     的那 1.5s(K230_FIRE_GRACE_MS)里还在一步几码地转 ID4/ID1 ⇒
                         *     机械臂和腕部的余振把激光点晃出靶心。
                         *  FIRE = K230 已经判成功并点亮激光 ⇒ 这一刻【立刻冻结姿态】:
                         *     转到 TARGET_HOLD(只等、不给舵机发任何指令), 3s 后照常抬臂收尾。
                         *  ⚠️ 不要再回到"继续等 D 帧微调"——那正是激光打偏的原因。
                         *  target_fire_tick 仍记下来, 日志/宽限逻辑照旧。 */
                        if (target_fire_tick == 0) {
                            target_fire_tick = HAL_GetTick();
                        }
                        MLOG("[靶] K230 已发射激光(FIRE) -> 【立刻冻结姿态】保持 %dms 后抬臂 "
                             "(收到 FIRE 后不再动 ID1/ID4, 免得把激光晃出靶子)",
                             (int)TARGET_FIRE_HOLD_MS);
                        target_sub_state = TARGET_HOLD;
                    } else if (strncmp(line, "OK", 2) == 0) {
                        MLOG("[靶] 精对准完成(收到OK, 共微调%u步) -> 保持姿态 %dms 后抬臂",
                             (unsigned)target_fine_steps, (int)TARGET_FIRE_HOLD_MS);
                        target_sub_state = TARGET_HOLD;
                    } else {
                        char *pD = strstr(line, "D:");
                        if (pD != NULL) {
                            char *pComma = strchr(pD, ',');
                            int err_x = atoi(pD + 2);
                            int err_y = (pComma != NULL) ? atoi(pComma + 1) : 0;

                            target_fine_d_seen = 1;   /* ⭐ 2026-10-11: 摄像头能看到 ⇒ 之后"没数据"才算丢靶 */

                            /* ⭐ 两个轴分工: 横向(ID1 底座) / 竖直(ID4 腕部)。
                             *    K230 新代码【每帧两个轴都报真实值】(见上面 fix_axis 的说明),
                             *    所以下面按 fix_axis 决定"这一帧动哪一个" —— 一次只动一个舵机,
                             *    因为转腕部(ID4)会同时影响画面横向(横滚耦合), 同时发会互相干扰。
                             * ⚠️ 容差用 TARGET_ALIGN_TOLERANCE(10), 与 K230 的 11px 窗口对齐 */
                            /* ⭐⭐ 2026-10-11: 【冻结带】—— 两轴都已经小到 K230 能判成功
                             *    (见 TARGET_STOP_NUDGE_TOL_PX 的取值说明)就【不再动舵机】,
                             *    只等它的 OK/FIRE。为什么要: 在靶心附近还一步几码地"蹭",
                             *    舵机/腕部的余振会一直存在, 激光点就跟着晃 ——
                             *    实测"瞄准靶心之后 ID4 还在抖, 激光出靶子"。 */
                            uint8_t freeze_here =
                                (uint8_t)(TARGET_STOP_NUDGE_TOL_PX > 0 &&
                                          abs(err_x) < (int)TARGET_STOP_NUDGE_TOL_PX &&
                                          abs(err_y) < (int)TARGET_STOP_NUDGE_TOL_PX);

                            /* ⭐⭐ 2026-10-11(对齐 K230 新代码): 【本帧该修哪一轴】
                             *   新 yolo_main.py 每帧把【两个轴都报真实值】(它自己注释里写的
                             *   "★ 两轴都报真实值(原来只报误差大的那一轴, 另一轴填 0)"),
                             *   所以老的"先横向、后竖直"固定顺序不再合适 —— 两轴都超窗时
                             *   纵向永远排不上队(每帧都被横向那条分支吃掉)。
                             *   改成:
                             *     两轴都超窗 → 先修【误差大的】那一轴(收敛快、少走步数);
                             *     只有一轴超窗 → 只动那一轴, 不去碰已在窗口内的轴
                             *                     (多动一下 = 多晃一下激光)。
                             *   ⚠️ 一次只动一个舵机: 转 ID4(腕部)会同时改变画面的横向
                             *      (横滚耦合), 两条指令同时发会互相干扰, 所以不能并行修两轴。 */
                            uint8_t fix_axis;   /* 0 = 冻结; 1 = 动 ID1(横向); 2 = 动 ID4(竖直) */
                            if (freeze_here) {
                                fix_axis = 0;
                            } else if (abs(err_x) >= TARGET_ALIGN_TOLERANCE &&
                                       abs(err_x) >= abs(err_y)) {
                                fix_axis = 1;
                            } else if (abs(err_y) >= TARGET_ALIGN_TOLERANCE) {
                                fix_axis = 2;
                            } else {
                                fix_axis = 1;   /* 只剩横向超窗(y 已在窗口内) */
                            }

                            if (fix_axis == 0) {
                                if (!target_freeze_warned) {
                                    target_freeze_warned = 1;
                                    MLOG("[靶] 两轴已进【冻结带】±%dpx (x=%d, y=%d) -> 不再动 ID1/ID4, "
                                         "冻结姿态让 K230 连判成功(它要 |dx|,|dy| 都 <%dpx 且稳定 0.4s)后打激光; "
                                         "若它一直不回 OK/FIRE, 就是它那边判不过 —— 见 TARGET_STOP_NUDGE_TOL_PX",
                                         (int)TARGET_STOP_NUDGE_TOL_PX, err_x, err_y,
                                         (int)TARGET_ALIGN_TOLERANCE);
                                }
                            } else if (fix_axis == 1) {
                                /* 画面偏左(err_x>0) 等价于接近阶段的 'L', 方向一致。
                                 * ⭐ 步长按误差大小自适应(见 Target_Id1StepFor) ——
                                 *    一律用大步会在容差窗口里来回摆(窗口现在只有
                                 *    TARGET_ALIGN_TOLERANCE=10px)。 */
                                int32_t d1 = Target_Id1StepFor(err_x);
                                uint8_t flip_x = 0;
#if TARGET_FINE_MICRO_ENABLE
                                /* ⭐⭐【稳定性④】方向翻转 = 上一步跨过了画面中心
                                 *    (本次误差符号与上一次横向决策相反) ⇒ 改用微步,
                                 *    免得在窗口两边来回摆(肉眼就是“手臂一直在晃”)。 */
                                flip_x = (uint8_t)(target_prev_ex_valid &&
                                                   ((err_x > 0) != (target_prev_ex > 0)));
                                if (flip_x) {
                                    d1 = (err_x > 0)
                                             ? ((int32_t)TARGET_ID1_STEP_MICRO * (int32_t)TARGET_ID1_LR_SIGN)
                                             : (-(int32_t)TARGET_ID1_STEP_MICRO * (int32_t)TARGET_ID1_LR_SIGN);
                                    MLOG("[靶] 精对准[横向]: ⚠ 方向翻转(上一步跨过中心, %dpx -> %dpx) "
                                         "-> 改用微步 %ld 码",
                                         (int)target_prev_ex, err_x, (long)d1);
                                }
#endif
                                target_prev_ex       = err_x;
                                target_prev_ex_valid = 1;

                                MLOG("[靶] 精对准[横向] x=%dpx (x>0=偏左) -> ID1 转 %+ld 码 [%s]",
                                     err_x, (long)d1,
                                     flip_x ? "微步" :
                                     (abs(err_x) > TARGET_ID1_STEP_BIG_PX) ? "大步" :
                                     (abs(err_x) > TARGET_ID1_STEP_MID_PX) ? "中步" : "小步");
                                Target_Id1Step(d1);
                                target_fine_steps++;
                                /* ⭐【稳定性②】本步走完再多静置 TARGET_FINE_SETTLE_MS
                                 *   (期间 K230 的 D 帧全丢) ⇒ 等手臂余振衰减、画面稳了
                                 *   再决策; 0 = 回老行为(走完立刻看下一帧) */
                                target_fine_move_ms  = (uint16_t)(TARGET_ID1_MOVE_MS +
                                                                   TARGET_FINE_SETTLE_MS);
                                target_id1_move_tick = HAL_GetTick();
                                target_sub_state = TARGET_FINE_MOVING;
                            }
#if TARGET_FIX_DY_WITH_ID4
                            else if (fix_axis == 2) {
                                /* ⚠️ 走到这里 = 两轴都超窗且竖直误差更大, 或只有竖直超窗
                                 *    (见上面 fix_axis 的说明) */
                                /* ⭐ 竖直: y>0 = 目标偏画面下, 用 ID4(腕部) 修。
                                 *    ⚠️ 方向由 TARGET_ID4_DY_SIGN 决定(见宏注释), 实测反了就取反。
                                 *    ⭐ 步长按误差大小自适应(见 Target_Id4StepFor):
                                 *       |y| 大 → TARGET_ID4_STEP_FAR(快); 否则 → TARGET_ID4_STEP。 */
                                int32_t d4 = Target_Id4StepFor(err_y);
                                uint8_t flip_y = 0;
#if TARGET_FINE_MICRO_ENABLE
                                /* ⭐⭐【稳定性④】方向翻转 = 上一步跨过中心 ⇒ 换微步
                                 *   (与横向同一套判断, 见 TARGET_FINE_MICRO_ENABLE) */
                                flip_y = (uint8_t)(target_prev_ey_valid &&
                                                   ((err_y > 0) != (target_prev_ey > 0)));
                                if (flip_y) {
                                    d4 = (err_y > 0)
                                             ? ((int32_t)TARGET_ID4_STEP_MICRO * (int32_t)TARGET_ID4_DY_SIGN)
                                             : (-(int32_t)TARGET_ID4_STEP_MICRO * (int32_t)TARGET_ID4_DY_SIGN);
                                    MLOG("[靶] 精对准[竖直]: ⚠ 方向翻转(上一步跨过中心, %dpx -> %dpx) "
                                         "-> 改用微步 %ld 码",
                                         (int)target_prev_ey, err_y, (long)d4);
                                }
#endif
                                target_prev_ey       = err_y;
                                target_prev_ey_valid = 1;

                                /* 标定 ID4 步长: 看“上一步 ID4 转了多少码、y 变了多少 px”。
                                 * ⚠️ 步长是自适应的, 所以必须打【上一步实际转的码数】,
                                 *    打宏值会把“px/码”算错 */
                                if (target_last_dy_valid && (target_last_dy != err_y)) {
                                    MLOG("[靶][竖直轴变化] 上一步 ID4 转 %ld 码后, |y| %ldpx -> %dpx "
                                         "(变化 %+ldpx; 负=变小=方向对) —— 据此调 TARGET_ID4_STEP",
                                         (long)target_last_dy_step, labs((long)target_last_dy), abs(err_y),
                                         labs((long)target_last_dy) - (long)abs(err_y));
                                }
                                target_last_dy       = err_y;
                                target_last_dy_valid = 1;
                                target_last_dy_step  = (d4 >= 0) ? d4 : -d4;

                                MLOG("[靶] 精对准[竖直] y=%dpx (y>0=偏下) -> ID4 转 %+ld 码 [%s]",
                                     err_y, (long)d4,
                                     flip_y ? "微步" :
                                     (abs(err_y) > TARGET_ID4_STEP_FAR_PX) ? "大步" :
                                     (abs(err_y) > TARGET_ID4_STEP_FINE_PX) ? "中步" : "细步");
                                Target_Id4Step(d4);
                                target_id4_steps++;
                                /* ⭐【稳定性②】本步走完再多静置一段时间(丢帧), 见横向那处说明 */
                                target_fine_move_ms  = (uint16_t)(TARGET_ID4_MOVE_MS +
                                                                   TARGET_FINE_SETTLE_MS);
                                target_id1_move_tick = HAL_GetTick();
                                target_sub_state = TARGET_FINE_MOVING;
                            }
#endif
                            else if (!target_fine_xok_warned) {
                                /* ⚠️ 正常【走不到这里】: 上面 fix_axis 已经把"冻结 / 修横向 /
                                 *    修竖直"三种情况分完了。只有把 TARGET_FIX_DY_WITH_ID4
                                 *    关掉(=0, 竖直不修)时, "只有 y 超窗"才会落到这里 ——
                                 *    只提示一次, 免得刷屏。 */
                                target_fine_xok_warned = 1;
                                MLOG("[靶] 两轴都已在容差(%dpx)内 (x=%d, y=%d), 等 K230 回 OK/FIRE; "
                                     "若一直不回, 就是 K230 那边判不通过(它现在要求 |dx| 和 |dy| 都 <%dpx)",
                                     (int)TARGET_ALIGN_TOLERANCE, err_x, err_y,
                                     (int)TARGET_ALIGN_TOLERANCE);
                            }
                        } else {
                            char c = line[0];
                            if (c == 'C' || c == 'L' || c == 'R') {
                                /* K230 还在发接近阶段的 C/L/R → 说明它还没切进
                                 * ALIGN 状态(例如 start_align 在路上被丢了)。
                                 * 按 1s 节流重发一次, 别干等到 20s 超时 */
                                if ((HAL_GetTick() - target_fine_cmd_tick) >= VISION_CMD_RESEND_MS) {
                                    MLOG("[靶] K230 还在发接近指令(未进对准模式), 重发 start_align");
                                    K230_Start_Align();
                                    target_fine_cmd_tick = HAL_GetTick();
                                }
                            } else {
                                MLOG("[靶] 忽略无关行: %s", line);
                            }
                        }
                    }
                }

                /* ⭐⭐ 2026-10-11(用户要求): 【丢靶自恢复】—— 见 TARGET_LOST_SEARCH_ENABLE
                 *    的说明。放在本状态最后: 上面若刚处理过一行(说明 K230 在发数据),
                 *    静默时间自然很小, 不会误触发。
                 *    ID1 一直在跟着画面转, 转到某步之后 K230 看不到靶子了(一行都不发),
                 *    这时让 ID4 上下动一下就把它重新收进识别范围, 然后继续等它的信号。 */
#if TARGET_LOST_SEARCH_ENABLE
                if (target_fire_tick == 0 &&                 /* K230 发过 FIRE 就会故意停发, 不算丢靶 */
                    target_fine_d_seen &&                    /* 本来能看到(D 帧来过) */
                    (target_id1_steps + target_fine_steps) >= (uint32_t)TARGET_LOST_MIN_ID1_STEPS &&
                    target_lost_cnt < (uint16_t)TARGET_LOST_SEARCH_MAX &&
                    K230_RxSilenceMs() >= (uint32_t)TARGET_LOST_SILENCE_MS &&
                    (HAL_GetTick() - target_lost_tick) >= (uint32_t)TARGET_LOST_SEARCH_GAP_MS) {
                    int32_t d4 = (int32_t)TARGET_LOST_ID4_STEP * (int32_t)target_lost_dir;

                    target_lost_dir = (int8_t)(-target_lost_dir);   /* 一次上抬、一次下低交替 */
                    target_lost_cnt++;
                    target_lost_tick = HAL_GetTick();
                    MLOG("[靶] 丢靶自恢复(%u/%d): 距上次收到 K230 数据 %lums(ID1 已转 %u 步, "
                         "yaw 无关) -> ID4 %s %ld 码去找靶, 然后继续等它的信号",
                         (unsigned)target_lost_cnt, (int)TARGET_LOST_SEARCH_MAX,
                         (unsigned long)K230_RxSilenceMs(),
                         (unsigned)(target_id1_steps + target_fine_steps),
                         (d4 > 0) ? "往上抬" : "往下低", (long)d4);
                    Target_Id4Step(d4);
                    target_fine_move_ms  = (uint16_t)(TARGET_ID4_MOVE_MS + TARGET_FINE_SETTLE_MS);
                    target_id1_move_tick = HAL_GetTick();
                    target_sub_state = TARGET_FINE_MOVING;
                }
#endif
                break;

            case TARGET_FINE_MOVING:
                /* 等刚才那一步舵机走完(target_fine_move_ms: ID1 与 ID4 的转动时间不同),
                 * 期间的旧帧丢掉(否则会拿“转之前”的旧误差连续累加);
                 * ⚠️ 但 OK/FIRE 不能丢 —— K230 对准成功后会只发一次,
                 *    丢了就只能等到精对准超时(20s)才收尾 */
                {
                    uint8_t ok_seen = 0;
                    uint8_t fire_seen = 0;
                    while (Mission_GetNewLine(line, sizeof(line))) {
                        if (strncmp(line, "FIRE", 4) == 0) {
                            fire_seen = 1;
                            ok_seen = 1;
                        } else if (strncmp(line, "OK", 2) == 0) {
                            ok_seen = 1;
                        }
                    }
                    if (ok_seen) {
                        /* ⭐⭐ 2026-10-11: 在【舵机还在走】的时候就收到 FIRE/OK —— 说明 K230
                         *    是在我们这一步动作【还没走完】时判的成功。这一步的剩余行程我们
                         *    已经发出去了, 没法撤, 腕部/底座还要再动完这一段 ⇒ 激光跟着晃。
                         *    ⚠️ 正常不该出现: K230 要"连续 2 帧(≥0.4s)都在 ±11px 内"才判成功,
                         *       而我们每步只给 400~600ms + 静置 250ms。真出现这行就说明
                         *       ① 步长太大, 走到半路就已经进了 K230 的窗口, 或者
                         *       ② K230 判成功时我们刚好在动。
                         *    对策(按优先级): 把 TARGET_ID4_STEP_FINE / TARGET_ID1_STEP_FINE
                         *       调小, 或把 TARGET_FINE_SETTLE_MS 加大, 让它"停稳了再判"。 */
                        MLOG("[靶] ⚠ 舵机还没走完就收到 %s -> 保持姿态 %dms 后抬臂 "
                             "(这一步的剩余行程会让激光晃一下; 常出现就把 *_STEP_FINE 调小)",
                             fire_seen ? "FIRE" : "OK", (int)TARGET_FIRE_HOLD_MS);
                        target_sub_state = TARGET_HOLD;
                    } else if ((HAL_GetTick() - target_id1_move_tick) >= target_fine_move_ms) {
                        target_sub_state = TARGET_FINE_IDLE;
                    }
                }
                break;
#endif /* !MISSION_TEST_NO_VISION (TARGET_FINE_IDLE / TARGET_FINE_MOVING) */

            case TARGET_HOLD:
                /* ⭐ 收到 K230 的 OK 之后, 【保持当前对准姿态】TARGET_FIRE_HOLD_MS,
                 * 让 K230 的激光稳稳打在靶上, 然后再抬臂收工。
                 * 为什么不能收到 OK 就抬: 激光是 K230 在它判成功那一刻自己点亮的,
                 *   而抬臂会立刻改变摄像头/激光朝向 → 激光就打偏/只闪一下。
                 * 用 Mission_Coop_Wait 等这 3 秒: 期间照刷陀螺仪 yaw、照收 K230 行,
                 * 且【不给舵机发任何指令】= 姿态真的保持不动。 */
                MLOG("[靶] 保持当前对准姿态 %dms (让激光打在靶上), 之后抬臂收尾",
                     (int)TARGET_FIRE_HOLD_MS);
                Mission_Coop_Wait(TARGET_FIRE_HOLD_MS);
                target_sub_state = TARGET_PERFORM;
                break;

            case TARGET_PERFORM:
                /* 收到 K230 的 OK 之后直接【抬大臂 → 收回手臂】, 不再摆“激光发射位”。
                 *
                 * 为什么可以省掉这一段:
                 *   激光是 K230 自己在对准成功那一刻点亮的(它会先发一行 "FIRE"),
                 *   而 K230 判“成功”= 摄像头正对靶心, 也就是此刻的 TARGET_LOOK 姿态。
                 *   再花 ~6s 摆到 TARGET_FIRE 已无意义(TARGET_FIRE 与 TARGET_LOOK
                 *   本只差几码), 而且很可能激光早就打完了。
                 *   ⇒ 直接抬臂收工, 省下 6 秒。
                 *
                 * ⚠️ 抬大臂会改变摄像头/激光朝向, 但此时激光已经打完(2s 内),
                 *    不影响命中。
                 * ⚠️ 若以后又要恢复“先摆发射位再抬臂”: 必须用 Target_GotoFirePose()
                 *    (它 = TARGET_FIRE + 精对准累计的 ID1/ID4 偏差), 不能用
                 *    Arm_Start_Target_Fire() —— 后者整表下发会把对准偏差冲掉。 */
                MLOG("阶段: 打靶收尾(收到OK -> 抬起大臂 -> 收回手臂)");
                Arm_Start_Target_Lift();               /* → ARM_POSE_TARGET_LIFT  */
                /* ⭐ 诊断: 抬完大臂立刻读回 5 个舵机的实际位置。
                 *   把这里读到的值和 TARGET_LIFT {222,1843,878,1834,93} 对比:
                 *     差 < 20 码      → 舵机有力, 机械臂没薓 → 薓是别处的问题;
                 *     明显偏离(偏小)  → 大臂被重力压下来了 → 供电跌落/扭矩不足/过热。
                 *   注: 这行会多花几十 ms(读 5 个舵机), 不需要诊断时可注释掉。 */
                Arm_LogActualPositions("靶-抬完大臂");
                /* ⭐ 省时: 当前姿态表里 TARGET_LIFT 与 SCAN_RESET 的 5 个值
                 *    【完全一样】(都是 {222,1843,878,1834,93}) ⇒ 这一步其实是
                 *    同一个姿态, 舵机一步都不会动, 却要白等一次“运动时间+保持时间”。
                 *    这里比对一下: 相同就跳过; 将来把 SCAN_RESET 改成别的标定值时
                 *    (两者不再相等)会自动恢复执行, 不会漏动作。 */
                {
                    const uint16_t *pl = s_arm_pose_table[ARM_POSE_TARGET_LIFT];
                    const uint16_t *pr = s_arm_pose_table[ARM_POSE_SCAN_RESET];
                    uint8_t same = 1;

                    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
                        if (pl[i] != pr[i]) { same = 0; break; }
                    }
                    if (same) {
                        MLOG("打靶收尾: SCAN_RESET 与 TARGET_LIFT 值相同 -> 跳过第二段(省一次空跑)");
                    } else {
                        /* ⭐ 2026-10-11: 这步在【打靶阶段】里执行 ⇒ 保持时间用打靶档(200ms),
                         *    与"打靶 200 / 其它 450"的口径一致(姿态名是 SCAN_RESET,
                         *    按名字会自动落到"其它"档, 所以这里显式指定)。 */
                        Arm_GotoPoseHold(ARM_POSE_SCAN_RESET, ARM_HOLD_MS_TARGET);
                    }
                }
                target_sub_state = TARGET_COMPLETE;
                break;

            case TARGET_COMPLETE:
                MLOG("任务2(打靶)完成");
                /* 为下一次打靶(如果重跑)复位; 同时清 0, 让下一个任务的
                 * Vision_SendTask 把上面三段摆臂阻塞期间攒下的旧帧 flush 掉 */
                target_id1_inited = 0;
                target_heard      = 0;
                target_fine_steps = 0;
                target_fine_tick  = 0;
                target_fine_cmd_tick = 0;
                target_fine_xok_warned = 0;
                target_freeze_warned   = 0;
                target_fire_tick  = 0;
                target_id4_steps  = 0;
                target_last_dy_valid = 0;
                target_last_dy_step  = 0;
                target_lost_cnt    = 0;     /* ⭐ 2026-10-11: 丢靶自恢复状态复位 */
                target_lost_tick   = 0;
                target_lost_dir    = 1;
                target_fine_d_seen = 0;
                target_prev_ex_valid = 0;   /* ⭐ 精对准“方向翻转检测”: 复位 */
                target_prev_ey_valid = 0;
                target_sub_state  = TARGET_IDLE;
                g_vision_task_in_progress = 0;
                /* ⭐ 打靶结束 → 右移 ROUTE_12_P2_A_MM(到【拐角】) → 后退 → 校 0° → 救援(转90°)
                 * ⚠️⚠️ 2026-10-07 实测结论 —— 【别再把转提前】:
                 *    这段右移到位后小车正好到车场【拐角】, 那里原地转 90° 的余量才充足;
                 *    把转提前到打靶位、或把这段右移改得太短, 都会在余量不足的地方转 → 扫出界。
                 *    (11:39 曾试过“打靶一结束就转”, 按这条结论已改回) */
                g_mission_state = STATE_12_PART2_MOVE_A;
                break;

            default: break;
        }
    }
    /* ================= 任务3: 救援 (视觉对准 + 抓取; K230 任务号=4) =================
     * 执行到本函数时: 小车已走完 “右转90° → 航向校准到位 → 停稳3s → 右移1000 → 停等3s”,
     *                 臂已摆到 ARM_POSE_HOSTAGE_LOOK(摄像头对准人质)
     *
     * ⭐ 对准方式两套, 由 RESCUE_SCHEME_ID1 切换(见文件头“救援任务参数”):
     *   【方案二】(=1, 当前) 与打靶同构 —— 【底盘完全不动】, 收到 L/R 就小步转
     *       底座 ID1; 收到 C 就认为对准 → 抓取。
     *   【方案一】(=0) 与排爆球/桶同构 —— L/R 先让车前进 / 右移一小段
     *       (车头已右转 90°), 再发 start_align 用 D:x,y 精对准(只修前后), 对准后抓取。
     * 两套共用的收尾:
     *   ⑥ RESCUE_SETTLE   停车稳定(FINE_TUNE_SETTLE_MS)
     *   ⑦ RESCUE_PERFORM  抓取: 该侧 抓取位 → 抱紧位 → 抬起位 → 回程姿态
     *       (⭐ 2026-10-10: “先退 15mm”不在这里 —— 它在更早的 ⑥ 到达人质处那一步)
     *   ⑧ RESCUE_COMPLETE 回主状态机 → 右移 ROUTE_17_RIGHT_B_MM …
     *
     * 🛡 方案二防卡死: 转满 RESCUE_ID1_STEP_MAX 步 或 超过 RESCUE_ID1_TIMEOUT_MS
     *    仍未收到 C(含 K230 完全不回应) → 强制当作 C 处理, 保证能往下走
     *
     * ⭐ 方案二在开始对准前会先让 ID1 慢速巡视一圈(中间→右→左→中间),
     *    期间一旦收到 K230 任何数据就中止巡视并转入正常对准 —— 见 RESCUE_SWEEP_*
     */
    else if (expected_task_number == 3) {
        switch (rescue_sub_state) {
            case RESCUE_IDLE:
#if MISSION_TEST_NO_VISION
                /* 测试: 无 K230, 模拟"已对准"→停稳→执行营救 */
                MLOG("[救援] 接近方向(测试模拟): C -> 直接抓取");
                Chassis_Stop();
                settle_until = HAL_GetTick() + FINE_TUNE_SETTLE_MS;
                rescue_sub_state = RESCUE_SETTLE;
#else
                if (RESCUE_SCHEME_ID1) {
                    /* ================= 方案二: 车不动, 只转底座 ID1 ================= */
                    /* 本次救援第一次进来: 复位计数/计时, 并把 ID1 缓存对齐到
                     * HOSTAGE_LOOK(进本状态前 STATE_16_RESCUE_RIGHT_A 已摆好它) */
                    if (!rescue_id1_inited) {
                        rescue_id1_inited = 1;
                        rescue_heard      = 0;
                        rescue_id1_steps  = 0;
                        rescue_id1_tick   = HAL_GetTick();
                        Arm_Id1Reset(ARM_POSE_HOSTAGE_LOOK);
                        /* ⭐ 把三个标定角度码【归一化】后打出来, 一眼核对方向/幅值:
                         *    中间 2059 → 0 ; 右 2503 → +444 ; 左 1561 → -498 */
                        MLOG("[救援] 底座ID1对准开始: 基准 %d (归一化 0); "
                             "右 %d → %+ld; 左 %d → %+ld (负=左/正=右, 见 Rescue_Id1Norm)",
                             (int)s_id1_pos,
                             (int)RESCUE_ID1_RIGHT_POS, (long)Rescue_Id1Norm(RESCUE_ID1_RIGHT_POS),
                             (int)RESCUE_ID1_LEFT_POS,  (long)Rescue_Id1Norm(RESCUE_ID1_LEFT_POS));
                    }
                    /* ⭐ 第一次进来先让 ID1 【慢速巡视】(中间→右→中间→左→中间, 5 个点,
                     *    只动 ID1)。巡视点位就是三个标定角度码, 由 Rescue_Id1Norm()
                     *    归一化成偏移(见 s_rescue_sweep_code)。
                     *    巡视途中 K230 一有【稳定回应】(连续 2 帧同一个 token)就中止,
                     *    并把那一行留下来给下面的正常流程处理(见 Rescue_SweepOnce)。
                     * ⭐⭐ 2026-10-10: 一圈没回应 → 再巡视一圈(最多 RESCUE_SWEEP_MAX_ROUNDS 圈,
                     *    0 = 不限; 收到回应立即结束巡视)。 */
                    if (RESCUE_SWEEP_ENABLE && !rescue_swept) {
                        rescue_swept = 1;
                        /* ⚠️ 必须【先发 run_task:4 再巡视】: 否则第一圈那 ~11.5s 里 K230
                         *    还在跑上一个任务(根本没在找人质), 一行都不会回 ——
                         *    巡视就成了“白转一圈”, 看起来跟没巡视一样。
                         *    此时 g_vision_task_in_progress 已被打靶 TARGET_COMPLETE
                         *    清 0, 所以这次发送会顺带 flush 掉旧帧 + 开 500ms 换任务
                         *    静默期, 之后读到的行才算 K230 对救援任务的真实回应。
                         *    (第二圈起不用再发: K230 已经在跑救援任务了) */
                        Vision_SendTask(K230_TASK_RESCUE, "SHAPE", &vision_cmd_tick);
                        for (uint8_t round = 1u; ; round++) {
                            MLOG("[救援] ID1 慢速巡视: 第 %u 圈(上限 %u 圈, 0=不限)",
                                 (unsigned)round, (unsigned)RESCUE_SWEEP_MAX_ROUNDS);
                            if (Rescue_SweepOnce(rescue_first_line, sizeof(rescue_first_line))) {
                                rescue_have_first = 1;   /* 这一行下一轮优先处理 */
                                break;
                            }
                            if (RESCUE_SWEEP_MAX_ROUNDS != 0u &&
                                round >= (uint8_t)RESCUE_SWEEP_MAX_ROUNDS) {
                                MLOG("[救援] 已巡视 %u 圈都没有回应 -> 转正常等待 K230 "
                                     "(之后若回 C/L/R 仍照常处理, 另有 %dms 超时兜底)",
                                     (unsigned)round, (int)RESCUE_ID1_TIMEOUT_MS);
                                break;
                            }
                            MLOG("[救援] 第 %u 圈无回应 -> 再巡视一圈", (unsigned)round);
                        }
                        /* 巡视本身也花了好几秒(可能好几圈): 把对准超时从此刻重新计,
                         * 免得一扫完就被判超时而盲抓 */
                        rescue_id1_tick = HAL_GetTick();
                    }
                    /* 🛡防卡死: 步数 或 时长 超限 → 强制当作已对准, 直接去抓 */
                    if (rescue_id1_steps >= RESCUE_ID1_STEP_MAX ||
                        (HAL_GetTick() - rescue_id1_tick) > RESCUE_ID1_TIMEOUT_MS) {
                        MLOG("[救援] 底座ID1对准强制结束(已转%u步, 用时%lums, 距上次收到K230数据 %lums) -> 去抓取",
                             (unsigned)rescue_id1_steps,
                             (unsigned long)(HAL_GetTick() - rescue_id1_tick),
                             (unsigned long)K230_RxSilenceMs());
                        Chassis_Stop();
                        settle_until = HAL_GetTick() + FINE_TUNE_SETTLE_MS;
                        rescue_sub_state = RESCUE_SETTLE;
                        break;
                    }
                    {
                        uint8_t got = 0;
                        uint8_t from_sweep = 0;   /* 1 = 这一行是巡视截下的(已在巡视里稳过) */

                        /* ⭐ 巡视中止时截下的那一行优先处理(别把 K230 的第一次回应丢掉) */
                        if (rescue_have_first) {
                            rescue_have_first = 0;
                            memcpy(line, rescue_first_line, sizeof(line));
                            line[sizeof(line) - 1] = '\0';
                            got = 1;
                            from_sweep = 1;
                        } else if (Mission_GetNewLine(line, sizeof(line))) {
                            got = 1;
                        }

                        if (got) {
                            char dir;
                            char *pD = strstr(line, "D:");

                            /* ⭐ 稳定帧: 只有【刚收到的新帧】才要连续 N 帧一致;
                             *    巡视截下的那一行在巡视里已经稳过一轮了, 直接采纳。
                             *    (C/L/R 比字符; D 帧只比“是不是 D”—— 数值每帧都在变) */
                            if (!from_sweep &&
                                !Vision_StableFeed(&s_vs_rescue, Vision_TokenOf(line),
                                                   VISION_STABLE_FRAMES_RESCUE,
                                                   VISION_STABLE_TIMEOUT_MS, "救援")) {
                                break;   /* 还没稳: 本轮不动 ID1, 继续等下一帧 */
                            }

                            if (Vision_IsDirLine(line, &dir)) {
                                /* ---- C / L / R: 与以前完全一样 ---- */
                                rescue_heard = 1;   /* 已联系上 K230: 之后不再定时重发 */
                                rescue_path_taken = dir;
                                MLOG("[救援] 接近方向: %c  (L=目标偏画面左 / C=居中 / R=偏右)", rescue_path_taken);
                                if (rescue_path_taken == 'L') {
                                    Rescue_Id1Step((int32_t)RESCUE_ID1_LR_SIGN * RESCUE_ID1_STEP);
                                    rescue_id1_steps++;
                                    rescue_id1_move_tick = HAL_GetTick();
                                    rescue_sub_state = RESCUE_ID1_MOVING;
                                } else if (rescue_path_taken == 'R') {
                                    Rescue_Id1Step(-(int32_t)RESCUE_ID1_LR_SIGN * RESCUE_ID1_STEP);
                                    rescue_id1_steps++;
                                    rescue_id1_move_tick = HAL_GetTick();
                                    rescue_sub_state = RESCUE_ID1_MOVING;
                                } else {
                                    /* 'C': 粗对准完成(K230 说目标已居中)
                                     * (不直接上 RESCUE_PERFORM, 而是绕一下 RESCUE_SETTLE,
                                     *  等 ID1 完全停稳再夹, 免得还在动就把人质抱歪) */
                                    if (RESCUE_USE_FINE_ALIGN && !rescue_fine_req) {
                                        /* ⭐ 2026-10-07: C 之后再发 start_align, 让 K230 从
                                         *    接近(只回 C/L/R, 没有幅值)切到 ALIGN(回 D:x,y,
                                         *    带幅值) —— 只有拿到幅值, ID1 才能用“按误差大小
                                         *    自适应”的小步收进 ±RESCUE_ID1_ALIGN_TOL;
                                         *    否则固定 60 码一步会一直跨过窗口来回摆, 也就是
                                         *    用户看到的“机械臂识别时一直转圈”。
                                         *    ⚠️ rescue_heard 已为 1 ⇒ 不会再重发 run_task:4,
                                         *       否则会把 K230 从 ALIGN 拽回 APPROACH。
                                         *    (与打靶 TARGET_FINE_IDLE 的处理完全一致) */
                                        rescue_fine_req = 1;
                                        s_rescue_d_valid = 0;   /* ⭐ 新一轮精对准: 先清掉上一轮记的 x/y */
                                        s_rescue_fb_px   = 0;
                                        s_rescue_last_dx = 0;
                                        MLOG("[救援] 收到C(共粗转%u步) -> 发 start_align 进入精对准(D:x,y)",
                                             (unsigned)rescue_id1_steps);
                                        K230_Start_Align();
                                        K230_FlushAll();
                                        break;   /* 留在本状态: 由下面的 D 分支小步微调 */
                                    }
                                    /* 已发过 start_align 又收到 C: K230 那边可能没实现 ALIGN
                                     * (还在用接近模式回 C/L/R) → 按 C 处理, 别干等 */
                                    MLOG("[救援] 收到C(共转%u步) -> 停稳后抓取",
                                         (unsigned)rescue_id1_steps);
                                    Chassis_Stop();
                                    settle_until = HAL_GetTick() + FINE_TUNE_SETTLE_MS;
                                    rescue_sub_state = RESCUE_SETTLE;
                                }
                            } else if (strncmp(line, "OK", 2) == 0) {
                                /* ⭐ 2026-10-07: K230 在 ALIGN 里判“对准成功”会回 OK
                                 *    (打靶那边还会先发 FIRE)。它判成功后就停发误差了,
                                 *    所以收到就直接去抓, 不要再干等 D 帧。
                                 *    ⚠️ 它用的是【K230 自己的】容差(2026-10-08 起
                                 *       与 RESCUE_ID1_ALIGN_TOL 一起统一成 25px)。 */
                                rescue_heard = 1;
                                MLOG("[救援] 精对准完成(收到 OK, 共微调%u步) -> 停稳后抓取",
                                     (unsigned)rescue_id1_steps);
                                Chassis_Stop();
                                settle_until = HAL_GetTick() + FINE_TUNE_SETTLE_MS;
                                rescue_sub_state = RESCUE_SETTLE;
                            } else if (pD != NULL) {
                                /* ---- ⭐ D:<x>,<y> —— 认为“已发现目标”, 用 x 小步修 ID1 ----
                                 * 和打靶同构: K230 的 x = 画面中 - 目标x, x>0 = 目标偏画面左,
                                 * 与接近阶段的 'L' 同向 ⇒ 直接用 RESCUE_ID1_LR_SIGN 定方向。
                                 * |x| < RESCUE_ID1_ALIGN_TOL 就算对准 → 停稳后抓取。 */
                                int err_x = atoi(pD + 2);
                                char *pYcomma = strchr(pD + 2, ',');
                                int err_y = (pYcomma != NULL) ? atoi(pYcomma + 1) : 0;

                                /* ⭐⭐ 2026-10-10(用户要求): 把这一帧的 x/y 都记下来 ——
                                 *   x 用于“抓取时补底座 ID1 残余”(见 RESCUE_GRAB_USE_RESIDUAL),
                                 *   y(前后误差) 用于“抓取时补 ID2 里程”(见 RESCUE_FB_ARM_*)。
                                 *   ⚠️ 救援方案二自己解析 D 帧、不走 Vision_FineAlignProcess,
                                 *      所以必须在这里记 —— 否则那两个补偿会拿到
                                 *      上一个任务(桶)留下的旧值。 */
                                s_rescue_last_dx = err_x;
                                s_rescue_fb_px   = err_y;
                                s_rescue_d_valid = 1;

                                rescue_heard = 1;
                                if (abs(err_x) >= RESCUE_ID1_ALIGN_TOL) {
                                    /* ⭐ 步长按误差大小自适应(与打靶同构, 见
                                     *    Rescue_Id1StepFor): 一律 60 码一步会跨过
                                     *    ±RESCUE_ID1_ALIGN_TOL 的窗口来回摆, 收敛不了 */
                                    int32_t d = Rescue_Id1StepFor(err_x);

                                    /* 标定步长用: 打“上一步 ID1 【实际】转了多少码、|x| 变了多少” */
                                    if (rescue_last_dx_valid && (rescue_last_dx != err_x)) {
                                        MLOG("[救援][ID1步长标定] 上一步 ID1 转 %ld 码后, |x| %ldpx -> %dpx "
                                             "(变化 %+ldpx; 负=变小=方向对) —— 据此调 RESCUE_ID1_STEP_FINE",
                                             (long)rescue_last_dx_step, labs((long)rescue_last_dx), abs(err_x),
                                             labs((long)rescue_last_dx) - (long)abs(err_x));
                                    }
                                    rescue_last_dx       = err_x;
                                    rescue_last_dx_valid = 1;
                                    rescue_last_dx_step  = (d >= 0) ? d : -d;

                                    MLOG("[救援] 收到误差 D:x=%dpx (x>0=偏画面左) -> ID1 转 %+ld 码 [%s]",
                                         err_x, (long)d,
                                         (abs(err_x) > RESCUE_ID1_STEP_BIG_PX) ? "大步" :
                                         (abs(err_x) > RESCUE_ID1_STEP_MID_PX) ? "中步" : "小步");
                                    Rescue_Id1Step(d);
                                    rescue_id1_steps++;
                                    rescue_id1_move_tick = HAL_GetTick();
                                    rescue_sub_state = RESCUE_ID1_MOVING;
                                } else {
                                    MLOG("[救援] 误差已进容差(x=%dpx < %dpx) -> 停稳后抓取",
                                         err_x, (int)RESCUE_ID1_ALIGN_TOL);
                                    Chassis_Stop();
                                    settle_until = HAL_GetTick() + FINE_TUNE_SETTLE_MS;
                                    rescue_sub_state = RESCUE_SETTLE;
                                }
                            } else {
                                MLOG("[救援] 忽略无关行: %s (继续等 C/L/R 或 D:x,y)", line);
                            }
                        } else if (rescue_fine_req &&
                                   K230_RxSilenceMs() > VISION_RESPONSE_TIMEOUT_MS) {
                            /* ⭐ 兜底: 发完 start_align 后 K230 一直没回任何数据
                             *    (K230 端该任务没实现 ALIGN / 它自己卡住) → 按“已对准”
                             *    去抓, 不在这里干等到 RESCUE_ID1_TIMEOUT_MS(20s)。 */
                            MLOG("[救援] 已发 start_align 但 %lums 没收到任何数据"
                                 "(K230 该任务可能没实现精对准) -> 按已对准兜底去抓取",
                                 (unsigned long)K230_RxSilenceMs());
                            Chassis_Stop();
                            settle_until = HAL_GetTick() + FINE_TUNE_SETTLE_MS;
                            rescue_sub_state = RESCUE_SETTLE;
                        } else if (!rescue_heard &&
                                   (!g_vision_task_in_progress ||
                                    HAL_GetTick() - vision_cmd_tick >= VISION_CMD_RESEND_MS)) {
                            Vision_SendTask(K230_TASK_RESCUE, "SHAPE", &vision_cmd_tick);
                        }
                    }
                } else {
                    /* ============ 方案一: 动底盘(L→前进 / R→右移) + D:x,y 精对准 ============
                     * ⚠️ 2026-10-07: 车头已在救援阶段前【右转 90°】, 所以“画面右”对应的
                     *    车体方向已从“后退”改成“右移”(轨迹方向不变, 见文件头/路线宏注释)。 */
                    if (Mission_GetNewLine(line, sizeof(line))) {
                        char dir;
                        if (!Vision_IsDirLine(line, &dir)) {
                            MLOG("[救援] 忽略无关行: %s (继续等 C/L/R)", line);
                            break;   /* 无关行: 继续等 C/L/R */
                        }
                        g_vision_task_in_progress = 0;
                        rescue_path_taken = dir;
                        MLOG("[救援] 接近方向: %c  (L=目标偏画面左 / C=居中 / R=偏右)", rescue_path_taken);
                        /* ⭐ 救援方向: 画面左右 = 车体前后(见上面“视觉方向映射”);
                         *   而车头现在向右转了 90° ⇒ 车体“前后”在场地里 = 车体“左右”:
                         *   L → 前进;  R → 右移;  C → 已在中心, 不动 */
                        if (rescue_path_taken == 'C') {
                            /* 已在中心: 不移动, 直接进精对准(此时只修前后) */
                            Vision_StartFineAlign("SHAPE", &s_align_rescue,
                                                  &state_start_tick, &align_start_time, &cooldown_until);
                            rescue_sub_state = RESCUE_WAIT_ALIGN;
                        } else if (rescue_path_taken == 'L') {
                            if (RESCUE_ADJUST_MM) Chassis_Move_Forward(RESCUE_ADJUST_MM);
                            rescue_sub_state = RESCUE_WAIT_ADJUST;
                        } else if (rescue_path_taken == 'R') {
                            if (RESCUE_ADJUST_MM) Chassis_Move_Right(RESCUE_ADJUST_MM);
                            rescue_sub_state = RESCUE_WAIT_ADJUST;
                        }
                    } else if (!g_vision_task_in_progress ||
                               HAL_GetTick() - vision_cmd_tick >= VISION_CMD_RESEND_MS) {
                        if (!g_vision_task_in_progress) {
                            rescue_idle_tick = HAL_GetTick();
                        } else if (HAL_GetTick() - rescue_idle_tick > VISION_RESPONSE_TIMEOUT_MS) {
                            /* K230 形状跟踪尚未实现/未回应: 超时跳过视觉, 盲走营救(防卡死) */
                            MLOG("[救援] 视觉一直没回应, 超时改为盲走抓取(防卡死)");
                            g_vision_task_in_progress = 0;
                            rescue_sub_state = RESCUE_PERFORM;
                            break;
                        }
                        Vision_SendTask(K230_TASK_RESCUE, "SHAPE", &vision_cmd_tick);
                    }
                }
#endif
                break;

            case RESCUE_ID1_MOVING:
                /* [方案二] 等 ID1 转完。期间把 K230 攒下的旧行【全部丢掉】,
                 * 保证回到 RESCUE_IDLE 时拿到的是“转完之后”采集的新帧,
                 * 否则会拿旧误差连续累加 → 转过头(和打靶一样的坑) */
                while (Mission_GetNewLine(line, sizeof(line))) {
                    /* 丢弃旧帧 */
                }
                if ((HAL_GetTick() - rescue_id1_move_tick) >= RESCUE_ID1_MOVE_MS) {
                    rescue_sub_state = RESCUE_IDLE;
                }
                break;

            case RESCUE_WAIT_ADJUST:
                if (Chassis_Task_Is_Complete()) {
                    Vision_StartFineAlign("SHAPE", &s_align_rescue,
                                          &state_start_tick, &align_start_time, &cooldown_until);
                    rescue_sub_state = RESCUE_WAIT_ALIGN;
                }
                break;

            case RESCUE_WAIT_ALIGN:
                if (Vision_FineAlignTimeout("SHAPE", state_start_tick, align_start_time)) {
                    Chassis_Stop();
                    settle_until = HAL_GetTick();
                    rescue_sub_state = RESCUE_SETTLE;
                    break;
                }
#if !MISSION_TEST_NO_VISION
                if (Mission_GetNewLine(line, sizeof(line))) {
                    if (Vision_FineAlignProcess(line, "SHAPE", &cooldown_until, &settle_until,
                                                &s_align_rescue)) {
                        rescue_sub_state = RESCUE_SETTLE;
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
                MLOG("阶段: 救援抓取(抓取位→抱紧→抬起→回程)");
#if !MISSION_TEST_NO_VISION
                /* ⭐⭐ 对准完成 → 把当前底座 ID1 【锁存】下来(用户要求的“赋值到指令集”):
                 *   这一刻 ID1 就是把摄像头对准人质的角度 ⇒ 后面 抓取/抱紧/抬起
                 *   全用这个值, 不再跟姿态表里的 ID1 走(ID1 不再动)。
                 *   ⚠️ 只写进运行时变量, 不修改 s_arm_pose_table ——
                 *      重新上电/再跑一次救援时, 姿态表还是原始标定值, 只是到时
                 *      会重新锁存一次。 */
                s_rescue_id1_lock  = s_id1_pos;
                s_rescue_id1_valid = 1;     /* ⭐ 2026-10-11: 有效标志(值可能是负数, 见其说明) */
                /* ⭐ 2026-10-10: 锁存的同时把【最后一次 D 帧的横向残余】换算成补偿量 ——
                 *   对准是在 |x| < RESCUE_ID1_ALIGN_TOL(25px) 就停的, 停下时目标可能
                 *   还偏在容差内任意处; 把那点残余补掉, 抓取才是“真正居中”的角度。
                 *   ⚠️ 没读到过 D 帧(s_rescue_d_valid=0, 例如 K230 只回 OK/C)时【不补】,
                 *      免得拿上一轮/上一个任务的旧残余乱补;
                 *      换算/限幅见 RESCUE_GRAB_USE_RESIDUAL。
                 *   ⭐ 同一帧的 y(前后)误差用于“补 ID2 里程”, 见 RESCUE_FB_ARM_* 与
                 *      Arm_GotoRescuePoseKeepId1 的第二个参数(不需要在这里换算)。 */
                s_rescue_id1_resid = 0;
#if RESCUE_GRAB_USE_RESIDUAL
                if (s_rescue_d_valid && RESCUE_GRAB_PX_PER_CODE > 0.0f) {
                    float f = (float)s_rescue_last_dx / RESCUE_GRAB_PX_PER_CODE
                              * (float)RESCUE_ID1_LR_SIGN;
                    int32_t c = (int32_t)((f >= 0.0f) ? (f + 0.5f) : (f - 0.5f));

                    if (c >  (int32_t)RESCUE_GRAB_RESIDUAL_MAX_CODES) c =  (int32_t)RESCUE_GRAB_RESIDUAL_MAX_CODES;
                    if (c < -(int32_t)RESCUE_GRAB_RESIDUAL_MAX_CODES) c = -(int32_t)RESCUE_GRAB_RESIDUAL_MAX_CODES;
                    s_rescue_id1_resid = c;
                }
#endif
                MLOG("救援: 底座 ID1 锁存 = %ld (抓取/抱紧/抬起 期间不会改动 ID1; "
                     "残余 %ldpx -> 补 %+ld 码, 手动偏移 %+d 码 ⇒ 抓取实际用 %ld)",
                     (long)s_id1_pos,
                     (s_rescue_d_valid ? (long)s_rescue_last_dx : 0L),
                     (long)s_rescue_id1_resid, (int)Rescue_GrabId1OffsetNow(),
                     (long)Rescue_GrabId1Code());
#endif
                /* ⚠️ 2026-10-10: “先退 15mm”那一步【不在抓取这一拍了】——
                 *    按用户要求已挪到【到达人质处、开始识别之前】, 见 STATE_16_RESCUE_RIGHT_A
                 *    的进入动作(宏仍是 ROUTE_RESCUE_GRAB_BACK_MM)。 */
                Arm_Start_Rescue_Grab();      /* ① 该侧抓取位 → ② 该侧抱紧位(ID1 锁定) */
                Arm_Start_Rescue_Retract();   /* ③ 该侧抬起位(ID1 锁定) */
                Arm_Start_Rescue_Return();    /* ④ 回程姿态(收臂, 准备横移; 这一步 ID1 回表中值) */
                rescue_sub_state = RESCUE_COMPLETE;
                break;

            case RESCUE_COMPLETE:
                MLOG("任务3(救援)完成");
                /* 复位, 并把 g_vision_task_in_progress 清 0,
                 * 让后面的视觉任务重新走 “Vision Start + 换任务静默期” */
                rescue_id1_inited = 0;
                rescue_heard      = 0;
                rescue_swept      = 0;
                rescue_fine_req   = 0;      /* 下次救援重新走“C → start_align → D 精对准” */
                rescue_have_first = 0;
                rescue_last_dx_valid = 0;   /* 步长标定统计也清掉(下次救援重新开始) */
                rescue_last_dx_step  = 0;
                s_rescue_id1_lock  = 0;
                s_rescue_id1_valid = 0;     /* ⭐ 解锁: 下次救援重新“对准后再锁存” */
                s_rescue_id1_resid = 0;     /* ⭐ 残余补偿一起清(下次重新按新残余算) */
                s_rescue_fb_px    = 0;      /* ⭐ 2026-10-10: 这一轮的 D 帧 x/y 一起清,
                                             *    免得下次救援/别的任务拿旧值去补里程 */
                s_rescue_last_dx  = 0;
                s_rescue_d_valid  = 0;
                rescue_sub_state  = RESCUE_IDLE;
                g_vision_task_in_progress = 0;
                /* ⭐ 抓取完成 → ⑦ 抓完后第 1 段右移(STATE_17_RESCUE_RIGHT_B)
                 * (⭐ 2026-10-10: “先退 15mm”那一步【不再在这里】—— 已按用户要求
                 *  挪到【夹取之前】(到抓取位先退一点再夹人质), 见 RESCUE_PERFORM) */
                g_mission_state = STATE_17_RESCUE_RIGHT_B;
                break;

            default: break;
        }
    }
}

#if MISSION_DEBUG_VISION_TASK
/* =====================================================================
 * 视觉单独调试(不跑路线、不动机械臂)
 *   MISSION_DEBUG_VISION_TASK:
 *     1=球 2=靶 3=桶 4=形状 -> 按 KEY1 自动对准对应目标一次
 *     5=串口链路监控          -> 纯收发打印, 车/臂完全不动
 *                                 KEY1=scan_qr, KEY1长按=run_task:1, KEY2=reset:0
 * ===================================================================== */
static uint8_t s_dbg_sub = 0;        /* 0=发指令等C/L/R 1=等盲走 2=精对准 3=停稳 4=完成 */
static uint8_t s_dbg_done = 0;
static uint32_t s_dbg_move_tick = 0;
static uint32_t s_dbg_state_tick = 0;
static uint32_t s_dbg_align_tick = 0;
static uint32_t s_dbg_cooldown = 0;
static uint32_t s_dbg_settle = 0;
static uint32_t s_dbg_cmd_tick = 0;
static uint32_t s_dbg_rx_count = 0;   /* 链路监控: 累计收到行数 */

#if MISSION_DEBUG_VISION_TASK == 6
/* =====================================================================
 * ⭐ 打靶两轴标定模式 (MISSION_DEBUG_VISION_TASK == 6)
 * ---------------------------------------------------------------------
 * 和上面 1~4 最大区别: 【底盘一步都不动】, 只转机械臂两个舵机:
 *      ID1(底座) 修横向 x       ID4(腕部) 修竖直 y
 * 用途: 标定这两个轴的【方向符号】(TARGET_ID1_LR_SIGN / TARGET_ID4_DY_SIGN)
 *       和【每步步长】(TARGET_ID1_STEP / TARGET_ID4_STEP)。
 * ⚠️ 上面 1~4 那套是“底盘横移式对准”, 用的是 K_GAIN(整台车横移),
 *    和打靶实际用的“只转底座”完全是两套逻辑 —— 所以拿 mode 2 是
 *    标不了 ID1/ID4 的, 必须用本模式。
 * ---------------------------------------------------------------------
 * 操作:
 *   ① 把靶子放在摄像头能看见的地方, 【故意放偏一点】
 *      (放正中间的话 K230 直接回 OK, 没东西可标);
 *   ② 上电按一下 KEY1 → 机械臂自动摆到 TARGET_LOOK, 然后开始微调;
 *   ③ 看日志:
 *        [TCAL] 横向轴: x=-80px (x>0=偏左) -> ID1 转 +60 码
 *        [TCAL] 竖直轴: y=-161px (y>0=偏下) -> ID4 转 -40 码
 *        [TCAL] 竖直轴变化: 上一步 ID4 动作后 -161px -> -121px (变化 40px)
 *      判定:
 *        |x| / |y| 越修越小 → 符号对、步长合适 → 完事
 *        越修越大        → 把对应的 *_SIGN 取反 (改宏 → 重载+构建)
 *        来回摆          → 步长太大, 把 *_STEP 调小
 *   ④ 看到 “[TCAL] 两轴都进容差” + “[TCAL] K230 已发射激光(FIRE)” 就是成功
 * ===================================================================== */
static uint8_t  s_tc_sub   = 0;      /* 0=摆姿态 1=发run_task等C/L/R 2=微调 3=等舵机动完 4=停稳 5=完 */
static uint8_t  s_tc_done  = 0;
static uint32_t s_tc_tick  = 0;      /* 进入“微调”的时刻(算总超时) */
static uint32_t s_tc_cmd_tick = 0;   /* 上次发 run_task / start_align 的时刻 */
static uint32_t s_tc_move_tick = 0;  /* 上次下舵机指令的时刻 */
static uint16_t s_tc_move_ms = 0;    /* 这一步要等多久 */
static uint32_t s_tc_i1_steps = 0;   /* 横向(ID1)已转步数 */
static uint32_t s_tc_i4_steps = 0;   /* 竖直(ID4)已转步数 */
static uint8_t  s_tc_tol_warned = 0;
static int32_t  s_tc_last_dx = 0;    /* 上一步动作前的横向误差(看变化量用) */
static uint8_t  s_tc_last_dx_valid = 0;
static int32_t  s_tc_last_dy = 0;
static uint8_t  s_tc_last_dy_valid = 0;
static int32_t  s_tc_last_i1_delta = 0;  /* 上一步 ID1 【实际】转的码数(步长自适应) */
static int32_t  s_tc_last_i4_delta = 0;  /* 上一步 ID4 【实际】转的码数 */
static uint32_t s_tc_settle_tick = 0;
static uint32_t s_tc_fire_tick = 0;      /* 收到 FIRE 的时刻(0=还没收到) */
static uint8_t  s_tc_xignore_warned = 0; /* [只标竖直] 模式下“请把靶子横向摆正”提示只打一次 */

static void TargetCalib_Start(void)
{
    s_tc_sub = 0;
    s_tc_done = 0;
    s_tc_tick = HAL_GetTick();
    s_tc_cmd_tick = 0;
    s_tc_move_tick = 0;
    s_tc_move_ms = 0;
    s_tc_i1_steps = 0;
    s_tc_i4_steps = 0;
    s_tc_tol_warned = 0;
    s_tc_last_dx_valid = 0;
    s_tc_last_dy_valid = 0;
    s_tc_fire_tick = 0;
    s_tc_xignore_warned = 0;
    g_vision_task_in_progress = 0;
    Chassis_Stop();               /* 底盘全程不动 */
    MLOG("打靶两轴标定: 开始 (底盘不动, 只用 ID1 修横向 / ID4 修竖直)");
}

static void TargetCalib_Update(void)
{
    char line[K230_LINE_MAX];
    char path;

    if (s_tc_done) {
        return;   /* 跑完一次就停住, 方便看日志 */
    }

    switch (s_tc_sub) {
        case 0:   /* 摆 TARGET_LOOK 姿态(摄像头对准靶) + 复位两个轴的基准 */
            MLOG("打靶两轴标定: 摆 TARGET_LOOK 姿态...");
#if MISSION_TEST_NO_ARM
            MLOG("打靶两轴标定: MISSION_TEST_NO_ARM=1, 跳过摆臂(只发指令不碰舵机)");
#else
            Arm_GotoPose(ARM_POSE_TARGET_LOOK);
#endif
            Arm_Id1Reset(ARM_POSE_TARGET_LOOK);
            Arm_Id4Reset(ARM_POSE_TARGET_LOOK);
            K230_FlushAll();
            Chassis_Stop();
            MLOG("打靶两轴标定[K标定] 基准位置: ID1=%d, ID4=%d; 本轮参数: "
                 "ID1 步长 小%d/中%d/大%d 符号=%d (%ums) / "
                 "ID4 步长 小%d/远%d 符号=%d (%ums), 容差=%dpx",
                 (int)s_id1_pos, (int)s_id4_pos,
                 (int)TARGET_ID1_STEP_FINE, (int)(TARGET_ID1_STEP / 2), (int)TARGET_ID1_STEP,
                 (int)TARGET_ID1_LR_SIGN, (unsigned)TARGET_ID1_MOVE_MS,
                 (int)TARGET_ID4_STEP, (int)TARGET_ID4_STEP_FAR,
                 (int)TARGET_ID4_DY_SIGN, (unsigned)TARGET_ID4_MOVE_MS,
                 (int)TARGET_ALIGN_TOLERANCE);
            MLOG("打靶两轴标定[K标定]: 靶子要【故意放偏】, 正中间 K230 会直接回 OK 没东西可标");
            s_tc_tick = HAL_GetTick();
            s_tc_sub = 1;
            break;

        case 1:   /* 发 run_task:2, 等 K230 的 C/L/R 方向行 */
            if (Mission_GetNewLine(line, sizeof(line))) {
                if (Vision_IsDirLine(line, &path)) {
                    g_vision_task_in_progress = 0;
                    MLOG("打靶两轴标定: K230 接近方向 %c (底盘不动, 直接进精对准)", path);
                    MLOG("打靶两轴标定: 请求精对准(发 start_align)");
                    K230_Start_Align();
                    K230_FlushAll();
                    s_tc_tick = HAL_GetTick();
                    s_tc_sub = 2;
                } else {
                    MLOG("打靶两轴标定: 忽略无关行: %s (继续等 C/L/R)", line);
                }
            } else if (!g_vision_task_in_progress ||
                       HAL_GetTick() - s_tc_cmd_tick >= VISION_CMD_RESEND_MS) {
                Vision_SendTask(K230_TASK_TARGET, "TCAL", &s_tc_cmd_tick);
            }
            break;

        case 2:   /* 微调: 读 D:x,y / OK / FIRE */
            /* ⭐ FIRE 宽限: K230 发完 FIRE 就静默(不再回 OK) —— 别白等 20s */
            if (s_tc_fire_tick != 0 &&
                (HAL_GetTick() - s_tc_fire_tick) >= K230_FIRE_GRACE_MS) {
                MLOG("打靶两轴标定: K230 发过 FIRE 但 %dms 内没等到 OK -> 结束(它发完 FIRE 就不发了)",
                     (int)K230_FIRE_GRACE_MS);
                s_tc_settle_tick = HAL_GetTick();
                s_tc_sub = 4;
                break;
            }
            /* 🛡防卡死: 步数或时长超限 -> 结束 */
            if (s_tc_i1_steps >= TARGET_ID1_STEP_MAX ||
                s_tc_i4_steps >= TARGET_ID4_STEP_MAX ||
                (HAL_GetTick() - s_tc_tick) > TARGET_ID1_TIMEOUT_MS) {
                MLOG("打靶两轴标定: 强制结束(横向%u步/竖直%u步, 用时%lums, 距上次收到K230数据 %lums)",
                     (unsigned)s_tc_i1_steps, (unsigned)s_tc_i4_steps,
                     (unsigned long)(HAL_GetTick() - s_tc_tick),
                     (unsigned long)K230_RxSilenceMs());
                s_tc_settle_tick = HAL_GetTick();
                s_tc_sub = 4;
                break;
            }

            if (!Mission_GetNewLine(line, sizeof(line))) {
                break;
            }

            if (strncmp(line, "FIRE", 4) == 0) {
                MLOG("打靶两轴标定: K230 已发射激光(FIRE) —— ⚠️ 它发完 FIRE 会静默, %dms 内没 OK 也收尾",
                     (int)K230_FIRE_GRACE_MS);
                if (s_tc_fire_tick == 0) {
                    s_tc_fire_tick = HAL_GetTick();
                }
            } else if (strncmp(line, "OK", 2) == 0) {
                MLOG("打靶两轴标定: 收到 OK, 精对准结束(横向%u步/竖直%u步)",
                     (unsigned)s_tc_i1_steps, (unsigned)s_tc_i4_steps);
                s_tc_settle_tick = HAL_GetTick();
                s_tc_sub = 4;
                break;
            } else {
                char *pD = strstr(line, "D:");
                if (pD != NULL) {
                    char *pComma = strchr(pD, ',');
                    int err_x = atoi(pD + 2);
                    int err_y = (pComma != NULL) ? atoi(pComma + 1) : 0;

                    /* 串行修正: 先横向(ID1), 后竖直(ID4)。转腕部会耦合画面横向,
                     * 同时动两个会互相干扰。K230 每帧只发一个轴(它挑大的那个)。
                     * ⚠️ 容差用 TARGET_ALIGN_TOLERANCE(20), 与 K230 新窗口一致 */
                    if (abs(err_x) >= TARGET_ALIGN_TOLERANCE && !TCAL_ONLY_Y) {
                        int32_t d1 = Target_Id1StepFor(err_x);

                        if (s_tc_last_dx_valid && s_tc_last_dx != err_x) {
                            MLOG("打靶两轴标定[横向轴变化] 上一步 ID1 转 %ld 码后, |x| %ldpx -> %dpx "
                                 "(变化 %+ldpx; 负=变小=方向对) —— 据此调 TARGET_ID1_STEP",
                                 (long)s_tc_last_i1_delta, labs((long)s_tc_last_dx), abs(err_x),
                                 labs((long)s_tc_last_dx) - (long)abs(err_x));
                        }
                        s_tc_last_dx = err_x;
                        s_tc_last_dx_valid = 1;
                        s_tc_last_i1_delta = (d1 >= 0) ? d1 : -d1;

                        MLOG("打靶两轴标定[横向] x=%dpx (x>0=偏左) -> ID1 转 %+ld 码 [%s]",
                             err_x, (long)d1,
                             (abs(err_x) > TARGET_ID1_STEP_BIG_PX) ? "大步" :
                             (abs(err_x) > TARGET_ID1_STEP_MID_PX) ? "中步" : "小步");
                        Target_Id1Step(d1);
                        s_tc_i1_steps++;
                        s_tc_move_ms   = TARGET_ID1_MOVE_MS;
                        s_tc_move_tick = HAL_GetTick();
                        s_tc_sub = 3;
                    }
#if TARGET_FIX_DY_WITH_ID4
                    else if (abs(err_y) >= TARGET_ALIGN_TOLERANCE) {
                        int32_t d4 = Target_Id4StepFor(err_y);

                        if (s_tc_last_dy_valid && s_tc_last_dy != err_y) {
                            MLOG("打靶两轴标定[竖直轴变化] 上一步 ID4 转 %ld 码后, |y| %ldpx -> %dpx "
                                 "(变化 %+ldpx; 负=变小=方向对) —— 据此调 TARGET_ID4_STEP",
                                 (long)s_tc_last_i4_delta, labs((long)s_tc_last_dy), abs(err_y),
                                 labs((long)s_tc_last_dy) - (long)abs(err_y));
                        }
                        s_tc_last_dy = err_y;
                        s_tc_last_dy_valid = 1;
                        s_tc_last_i4_delta = (d4 >= 0) ? d4 : -d4;

                        MLOG("打靶两轴标定[竖直] y=%dpx (y>0=偏下) -> ID4 转 %+ld 码 [%s]",
                             err_y, (long)d4,
                             (abs(err_y) > TARGET_ID4_STEP_FAR_PX) ? "大步" : "小步");
                        Target_Id4Step(d4);
                        s_tc_i4_steps++;
                        s_tc_move_ms   = TARGET_ID4_MOVE_MS;
                        s_tc_move_tick = HAL_GetTick();
                        s_tc_sub = 3;
                    }
#endif
                    else if (TCAL_ONLY_Y && abs(err_x) >= TARGET_ALIGN_TOLERANCE) {
                        /* 只标竖直模式: K230 新代码【两个轴都会报】(不再"只报误差大的那一轴"),
                         * 而我们又不许动 ID1 → 只能提示人工把靶子横向摆正 */
                        if (!s_tc_xignore_warned) {
                            s_tc_xignore_warned = 1;
                            MLOG("打靶两轴标定[只标竖直]: 现在 K230 报的是横向 x=%dpx"
                                 " (新 K230 两个轴都会报); 请把靶子【横向】挪到画面中间"
                                 "(|x|<%dpx), 免得横向误差一直占着这一步不放",
                                 err_x, (int)TARGET_ALIGN_TOLERANCE);
                        }
                    }
                    else if (!s_tc_tol_warned) {
                        s_tc_tol_warned = 1;
                        MLOG("打靶两轴标定: 两轴都已在容差(%dpx)内 (x=%d, y=%d), 等 K230 回 OK/FIRE; "
                             "若一直不回, 就是 K230 那边判不通过(它现在要求 |dx| 和 |dy| 都 <%dpx)",
                             (int)TARGET_ALIGN_TOLERANCE, err_x, err_y,
                             (int)TARGET_ALIGN_TOLERANCE);
                    }
                } else if (Vision_IsDirLine(line, &path)) {
                    /* K230 还在发接近指令 → 说明它没切进 ALIGN(可能 start_align 丢了),
                     * 按 1s 节流重发, 别干等到 20s 超时 */
                    if ((HAL_GetTick() - s_tc_cmd_tick) >= VISION_CMD_RESEND_MS) {
                        MLOG("打靶两轴标定: K230 还在发接近指令(%c), 重发 start_align", path);
                        K230_Start_Align();
                        s_tc_cmd_tick = HAL_GetTick();
                    }
                } else {
                    MLOG("打靶两轴标定: 忽略无关行: %s", line);
                }
            }
            break;

        case 3:   /* 等刚才那一步舵机转完; 期间旧帧丢掉, 但 OK/FIRE 要留(只发一次) */
            {
                uint8_t ok_seen = 0;
                while (Mission_GetNewLine(line, sizeof(line))) {
                    if (strncmp(line, "OK", 2) == 0) {
                        ok_seen = 1;
                    } else if (strncmp(line, "FIRE", 4) == 0) {
                        MLOG("打靶两轴标定: K230 已发射激光(FIRE)");
                        if (s_tc_fire_tick == 0) {
                            s_tc_fire_tick = HAL_GetTick();
                        }
                    }
                }
                if (ok_seen) {
                    MLOG("打靶两轴标定: 在挪舵机期间就收到 OK -> 结束");
                    s_tc_settle_tick = HAL_GetTick();
                    s_tc_sub = 4;
                } else if ((HAL_GetTick() - s_tc_move_tick) >= (uint32_t)s_tc_move_ms) {
                    s_tc_sub = 2;      /* 舵机走完了, 回去读 K230 新帧 */
                }
            }
            break;

        case 4:   /* 停稳 + 打总结 */
            if ((HAL_GetTick() - s_tc_settle_tick) >= FINE_TUNE_SETTLE_MS) {
                MLOG("打靶两轴标定: 完成 —— ID1 转了 %u 步(净偏移 %+ld 码), ID4 转了 %u 步(净偏移 %+ld 码)",
                     (unsigned)s_tc_i1_steps, (long)s_id1_offset,
                     (unsigned)s_tc_i4_steps, (long)s_id4_offset);
                MLOG("打靶两轴标定: 怎么判定 —— 上面每步日志里 |x|/|y| 越修越小 = 符号对; "
                     "越修越大 = 把 TARGET_ID1_LR_SIGN / TARGET_ID4_DY_SIGN 取反; "
                     "来回摆 = 把 TARGET_ID1_STEP / TARGET_ID4_STEP 调小");
                s_tc_done = 1;
                s_tc_sub = 5;
            }
            break;

        default:  /* 5 = 完成 */
            break;
    }
}
#endif /* MISSION_DEBUG_VISION_TASK == 6 */

/* 链路监控发送: which 0=reset:0, 1=run_task:1(球), 2=run_task:2(靶) */
void Mission_DebugVisionLinkSend(uint8_t which)
{
    if (which == 0) {
        K230_Reset();
        MLOG("发送: reset:0 (让 K230 回待机状态)");
    } else if (which == 1) {
        K230_Run_Specific_Task(K230_TASK_BALL);
        MLOG("发送: run_task:%d (球)", (int)K230_TASK_BALL);
    } else {
        K230_Run_Specific_Task(K230_TASK_TARGET);
        MLOG("发送: run_task:%d (靶)", (int)K230_TASK_TARGET);
    }
}

void Mission_DebugVisionStart(void)
{
#if MISSION_DEBUG_VISION_TASK == 6
    TargetCalib_Start();
    return;
#endif
    s_dbg_sub = 0;
    s_dbg_done = 0;
    g_vision_task_in_progress = 0;
    Chassis_Stop();
    MLOG("视觉调试: 任务 %d 开始 (底盘不动/机械臂不动)", (int)MISSION_DEBUG_VISION_TASK);
    MLOG("视觉调试[K标定]: 先用尺子把目标从画面正中挪开已知距离 D(mm);"
         " 对准时看 [K标定] 日志, a = D / 初始误差(px), 建议 K_GAIN = 0.5×a");
}

void Mission_DebugVisionUpdate(void)
{
    uint8_t task;
    const char *tag;
    const AlignAxisCfg_t *cfg;   /* 本任务(球/靶/桶/形状)对应的轴向/方向配置 */
    int32_t c_mm, l_mm, r_mm;
    char line[K230_LINE_MAX];

    if (MISSION_DEBUG_VISION_TASK == 5) {
        /* ---- 串口链路监控: 只收发, 不动车/臂 ---- */
        static uint8_t inited = 0;
        if (!inited) {
            inited = 1;
            MLOG("链路监控: KEY1=run_task:1(球) KEY1长按=run_task:2(靶) KEY2=reset:0");
        }
        while (Mission_GetNewLine(line, sizeof(line))) {
            s_dbg_rx_count++;
            MLOG("收到 K230[%lu]: %s", (unsigned long)s_dbg_rx_count, line);
        }
        return;
    }

#if MISSION_DEBUG_VISION_TASK == 6
    /* ---- 打靶两轴标定: 底盘一步不动, 只转 ID1(横向) / ID4(竖直) ---- */
    TargetCalib_Update();
    return;
#endif

     if (s_dbg_done) {
        return;   /* 完成一次后停住, 方便看结果 */
    }

    switch (MISSION_DEBUG_VISION_TASK) {
        case 1: task = K230_TASK_BALL;   tag = "BALL";   c_mm = 15; l_mm = 60; r_mm = 60;
                cfg = &s_align_ball;   break;
        case 2: task = K230_TASK_TARGET; tag = "TARGET"; c_mm = 0;  l_mm = 50; r_mm = 50;
                cfg = &s_align_target; break;
        case 3: task = K230_TASK_BUCKET; tag = "BUCKET"; c_mm = 20; l_mm = 60; r_mm = 60;
                cfg = &s_align_bucket; break;
        case 4: task = K230_TASK_RESCUE; tag = "SHAPE";  c_mm = 0;  l_mm = 50; r_mm = 50;
                cfg = &s_align_rescue; break;
        default: s_dbg_done = 1; return;
    }

    switch (s_dbg_sub) {
        case 0:   /* 发 run_task, 等 C/L/R */
#if MISSION_TEST_NO_VISION
            MLOG("[%s] 接近方向(测试模拟): C", tag);
            if (c_mm) Chassis_Move_Forward(c_mm);
            s_dbg_move_tick = HAL_GetTick();
            s_dbg_sub = 1;
#else
            if (Mission_GetNewLine(line, sizeof(line))) {
                char path;
                if (!Vision_IsDirLine(line, &path)) {
                    MLOG("[%s] 忽略无关行: %s", tag, line);
                    break;
                }
                g_vision_task_in_progress = 0;
                MLOG("[%s] 接近方向: %c", tag, path);
                if (path == 'C')      { if (c_mm) Chassis_Move_Forward(c_mm); }
                else if (path == 'L') { if (l_mm) Chassis_Move_Left(l_mm); }
                else                  { if (r_mm) Chassis_Move_Right(r_mm); }
                s_dbg_move_tick = HAL_GetTick();
                s_dbg_sub = 1;
            } else if (!g_vision_task_in_progress ||
                       HAL_GetTick() - s_dbg_cmd_tick >= VISION_CMD_RESEND_MS) {
                Vision_SendTask(task, tag, &s_dbg_cmd_tick);
            }
#endif
            break;

        case 1:   /* 等盲走完成 */
            if (Chassis_Task_Is_Complete() || (HAL_GetTick() - s_dbg_move_tick > BLIND_MOVE_TIMEOUT_MS)) {
#if MISSION_TEST_NO_VISION
                MLOG("[%s] 精对准(测试模拟): 已对准", tag);
                Chassis_Stop();
                s_dbg_settle = HAL_GetTick() + FINE_TUNE_SETTLE_MS;
                s_dbg_sub = 3;
#else
                Vision_StartFineAlign(tag, cfg, &s_dbg_state_tick, &s_dbg_align_tick, &s_dbg_cooldown);
                s_dbg_sub = 2;
#endif
            }
            break;

        case 2:   /* 精对准 */
            if (Vision_FineAlignTimeout(tag, s_dbg_state_tick, s_dbg_align_tick)) {
                s_dbg_settle = HAL_GetTick();
                s_dbg_sub = 3;
            }
#if !MISSION_TEST_NO_VISION
            else if (Mission_GetNewLine(line, sizeof(line))) {
                if (Vision_FineAlignProcess(line, tag, &s_dbg_cooldown, &s_dbg_settle,
                                            cfg)) {
                    s_dbg_sub = 3;
                }
            }
#endif
            break;

        case 3:   /* 停稳 */
            if (HAL_GetTick() >= s_dbg_settle) {
                s_dbg_sub = 4;
            }
            break;

        default:  /* 完成 */
            MLOG("视觉调试: 任务 %d 完成", (int)MISSION_DEBUG_VISION_TASK);
            s_dbg_done = 1;
            break;
    }
}
#endif /* MISSION_DEBUG_VISION_TASK */

#if MISSION_DEBUG_ARM_SEQ
/* =====================================================================
 * 机械臂动作单独调试(底盘完全不动, 不跑视觉/不跑路线)
 *   MISSION_DEBUG_ARM_SEQ: 1=排爆序列
 * 用法: 改宏 -> 重载构建 -> 上电按 KEY1 跑一遍; 全程不发任何底盘指令。
 * ===================================================================== */
static uint8_t s_armdbg_step = 0;
static uint8_t s_armdbg_idle = 1;   /* 1=空闲(等 KEY1), 0=序列执行中 */

void Mission_DebugArmStart(void)
{
    if (!s_armdbg_idle) {
        MLOG("机械臂调试: 正在执行序列, 忽略本次按键");
        return;
    }
    Chassis_Stop();          /* 确认底盘停住; 之后全程不发底盘指令 */
    s_armdbg_step = 0;
    s_armdbg_idle = 0;
    MLOG("机械臂调试 序列%d: 开始 (底盘锁定不动)", (int)MISSION_DEBUG_ARM_SEQ);
}

//该函数在机械臂的调试模式下，按下按键之后，在空闲阶段s_armdbg_step会增加，动作下一个
void Mission_ChangeStep(void)
{
    /* ⚠️ 原来这里是 if (!s_armdbg_idle) { "busy, ignore"; return; }
     *    —— 条件写反了: 按键时序列恰好正在运行(idle==0), 于是每次都被挡掉,
     *    永远切不到下一个动作。现在改成“已启动才切步”。 */
    s_armdbg_idle = 0;
     if (s_armdbg_idle) {
        MLOG("机械臂调试: 尚未开始(按 KEY1 启动)");
        return;
    }
    s_armdbg_step++;
    if (s_armdbg_step > 5) {
        s_armdbg_step = 0;
    }
    MLOG("目前的阶段是: %d", (int)s_armdbg_step);
}

/* 供 main.c 判断“KEY1 按下去是启动还是切步” */
uint8_t Mission_DebugArmIsIdle(void)
{
    return s_armdbg_idle;
}

void Mission_DebugArmUpdate(void)
{
    if (s_armdbg_idle) {
        return;
    }
    switch (s_armdbg_step) {
        case 0:   /* 回初始位 */
            MLOG("机械臂调试: 回到初始位(HOME)");
            Servos_SetPositions(Servos_GetHomePositions(), 1500);
            HAL_Delay(2000);
            // s_armdbg_step = 1;
            break;

        case 1:     /* 扫码 */
            // Arm_GotoPose(ARM_POSE_HOSTAGE_LOOK);//机械臂转向至抓取人质位
            // MLOG("Debug arm: HOSTAGE_LOOK抓取人质状态");
            // Arm_GotoPose(ARM_POSE_BALL_LOOK);
            //  MLOG("Debug arm: Look看球");
            MLOG("机械臂调试: 停顿(等手动摆臂)");
            HAL_Delay(500);
            // MLOG("Debug arm: SCAN扫码位置状态");
            // Arm_GotoPose(ARM_POSE_SCAN);        //扫码姿态
            // HAL_Delay(2000);
            // s_armdbg_step = 2;
            break;

        case 2:   /* 抓取序列(张开→下放→闭合→抬臂) */
            // Arm_Start_Rescue_Grab();//抓取人质
            MLOG("机械臂调试: 抓取小球(GRAB)");
            Arm_Start_Bomb_Grab();          //排爆抓取小球
            
            // s_armdbg_step = 3;
            break;

        case 3:   /* 停顿(便于观察/换物) */
            MLOG("机械臂调试: 停顿(便于观察/换物)");
            HAL_Delay(500);
            // s_armdbg_step = 4;
            break;

        case 4:   /* 放置序列(转向→下放→松开→抬臂) */
            // Arm_Start_Rescue_Retract();//放置人质
            MLOG("机械臂调试: 放置小球(PLACE)");
            Arm_Start_Bomb_Place();         //排爆放置小球到球桶
            // s_armdbg_step = 5;
            break;

        case 5:   /* 回SCAN_RESET, 结束 */
            Arm_GotoPose(ARM_POSE_SCAN_RESET);      //扫码后复位
            HAL_Delay(2000);
            MLOG("机械臂调试: 全部完成");
            // s_armdbg_idle = 1;
            break;
            

        default:
            s_armdbg_idle = 1;
            break;
    }
}
#endif /* MISSION_DEBUG_ARM_SEQ */

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
#if MISSION_TEST_NO_VISION
    static uint32_t qr_scan_start = 0;
#endif
    static uint32_t s_qr_cmd_tick = 0;   /* STATE_2 开始等 SCAN_OK 的时刻(超时兜底用) */
    /* 阶段四③: 停下等待的起始时刻(等 RESCUE_STOP_WAIT_MS 再摆臂) */
    static uint32_t s_rescue_wait_tick = 0;
    /* 阶段四③': 航向校准【到位】后的“原地停稳 3s”起始时刻(RESCUE_ALIGN_SETTLE_MS) */
    static uint32_t s_rescue_align_tick = 0;
    /* 阶段三: 打靶走位第 2 段右移后“停车停稳”的起始时刻(TARGET_STOP_SETTLE_MS) */
    static uint32_t s_target_stop_tick = 0;

    /* 每周期先把 UART4 中断里那一行(单缓冲)搬进小队列, 再交给状态机:
     * K230 新代码是连续输出的(接近阶段每 ~0.5s 一行 C/L/R), 不搬的话
     * 只靠 Mission_Coop_Wait(摆臂时)去收, 路上很容易被下一行覆盖丢掉。 */
    K230_QueuePush();

    /* ============================================================
     * 第一段: 状态进入动作 (仅在状态切换瞬间执行一次)
     * ============================================================ */
    if (g_mission_state != last_state) {
        MLOG("状态切换: %d -> %d", (int)last_state, (int)g_mission_state);
        last_state = g_mission_state;   /* 记录新状态, 防止同一状态重复触发 */

        switch (g_mission_state) {
            /* ---------- 阶段一: 扫码并前往排爆区 ---------- */
            case STATE_1_MOVING_TO_QR_SCAN:   Chassis_Move_Forward(ROUTE_1_TO_QR_MM); break;  /* 起点→扫码点(直行) */
            /* 扫码: 车停稳后机械臂先摆到“扫码姿态”再让 K230 扫。
             * ⭐ 2026-10-06: 改成【多角度扫描】—— Arm_Scan_Begin() 先把整张
             *    ARM_POSE_SCAN 姿态摆到位, 之后由转移检查里的 Arm_Scan_Poll()
             *    用 ID4 轮换 3 个角度, 提高二维码命中率(见该函数注释)。 */
            case STATE_2_PERFORMING_QR_SCAN:  Chassis_Stop(); Arm_Scan_Begin(); break;

            case STATE_3_MOVE_LEFT_A:
                /* ⭐⭐ 2026-10-11(用户确认): 本段(= 扫码后左移 ROUTE_3_LEFT_A_MM)不要
                 *    平滑减速, 直接急停 —— 见 ROUTE_3_LEFT_STOP_MODE 的说明。 */
                Route_ApplyStopMode(ROUTE_3_LEFT_STOP_MODE);
                Chassis_Move_Left(ROUTE_3_LEFT_A_MM);
                break;    /* 扫码后左移 A 段 */
            case STATE_3A_HEADING_CORRECT:    Turn_Angle_Compat(0.1f); break;                 /* 左移后航向校正到 0° */
            case STATE_4_MOVE_TOPLEFT:        Chassis_Move_Diagonal(ROUTE_4_DIAG_FWD_MM, ROUTE_4_DIAG_LEFT_MM); break;  /* 左上斜跑(前/左分量见宏) */
            case STATE_4A_HEADING_CORRECT:    Turn_Angle_Compat(0.1f); break;                 /* 斜跑后航向校正到 0° */
            case STATE_5_STOP_BEFORE_RAMP:    Chassis_Move_Forward(ROUTE_5_TO_RAMP_MM); break;/* 斜坡前直行 */
            case STATE_6_CROSSING_RAMP_A:     Turn_Angle_Compat(0.1f); break;                 /* 过斜坡段: 航向校正 */

            /* ---------- STATE_7 分段: 左移B(前半) -> 前进 -> 左移B(后半) -> 中间停 -> 左移C -> 航向校正 ----------
             * ⭐⭐ 2026-10-10(用户要求): 左移 B 段【对半拆开走】, 中间插一次“车头前进”:
             *    左移 871/2 → 车头前进 ROUTE_7_LEFT_B_FWD_MM(20mm) → 左移剩下那一半。
             *    目的: 一次左移 871mm 车身会甩/前后漂, 中间往前顶一下把车“顺”回来
             *    (三个宏见 ROUTE_7_LEFT_B_MM 处的说明; 改总长只需改那一个数)。 */
#if ROUTE_7_LEFT_B_SPLIT_ENABLE
            case STATE_7_MOVE_LEFT_B:
                /* ⭐⭐ 2026-10-11(用户要求): 本段(左移 B, ROUTE_7_LEFT_B_MM)不要平滑减速,
                 *    直接急停 —— 见 ROUTE_7_LEFT_B_STOP_MODE 的说明。
                 *    ⚠️ "拆半走法"下这里只是前半, 同样按急停处理(总长不变)。 */
                Route_ApplyStopMode(ROUTE_7_LEFT_B_STOP_MODE);
                Chassis_Move_Left(ROUTE_7_LEFT_B_HALF_MM);
                break;   /* 左移阶段B 前半 */
            case STATE_7B_MOVE_FWD_MID:       Chassis_Move_Forward(ROUTE_7_LEFT_B_FWD_MM); break; /* ⭐ B 段中间: 车头前进 */
            case STATE_7_MOVE_LEFT_B2:        Chassis_Move_Left(ROUTE_7_LEFT_B_HALF2_MM); break;  /* 左移阶段B 后半 */
#else
            /* ⭐⭐ 2026-10-10(用户要求): 一整段左移 —— 中间【不前进】、也【不分段】
             *    (上面那两行就是被这个宏关掉的部分; 走完直接进 STATE_7A 停顿 + 转正) */
            case STATE_7_MOVE_LEFT_B:
                /* ⭐⭐ 2026-10-11(用户要求): 本段(左移 B, ROUTE_7_LEFT_B_MM)不要平滑减速,
                 *    直接急停 —— 见 ROUTE_7_LEFT_B_STOP_MODE 的说明。 */
                Route_ApplyStopMode(ROUTE_7_LEFT_B_STOP_MODE);
                Chassis_Move_Left(ROUTE_7_LEFT_B_MM);
                break;
            case STATE_7B_MOVE_FWD_MID:       break;   /* 不会被进入(见 ROUTE_7_LEFT_B_SPLIT_ENABLE) */
            case STATE_7_MOVE_LEFT_B2:        break;   /* 不会被进入 */
#endif
            case STATE_7A_INTERMEDIATE_STOP:  Turn_Angle_Compat(0.1f); break;                 /* 中间停顿 + 航向校正 */
            case STATE_7B_MOVE_LEFT_C:        break;//Chassis_Move_Diagonal(ROUTE_7_LEFT_C_MM,ROUTE_7_LEFT_C_MM);Chassis_Move_Left(ROUTE_7_LEFT_C_MM); break;    /* 左移阶段C */
            case STATE_7A_HEADING_CORRECTION: break;//Turn_Angle_Compat(0.1f);                  /* 航向校正 */

            /* ---------- 排爆区走位 ---------- */
            case STATE_8_MOVE_FORWARD_A:      Chassis_Move_Forward(ROUTE_8_TO_BOMB_AREA_MM); break;  /* 直线前进进入排爆区 */
            case STATE_8A_HEADING_CORRECT:    Turn_Angle_Compat(0.1f); break;                 /* 前进后航向校正 */
            case STATE_8B_ADJUST_RIGHT:
                /* ⭐ 2026-10-11: 本段的停车方式由 ROUTE_8B_RIGHT_STOP_MODE 决定
                 *    (用户确认"不要减速"的是 STATE_3 那段, 所以这里当前 = 0 平滑减速;
                 *     要让本段也急停, 把 ROUTE_8B_RIGHT_STOP_MODE 置 1) */
                Route_ApplyStopMode(ROUTE_8B_RIGHT_STOP_MODE);
                Chassis_Move_Right(ROUTE_8B_RIGHT_MM);
                break;    /* 右移微调 */
            /* 右移到位后(8C)先把机械臂摆到“看球姿态”, 让 STATE_11 的视觉对准一开始
             * 就能看到小球; 先发转向指令再摆臂 → 车转+臂动 同时进行, 省时间 */
            case STATE_8C_HEADING_CORRECT:    Turn_Angle_Compat(0.1f); Arm_GotoPose(ARM_POSE_BALL_LOOK); break;
            case STATE_9_TURN_FOR_BOMB:       break;//Chassis_Rotate(ROUTE_9_TURN_DEG); break;        /* 转向排爆点(角度见宏) */
            case STATE_9A_ADJUST_LEFT:        break;//Chassis_Move_Left(ROUTE_9A_LEFT_MM); break;     /* 左移微调 */
            case STATE_10_APPROACH_BOMB:      break; //Chassis_Move_Left(ROUTE_10_APPROACH_MM); break; /* 接近炸弹 */
            case STATE_11_PERFORMING_BOMB_DISPOSAL: Chassis_Stop(); break;                    /* 停下, 交给视觉子状态机 */

            /* ⭐⭐ 2026-10-09 新增(用户要求): 排爆(抓球 + 放桶)全做完、进打靶走位之前,
             *    先做一次航向校正 —— 做法与打靶走位那几处完全一样(先挪一小步再转正):
             *      ① Route_MinStep(ROUTE_BOMB_AFTER_STEP_MM = +9): 车头前进 9mm
             *         (此刻车头还是 0° ⇒ 走的是【场地前方】), 把轮子"滚起来";
             *      ② Turn_Angle_Compat(0.1f): 转到绝对 0°(与全程其它 0° 校正同一套)。
             *    ⚠️ Route_MinStep 内部【阻塞等到位】, 所以紧接着发转向是安全的;
             *       转向到位由转移检查里的 Chassis_Task_Is_Complete() 等。 */
            case STATE_11A_HEADING_CORRECT:
#if GYRO_BIAS_TGT_ON
                /* ⭐ 方案②且在打靶走位段启用: 排爆后起锚, 打靶平移段开始零偏在线估计 */
                Chassis_GyroBias_Start();
#endif
                MLOG("排爆后: 航向校正(先挪 %+dmm, 再转到 0°)", (int)ROUTE_BOMB_AFTER_STEP_MM);
                Route_MinStep(ROUTE_BOMB_AFTER_STEP_MM);
                /* ⭐⭐ 2026-10-11(用户要求): 改成"转到位 → 停稳 → 再压一次"的阻塞式写法。
                 *    原来只发一次转向、由转移条件等它"完成" —— 而"完成"允许车还在
                 *    以 ≤10°/s 慢慢转(见 Chassis.h 的 CH_TURN_RATE_THRESHOLD), 判完成后
                 *    又是 Chassis_Stop() 断输出【滑行】⇒ 最终朝向会再偏 1~2°。
                 *    ⚠️ 顺序: 每次都是"发转向 → 阻塞等到位", 中间那次停稳只是等。
                 *    ⚠️⚠️ 2026-10-11 实测补丁: 两次等待都必须用 Chassis_WaitTurnDone()
                 *      (超时会强制 Chassis_Stop()) —— 实车这次两次都撞上限, 而老写法
                 *      超时后转向环【还开着】(剩余角 126°), 紧接着的右移就变成
                 *      "边走边转", 走出弧线(日志 STEER on=1 err=126 out=192)。 */
                {
                    /* ⭐ 2026-10-11: 偏差 ≤ TURN_SKIP_DEG 就别发了(1~2° 的校正
                     *    在这台车上转不动、还会把位置目标越积越大, 见 TURN_SKIP_DEG)。 */
                    if (!Turn_SkipIfTiny(0.0f)) {
                        /* ⭐⭐ 用户要求: 【放完球这次校正力度加大】(见 BOMB_AFTER_ALIGN_TURN_ADJUST) */
                        Chassis_SetNextTurnAdjust(BOMB_AFTER_ALIGN_TURN_ADJUST);
                        Chassis_Rotate_To(0.0f);
                        Chassis_WaitTurnDone(BOMB_AFTER_ALIGN_TIMEOUT_MS);
                    }
                    MLOG("排爆后: 第1次转到 0° 结束, yaw=%.1f°, 停稳 %dms 后再压一次",
                         (double)Chassis_GetYaw(), (int)BOMB_AFTER_ALIGN_SETTLE_MS);
                    Mission_Coop_Wait(BOMB_AFTER_ALIGN_SETTLE_MS);

                    if (!Turn_SkipIfTiny(0.0f)) {
                        /* ⭐ 压正这次同样加大力度(否则"停稳后剩的那点残余"照样转不动) */
                        Chassis_SetNextTurnAdjust(BOMB_AFTER_ALIGN_TURN_ADJUST);
                        Chassis_Rotate_To(0.0f);
                        Chassis_WaitTurnDone(BOMB_AFTER_ALIGN_TIMEOUT_MS);
                    }
                    MLOG("排爆后: 第2次(压正)结束, 最终 yaw=%.1f° -> 开始右移",
                         (double)Chassis_GetYaw());
                }
                break;

            /* ---------- 阶段二: 打靶 ----------
             * ⭐⭐ 2026-10-10(用户要求): 走位由两个开关切换
             *    (ROUTE_12_SPLIT_ENABLE / ROUTE_12_MID_CORRECT_ENABLE) ——
             *   【SPLIT=0, MID=1(当前)】一条长路 + 中间校一次 + 最后校一次:
             *     STATE_11A(出发前先校 0°) → MOVE_A(右移【前半】)
             *     → CORRECT_B【中间那次: 挪一小步 + 校到 0°】
             *     → MOVE_B(右移【后半】)
             *     → CORRECT_B2(最后一次校 + 停稳) → MOVE_C(摆 ARM_POSE_TARGET_LOOK)
             *     → STATE_13 打靶视觉对准
             *     (CORRECT_A / MOVE_A2 / CORRECT_A2 / MOVE_B2 这 4 个不会被进入)
             *   【SPLIT=0, MID=0】一条路直接走完, 中途不校正, 只剩 CORRECT_B2 那一次;
             *   【SPLIT=1】老做法(每段右移再对半拆, 每一半走完都校一次 = 共 4 次):
             *     MOVE_A → CORRECT_A → MOVE_A2 → CORRECT_A2
             *     → MOVE_B → CORRECT_B → MOVE_B2 → CORRECT_B2 → MOVE_C
             *   三种走法的【总距离都是 ROUTE_12_P1_A_MM + ROUTE_12_P1_B_MM】⇒ 终点不变。
             * 打完靶收尾: PART2_MOVE_A(右移) → PART2_CORRECT_A(航向校正) → 救援
             *
             * ⭐ 右移都走 Target_MoveRight()(现在 BACK_COMP = 0 ⇒ 就是普通右移)。
             * ⭐ 各处航向校正【之前】的“挪一小步”: 总开关 = ROUTE_12_MINSTEP_ENABLE(当前 = 1),
             *    各用【自己的】固定步长宏(初始都是 -15):
             *      第 1 处 CORRECT_A  = ROUTE_12_A_MINSTEP_MM
             *      第 2 处 CORRECT_A2 = ROUTE_12_A2_MINSTEP_MM(+可选“按偏航角”方案二)
             *      第 3 处 CORRECT_B  = ROUTE_12_B_MINSTEP_MM
             *      第 4 处 CORRECT_B2 = ROUTE_12_B2_MINSTEP_MM(+可选“按偏航角”方案二)
             *
             * ⚠️ 写法上的关键点(看下面代码时心里要数这一条):
             *   【进入动作】只在“刚切到这个状态”的那一瞬间执行一次,
             *   之后每个主循环只跑【转移检查】。
             *   ⇒ “航向校正/右移”这种需要时间的动作, 就是靠
             *     “进入时发一次指令 + 转移条件里等 Chassis_Task_Is_Complete()”
             *     配合完成的(绝不能在转移里再发一次, 否则会反复重发)。 */
#if ROUTE_12_SPLIT_ENABLE
            case STATE_12_PART1_MOVE_A:       Target_MoveRight(ROUTE_12_P1_A_HALF_MM); break;  /* ⭐ 第1段右移 前半 */
#else
            /* ⭐⭐ 2026-10-10(用户要求): 一条路走到打靶位置(第1段+第2段 合并成 1708mm 左右).
             *    中途那次校正由 ROUTE_12_MID_CORRECT_ENABLE 决定:
             *      1(当前) = 这里只走【前半】, 走完做“挪一小步 + 校一次航向”(CORRECT_B),
             *                再由 MOVE_B 走【后半】, 最后由 CORRECT_B2 校最后一次;
             *      0       = 一次走完全程(中途不校正), 只剩最后那次校正。 */
            case STATE_12_PART1_MOVE_A:
#if ROUTE_12_MID_CORRECT_ENABLE
                MLOG("打靶走位: 右移【前半】%dmm(走完在中间校一次航向)", (int)ROUTE_12_TOTAL_HALF_MM);
                Target_MoveRight(ROUTE_12_TOTAL_HALF_MM);
#else
                MLOG("打靶走位: 一条路右移 %dmm(第1段+第2段合并, 中途不校正)",
                     (int)ROUTE_12_TOTAL_MM);
                Target_MoveRight(ROUTE_12_TOTAL_MM);
#endif
                break;
#endif
            /* ⭐ 第 1 处航向校正(第1段前半走完): 先挪 ROUTE_12_A_MINSTEP_MM, 再转正到 0° */
            case STATE_12_PART1_CORRECT_A:
                Route_BackMinStep(ROUTE_12_A_MINSTEP_MM);
                Turn_Angle_Compat(0.1f);
                break;
            case STATE_12_PART1_MOVE_A2:      Target_MoveRight(ROUTE_12_P1_A_HALF2_MM); break; /* ⭐ 第1段右移 后半 */
            /* ⭐ 第 2 处航向校正(第1段全程走完) ——
             *    原来一次右移 850mm 车头会攒到约 5°, 所以每半段末尾都要校一次。
             *    挪多少由 Route_A2_MinStepMM() 按 ROUTE_12_A2_STEP_MODE 选方案:
             *      ★当前 = 方案一: 固定 ROUTE_12_A2_MINSTEP_MM(-15)。 */
            case STATE_12_PART1_CORRECT_A2:
                Route_BackMinStep(Route_A2_MinStepMM());
                Turn_Angle_Compat(0.1f);
                break;
#if ROUTE_12_SPLIT_ENABLE
            case STATE_12_PART1_MOVE_B:       Target_MoveRight(ROUTE_12_P1_B_HALF_MM); break;   /* 老走法: 第2段右移 前半 */
#else
            /* "一条路"走法的【后半】(中间那次校正之后接着把剩下的走完) */
            case STATE_12_PART1_MOVE_B:       Target_MoveRight(ROUTE_12_TOTAL_HALF2_MM); break;
#endif
            /* ⭐ 第 3 处航向校正(老走法: 第2段前半走完) —— 先挪 ROUTE_12_B_MINSTEP_MM, 再转正到 0°。
             * ⭐⭐ 2026-10-10: 它现在也是【"一条路"走法中间那次航向校正】(用户要求:
             *    "在中间的地方加一段航向校正 + 挪一小步")—— 即前半走完先挪一小步再校一次,
             *    然后才由 MOVE_B 走后半。开关 = ROUTE_12_MID_CORRECT_ENABLE(当前 = 1)。
             *    挪多少 = ROUTE_12_B_MINSTEP_MM(当前 -15 = 车头后退 15mm)。
             * ⭐ 2026-10-06 历史原因: 原来这里【只停车、不校正航向】—— 实测右移 850mm
             *    之后车头偏了约 5°(日志 I/HDG err=-5.0), 下一步 MOVE_C 就把
             *    机械臂/摄像头伸出去识别靶子 → “车是斜的就开始识别”。
             * ⚠️ 不要先调 Chassis_Stop(): 它会 chassis_freeze() + 清 s_turn_open,
             *    紧接着 Rotate_To 又开转向 —— 两个状态混在同一拍里容易让
             *    Chassis_Task_Is_Complete() 的判定提前成立(转一半就判“完成”)。
             *    转向结束时它自己会调 Chassis_Stop()。 */
            case STATE_12_PART1_CORRECT_B:
#if ROUTE_12_MID_CORRECT_ENABLE
                /* ⭐⭐ 2026-10-10(用户要求): 顺序 = 【先摆正车头 → 再后退 → 再走后半段】。
                 *    所以这里【阻塞】等到车头真的摆正了, 才做那一步“挪一小步(后退)”:
                 *      ① Chassis_Rotate_To(0) + 等到位(超时上限 = ROUTE_12_MID_ALIGN_TIMEOUT_MS);
                 *      ② Route_BackMinStep(ROUTE_12_B_MINSTEP_MM) —— 摆正之后再后退,
                 *         走完才切走(转完立刻走, 轮子也还“活着”, 不会卡着转不动)。
                 *    ⚠️ 顺序不能倒过来: 先挪步再转会覆盖掉转向指令(Chassis_Rotate 内部
                 *       会清 s_moving), 那就变成“只退不转”了 —— 这正是之前那种写法的问题。
                 *    ⚠️ 用 Mission_Coop_Wait 等(不是 HAL_Delay): 期间照刷陀螺仪 yaw、
                 *       照收 K230 行, 否则转向环读到冻结角度会“转不完”。
                 *    ⚠️ 超时后仍会执行后退(会取消没转完的转向) —— 判据看下面日志里
                 *       “耗时”是否贴近上限且 yaw 不为 0, 见 ROUTE_12_MID_ALIGN_TIMEOUT_MS。 */
                {
                    if (!Turn_SkipIfTiny(0.0f)) {
                        Chassis_Rotate_To(0.0f);
                        /* ⭐⭐ 2026-10-11: 用 Chassis_WaitTurnDone —— 超时会强制 Chassis_Stop(),
                         *    否则"没转完就去后退"会把转向指令顶掉(老注释里那句"超时后仍会
                         *    执行后退(会取消没转完的转向)"的隐患就此消除)。 */
                        Chassis_WaitTurnDone(ROUTE_12_MID_ALIGN_TIMEOUT_MS);
                    }
                    MLOG("打靶走位(中间): 车头已摆正 yaw=%.1f°(目标 0°), 接着后退 %dmm",
                         (double)Chassis_GetYaw(),
                         (int)ROUTE_12_B_MINSTEP_MM);
                    Route_BackMinStep(ROUTE_12_B_MINSTEP_MM);
                }
#else
                Route_BackMinStep(ROUTE_12_B_MINSTEP_MM);
                Chassis_Rotate_To(0.0f);
#endif
                break;
            case STATE_12_PART1_MOVE_B2:      Target_MoveRight(ROUTE_12_P1_B_HALF2_MM); break; /* ⭐ 第2段右移 后半 */
            /* ⭐ 第 4 处航向校正(第2段全程走完) —— 摆臂前的“停稳”就挂在这一步上
             *    (TARGET_STOP_SETTLE_MS 从本状态进入时刻起算)。
             *    挪多少由 Route_B2_MinStepMM() 按 ROUTE_12_B2_STEP_MODE 选方案
             *    (与第 2 处独立; ★当前 = 方案一: 固定 ROUTE_12_B2_MINSTEP_MM = -15)。 */
            case STATE_12_PART1_CORRECT_B2:
                Route_BackMinStep(Route_B2_MinStepMM());
                Chassis_Rotate_To(0.0f);
                s_target_stop_tick = HAL_GetTick();
                break;
            /* ⭐⭐ 打靶摆臂/识别之前: 【阻塞式】把车头摆正再伸臂
             * 用户实测反馈:“航向纠正到一半就进了视觉状态机, 车身斜着打靶”。
             * 所以这里不能只发个转向指令就往下走, 必须【同步等它转完】:
             *   ① 发转向指令(转到绝对 0°);
             *   ② 阻塞等 Chassis_Task_Is_Complete() (转向环自己判到位);
             *   ③ 再停稳 TARGET_STOP_SETTLE_MS;
             *   ④ 打印实际 yaw 供核对, 然后才 Arm_GotoPose(TARGET_LOOK)。
             * ⚠️ 用 Mission_Coop_Wait 等(不是 HAL_Delay): 期间照刷陀螺仪 yaw、
             *    照收 K230 行, 否则转向环读不到新角度会“转不完”。
             * ⚠️ 5s 超时兜底: 万一陀螺仪/转向环异常也不卡死在这里。
             * 摆完臂后转移条件 Chassis_Task_Is_Complete() 已为真 → 直接进 STATE_13,
             * 也就是说【进视觉状态机时车头一定是正的】。 */
            case STATE_12_PART1_MOVE_C:
                {
                    if (!Turn_SkipIfTiny(0.0f)) {
                        Chassis_Rotate_To(0.0f);
                        Chassis_WaitTurnDone(5000u);     /* ⭐ 2026-10-11: 超时会强制停车 */
                    }
                    Mission_Coop_Wait(TARGET_STOP_SETTLE_MS);
                    MLOG("打靶摆臂前: 航向校正结束, 当前 yaw=%.1f° (目标 0°)",
                         (double)Chassis_GetYaw());
                }
                Arm_GotoPose(ARM_POSE_TARGET_LOOK);
                break;

            /* ⚠️ TURN_A/TURN_B 已不在流程里(不可达): 上面 MOVE_C 摆完臂直接跳 STATE_13。
             *    这里置空, 免得万一被进入时车突然跑 400mm */
            case STATE_12_TURN_A:             break;//Chassis_Move_Right(ROUTE_12_TURN_A_DEG);
            case STATE_12_TURN_B:             break;//Turn_Angle_Compat(0.1f);

            /* ⭐ 打靶收尾三段 —— 顺序: 右移(到拐角) → 后退 → 校0° → 才进救援阶段转 90°。
             * ⚠️⚠️ 2026-10-07 实测: 这三段【必须在转 90° 之前】走完 —— 右移到位后小车
             *    正好到车场【拐角】, 只有在那儿原地转 90° 的余量才充足。
             *    ⇒ 此时车头还是 0°, 所以②用的是“后退”(Chassis_Move_Backward)。
             * (后面的 MOVE_B/CORRECT_B/MOVE_C 未使用) */
            case STATE_12_PART2_MOVE_A:       Chassis_Move_Right(ROUTE_12_P2_A_MM); break;   /* ⭐ 右移到【拐角】 */
            case STATE_12_PART2_MOVE_BACKWARD: Chassis_Move_Backward(ROUTE_12_P2_BACK_MM); break;
            /* ③ ⭐ 校 0° → 进救援阶段转 90°。
             * ⭐ 方案 A2: PART2 的右移/后退也可能把纠偏停掉过 ⇒ 这里读到的就是真实偏差,
             *    Turn_Angle_Compat 会按绝对角把车头转回 0°, 救援段的 -90° 基准也就准了。 */
            case STATE_12_PART2_CORRECT_A:    Turn_Angle_Compat(0.1f); break;
            case STATE_12_PART2_MOVE_B:       break;   /* 已废弃 */
            case STATE_12_PART2_CORRECT_B:    break;//Turn_Angle_Compat(0.1f); break;                 /* 航向校正 */
            case STATE_12_PART2_MOVE_C:       break; //Chassis_Move_Right(ROUTE_12_P2_C_MM); break;  /* Part2 第3段 */

            /* 进入打靶视觉子状态机(底盘完全不动, 只转底座 ID1 对准 → 摆 FIRE/LIFT/SCAN_RESET)。
             * ⚠️ 这里不再做航向校正 —— CORRECT_A 已经校过一次, 而且 MOVE_C 摆臂时
             *    底盘是静止的, 不需要再校。 */
            case STATE_13_PERFORMING_TARGETING: break;

            /* ---------- 阶段四: 救援 (2026-10-07 改为“先掉头, 再横移”) ----------
             * ⭐⭐ 2026-10-07 实测(顺序不能颠倒): ①~③【必须】跑在打靶收尾那三段平移之后 ——
             *    那段右移到位后小车正好到车场【拐角】, 只有在那儿原地转 90° 余量才充足;
             *    把转提前(或把那段右移改得太短)会让车在余量不足处转 → 扫出界。
             * 完整流程:
             *   ①车头右转 90°(RESCUE_HEADING_DEG) + 设航向基准
             * → ①'【车头前进 ROUTE_RESCUE_AFTER_TURN_MM=50mm】(转完之后先送一段,
             *       再校准/停稳/右移进救援区 —— 用户 2026-10-07 要求)
             * → ②航向校准到 -90°
             * → ③校准【到位】后原地停稳 RESCUE_ALIGN_SETTLE_MS(3s)
             * → ③'【右移前再纯校一次航向】(★2026-10-09 新增, 不挪步: 原地右转 90°
             *       可能把车身带偏, 横移前再按绝对角校一次)
             * → ④右移【分两段】(⭐ 2026-10-09 新增): 第一段 ROUTE_14_MID_MM
             *      → ④'【中途一次航向校准】(挪 ROUTE_RESCUE_MID_STEP_MM + 校准到 -90°)
             *      → 第二段 ROUTE_14_MID2_MM(进救援区) → ⑤停下等 RESCUE_STOP_WAIT_MS
             * → ⑥摆 ARM_POSE_HOSTAGE_LOOK → ⑦视觉对准 + 抓取(子状态机)
             * → ⑧右移 ROUTE_17_RIGHT_B_MM → ⑨航向校准
             * → ⑩右移 ROUTE_18_RIGHT_C_MM → ⑪航向校准 → ⑫停下(任务完成)
             * ⭐ 航向纠正之前都先挪一小步(Route_MinStep), 车停死后原地转正容易转不到位,
             *    先让轮子滚起来更好转(取值见各宏):
             *      ②  ROUTE_RESCUE_AFTER_TURN_MM(转完后车头前进)
             *      ④' ROUTE_RESCUE_MID_STEP_MM  (右移中途; ★2026-10-09 新增)
             *      ⑨  ROUTE_RESCUE_FWD_MM        ⑪ ROUTE_RESCUE_LAST_STEP_MM
             *    ⚠️ 现在“进救援区 → 终点”一共 4 处航向纠正(④' ⑨ ⑪ + ② 转完那次)。
             * ⭐ 为什么“后退”全改成“右移”: 车头右转 90°(顺时针)之后, 车体的
             *    【右】方向正好等于原来的【后】方向 ⇒ 轨迹不变、只是车身姿态转了 90°。
             * ⚠️ ⑦ 的对准方式由 RESCUE_SCHEME_ID1 切换(1=转 ID1 车不动 / 0=动底盘)。
             * ------------------------------------------------------------------ */
            /* ① 车头【右转 90°】—— 位置由打靶收尾那三段平移决定:
             *    右移(到【拐角》) → 后退50 → 校0° 之后才轮到它(见 STATE_12_PART2_*);
             *    只有在那儿转的余量才够, 别提前。
             *    ⚠️ 必须同时把【航向基准】改成 -90°: 下面④的 Chassis_Move_Right
             *       是靠“航向保持”走直线的, 基准还是 0° 的话车会被一路拽回原朝向。 */
            case STATE_14_MOVE_FORWARD_B:
#if GYRO_BIAS_TGT_ON
                /* ⭐ 方案②且在打靶走位段启用: 打靶平移段结束停锚, 救援段恢复用原始 yaw */
                Chassis_GyroBias_Stop();
#endif
                /* ⭐⭐ 2026-10-11(用户实测"转弯 90° 时航向校准不准"): 改成【阻塞式
                 *    "转到位 → 停稳 → 再压正"】(门槛 1°, 不被 TURN_SKIP_DEG 的 2° 放过),
                 *    并把航向基准同步到 -90° —— 后面 ④ 右移靠航向保持走直线。
                 *    见 Turn_AlignTightBlocking 的注释; 由 RESCUE_TURN_* 三个宏控制。 */
                Turn_AlignTightBlocking(RESCUE_HEADING_DEG);
                MLOG("救援①: 车头右转 90° -> 目标航向 %.1f° (已转正+压正, 航向基准已同步)",
                     (double)RESCUE_HEADING_DEG);
                break;
            /* ② 航向校准: 转到绝对 -90°(顺带再确认一次航向基准)。
             *    转移条件 = 转向环自己判“到位”(Chassis_Task_Is_Complete) */
            case STATE_15_TURN_FOR_HOSTAGE:
                /* ⭐ 2026-10-08: 转完 90° 后先挪一步(+75mm 前进)再校准(见 ROUTE_RESCUE_AFTER_TURN_MM)
                 * ⭐⭐ 2026-10-11(用户要求): 这一步加【冲击保护】(限速 + 提前减速),
                 *    防止刚掉完头就全速冲出去过界 —— 见 ROUTE_RESCUE_AFTER_TURN_* 宏。 */
                Route_ImpactGuard(ROUTE_RESCUE_AFTER_TURN_MM,
                                  ROUTE_RESCUE_AFTER_TURN_SPEED_CAP_MMPS,
                                  ROUTE_RESCUE_AFTER_TURN_RAMP_MM);
                Route_MinStep(ROUTE_RESCUE_AFTER_TURN_MM);
                Heading_AlignTo(RESCUE_HEADING_DEG);
                break;
            /* ③ ⭐ 2026-10-07 新增: 校准【到位】之后原地停车再等
             *    RESCUE_ALIGN_SETTLE_MS(3s) 才允许右移进救援区。
             *    目的: 防止“车头刚转到 90° 上下、角速度/车身还在晃”就横移
             *    (那时横移的航向保持会拿残余角当基准 → 越走越斜)。
             *    ⚠️ 顺序是“先到位、后计时”, 不是“最多等 3 秒”。 */
            case STATE_15C_RESCUE_ALIGN_SETTLE:
#if GYRO_BIAS_RESCUE_ON
                /* ⭐ 方案②且 GYRO_BIAS_SCOPE_RESCUE=1: 救援区停稳期间起锚(先做 ZUPT),
                 *    这样后面的 ④ 右移就已经有零偏估计了; 一路用到 STATE_19 停锚。 */
                Chassis_GyroBias_Start();
#endif
                Chassis_Stop();
                s_rescue_align_tick = HAL_GetTick();
                MLOG("救援③: 航向已校准到位, 原地停稳 %dms 再右移 (当前 yaw=%.1f°, 目标 %.1f°)",
                     (int)RESCUE_ALIGN_SETTLE_MS, (double)Chassis_GetYaw(),
                     (double)RESCUE_HEADING_DEG);
                break;
            /* ⭐⭐ 2026-10-09 新增(用户要求): ③ 停稳之后、④ 右移进救援区之前,
             *    再【纯校一次航向】—— 原地右转 90° 可能把车身带偏一点, 转完又挪过
             *    一小步 + 停了 3s, 真正开始横移前再按绝对角校一次更稳。
             *    Route_MinStep(ROUTE_RESCUE_PRE_RIGHT_STEP_MM) 本处填 0
             *    ⇒ 不前进后退, 只纠正陀螺仪(见该宏说明);
             *    Heading_AlignTo(-90) 顺带把航向基准重新同步, 后面右移按 -90° 走直线。 */
            case STATE_15D_RESCUE_RECORRECT:
                MLOG("救援③': 右移前再校一次航向(挪步 %+dmm, 目标 %.1f°)",
                     (int)ROUTE_RESCUE_PRE_RIGHT_STEP_MM, (double)RESCUE_HEADING_DEG);
                Route_MinStep(ROUTE_RESCUE_PRE_RIGHT_STEP_MM);
                Heading_AlignTo(RESCUE_HEADING_DEG);
                break;
            /* ④ 右移进救援区【第一段】(原来这里是“后退 ROUTE_14_TO_HOSTAGE_MM”,
             *    现在整段拆成两段: 这一段走完先在中途校一次航向, 见下面 ④') */
            case STATE_15B_RESCUE_APPROACH_RIGHT:  Chassis_Move_Right(ROUTE_14_MID_MM); break;
            /* ⭐⭐ 2026-10-09 新增(用户要求): 右移中途(还没到救援区)插一次航向校准,
             *    做法与前面几处 ②⑨⑪ 完全一样 —— 【先挪一小步, 再按绝对角校准】:
             *      ① Route_MinStep(ROUTE_RESCUE_MID_STEP_MM = -15): 负值 = 车头【后退】,
             *         而此刻车头是 -90° ⇒ 实际是往【场地左】走 15mm(与右移反向,
             *         等于让这段右移距离少走一点), 顺带让轮子滚起来好转;
             *      ② Heading_AlignTo(RESCUE_HEADING_DEG): 按绝对角校准回 -90°
             *         (顺带重新同步航向基准, 后面第二段右移继续按 -90° 走直线)。
             *    ⚠️ Route_MinStep 内部【阻塞等到位】, 所以这里紧接着发转向是安全的。 */
            case STATE_15B2_RESCUE_MID_CORRECT:
                MLOG("救援④: 右移中途航向校准(Route_MinStep %+dmm + 校准到 %.1f°)",
                     (int)ROUTE_RESCUE_MID_STEP_MM, (double)RESCUE_HEADING_DEG);
                Route_MinStep(ROUTE_RESCUE_MID_STEP_MM);
                Heading_AlignTo(RESCUE_HEADING_DEG);
                break;
            /* ④' 右移【第二段】: 校准完接着走剩下的路, 走完就进救援区(→ ⑤ 停等) */
            case STATE_15B3_RESCUE_APPROACH_RIGHT2: Chassis_Move_Right(ROUTE_14_MID2_MM); break;
            /* ⑤ 停下等 RESCUE_STOP_WAIT_MS: 等车体晃动停稳, 再让机械臂摆出去 */
            case STATE_15A_RESCUE_STOP_WAIT:       Chassis_Stop(); s_rescue_wait_tick = HAL_GetTick(); break;
            /* ⑥ 到达人质处: ⭐⭐2026-10-10(用户要求)【先退一小步】再摆“识别人质”姿态 ——
             *    先后退 ROUTE_RESCUE_GRAB_BACK_MM(默认 -15 = 车头后退 15mm), 再伸臂;
             *    之后 ⑦ 才用 ID1 做巡视/识别/对准(对准结果不受这一步影响, 因为识别全在退完之后)。
             *    ⚠️ 顺序是“先退后伸臂”: 退的时候臂还收着(上一个姿态), 不会剐蹭到人质架。
             *    ⚠️ 本步【阻塞】到走完才发姿态指令 —— 机械臂一定是在车停稳后才伸出去;
             *      用 Mission_Coop_Wait 等(不是 HAL_Delay): 期间照刷陀螺仪 yaw、照收 K230 行,
             *      否则航向保持读到冻结角度。3.5s 超时兜底。
             *    ⚠️ 进入动作只在“刚切到本状态”时执行一次, 所以不会重复退。 */
            case STATE_16_RESCUE_RIGHT_A:
                if (ROUTE_RESCUE_GRAB_BACK_MM != 0) {
                    uint32_t t_gb = HAL_GetTick();

                    MLOG("救援: 到人质处 -> 先挪 %+dmm(%s) 再伸臂识别",
                         (int)ROUTE_RESCUE_GRAB_BACK_MM,
                         (ROUTE_RESCUE_GRAB_BACK_MM > 0) ? "车头前进" : "车头后退");
                    if (ROUTE_RESCUE_GRAB_BACK_MM > 0) {
                        Chassis_Move_Forward(ROUTE_RESCUE_GRAB_BACK_MM);
                    } else {
                        Chassis_Move_Backward(-(ROUTE_RESCUE_GRAB_BACK_MM));
                    }
                    while (!Chassis_Task_Is_Complete() &&
                           (HAL_GetTick() - t_gb) < 3500u) {
                        Mission_Coop_Wait(20);
                    }
                }
                Arm_GotoPose(ARM_POSE_HOSTAGE_LOOK);
                break;
            /* ⑦ 底盘完全不动, 交给救援视觉子状态机(对准 + 抓取) */
            case STATE_20_PERFORMING_HOSTAGE_RESCUE: break;
            /* ⑧⑨⑩⑪ 抓完后的撒退: 右移650 → 航向 → 右移890 → 航向
             * (原来是“后退 ×2”; 车头已右转 90°, 所以右移 = 原来的后退方向)
             * ⭐⭐ 2026-10-10(用户要求): 由 RESCUE_TAIL_ONESHOT 切换(当前 = 1)——
             *    1 = 抓完 → ⑧【原地校一次航向(不挪步)】→ ⑩【一段走完 ⑧+⑩ 的距离】→ ⑫停;
             *    0 = 回到下面老做法(两段各自走完 + 各校一次)。 */
#if RESCUE_TAIL_ONESHOT
            /* ⭐⭐ 2026-10-10(用户要求): 抓完人质 → 【先原地校一次航向(不挪步)】→ 再一段走到终点。
             *    ⑧ 这一步【只校航向】: Route_MinStep(ROUTE_RESCUE_FWD_MM) 当前 = 0 ⇒ 不挪步,
             *       只按绝对角把车头摆正(-90°, 顺带重同步航向基准), 让后面 1614mm 长距离更直;
             *    ⑩ 那一步才是一次性连续右移(⑩+⑫ 合并), 中途不再停车、不再校正。 */
            case STATE_17_RESCUE_RIGHT_B:
                /* ⭐⭐ 2026-10-11(用户要求): 撤退段降速 —— 抱上人质后重心偏, 350mm/s 横移
                 *    刮地扰动大、段末甩尾大; 从这一步起降到 ROUTE_RESCUE_RETURN_SPEED_MMPS,
                 *    到 STATE_19(⑫ 终点)再恢复 ROUTE_DEFAULT_SPEED_MMPS。 */
                Chassis_SetMaxSpeed(ROUTE_RESCUE_RETURN_SPEED_MMPS);
                MLOG("救援⑧: 进入撤退段, 车速降到 %dmm/s(抱人质后减扰动/甩尾)",
                     (int)ROUTE_RESCUE_RETURN_SPEED_MMPS);
#if (GYRO_BIAS_TAIL_ON && !GYRO_BIAS_RESCUE_ON)
                /* ⭐⭐ 2026-10-11(用户要求): 【抓取人质后】才启用陀螺仪零偏方案 ——
                 *    从这一步(⑥ 抓完后的第 1 个状态)起锚, 一路用到终点(⑫ 停锚)。
                 *    起锚时会把软件航向 corrected_yaw 对齐到当前航向, 所以紧接着的
                 *    Heading_AlignTo(-90°) 和后面的右移都以它为基准, 不会跳变。
                 *    ⚠️ 若 GYRO_BIAS_SCOPE_RESCUE=1, 起锚已提前到 STATE_15C,
                 *       这里就不再重复起锚(见 GYRO_BIAS_SCOPE_RESCUE 处的说明)。 */
                Chassis_GyroBias_Start();
#endif
                MLOG("救援: 抓完人质 -> 先原地校一次航向(挪步 %+dmm, 目标 %.1f°)",
                     (int)ROUTE_RESCUE_FWD_MM, (double)RESCUE_HEADING_DEG);
                Route_MinStep(ROUTE_RESCUE_FWD_MM);
                Heading_AlignTo(RESCUE_HEADING_DEG);
                break;
#else
            case STATE_17_RESCUE_RIGHT_B:
                /* ⭐⭐ 2026-10-11(用户要求): 撤退段降速(同上面 RESCUE_TAIL_ONESHOT=1 分支) */
                Chassis_SetMaxSpeed(ROUTE_RESCUE_RETURN_SPEED_MMPS);
                MLOG("救援⑧: 进入撤退段, 车速降到 %dmm/s(抱人质后减扰动/甩尾)",
                     (int)ROUTE_RESCUE_RETURN_SPEED_MMPS);
                Chassis_Move_Right(ROUTE_17_RIGHT_B_MM);
                break;
#endif
            case STATE_17A_RESCUE_HEADING_CORRECT:
                Route_MinStep(ROUTE_RESCUE_FWD_MM);   /* ⭐ 转正前先挪一步(+6 = 车头前进) */
                Heading_AlignTo(RESCUE_HEADING_DEG);
                break;
#if RESCUE_TAIL_ONESHOT
            /* ⭐ 校准完 → 走最后一整段(⑧+⑩ 合并)。
             *    ⭐⭐ 2026-10-11(用户要求): 带一个"车头左方分量"(RESCUE_TAIL_LEFT_COMP)。
             *    ⚠️ 【左】与这段的【右移】是同一根轴的两端 ⇒ 等效于【少走】那么多:
             *       实际右移 = 总距离 × (1 − RESCUE_TAIL_LEFT_COMP)。
             *       例: 0.038 + 1614mm ⇒ 少走 ≈61mm。
             *    另外还留着一个"车头前方分量"(RESCUE_TAIL_FWD_COMP, 默认 0):
             *       那个才是真正横着偏出去(与右移垂直), 用 Chassis_Move_Right_WithBack 传。
             *    ⭐⭐ 2026-10-11(用户要求, 同批): 这段现在是【对半走 + 中点插一次
             *       "航向校正 + 往前进 12mm"】, 剩下的一次性冲到终点
             *       (RESCUE_TAIL_MID_CORRECT_ENABLE=1), 详见该宏处的说明。 */
            case STATE_18_RESCUE_RIGHT_C:
                {
                    int32_t left_mm = (int32_t)((float)RESCUE_TAIL_TOTAL_MM
                                                * RESCUE_TAIL_LEFT_COMP + 0.5f);
                    int32_t net_mm  = RESCUE_TAIL_TOTAL_MM - left_mm;

                    MLOG("救援: 校准完 -> 一路走到底, 右移 %dmm(名义 %dmm, 扣左分量 %dmm, 比例 %.3f)"
                         " + 前向补 %dmm, 中途不停不校正",
                         (int)net_mm, (int)RESCUE_TAIL_TOTAL_MM, (int)left_mm,
                         (double)RESCUE_TAIL_LEFT_COMP,
                         (int)((float)RESCUE_TAIL_TOTAL_MM * RESCUE_TAIL_FWD_COMP + 0.5f));
                    /* ⭐⭐ 2026-10-11: 把"起点航向 + 漂移率 + 坏读数/读失败"打出来 ——
                     *   长横移(≈5s)之后的"车头偏"到底是不是温漂, 看这行就能定性, 见
                     *   Chassis.h 的 drift 说明:
                     *     drift ≲0.05°/s  → 传感器不漂 ⇒ 偏是机械/打滑(该查辊子/轮子);
                     *     drift ≳0.2°/s   → 温漂明显 ⇒ 5s 就白歪 >1°, 该缩短长段或换 9 轴。
                     *   ⚠️ Chassis_Move_Right_WithBack 内部【只用"点车头左方=少走右移"】的
                     *      同一根轴, 不会给车体施加偏航力矩 ⇒ 左分量本身不会让车头偏。 */
                    MLOG("救援: 走这一段之前的 yaw=%.1f°, yaw静止漂移率=%.3f°/s, 坏读数 %lu",
                         (double)Chassis_GetYaw(), (double)Chassis_GetYawDriftDps(),
                         (unsigned long)Chassis_GetYawGlitchCount());
#if RESCUE_TAIL_MID_CORRECT_ENABLE
                    /* ⭐⭐ 2026-10-11(用户要求): 【对半走, 中点插一次"航向校正 + 往前进
                     *    RESCUE_TAIL_MID_STEP_MM(12mm)"】, 剩下的路【一次性冲到终点】。
                     *    这一段是全程最长的一次连续横移(≈1.6m), 走完就结束了, 攒下的偏差
                     *    没有后续校正兜底。详见 RESCUE_TAIL_MID_STEP_MM 处的说明。
                     *    顺序(用户要求): 先走前半 → 校车头 → 再往前进 12mm → 一次走完后半。 */
                    {
                        int32_t half1 = net_mm / 2;
                        int32_t half2 = net_mm - half1;
                        uint32_t t_half = HAL_GetTick();

                        MLOG("救援: 最后一段【前半】右移 %dmm(总 %dmm 对半), 之后在中点校航向 + 前进 %dmm",
                             (int)half1, (int)net_mm, (int)RESCUE_TAIL_MID_STEP_MM);
                        Chassis_Move_Right_WithBack(half1, -(RESCUE_TAIL_FWD_COMP));

                        /* ⚠️ 必须【阻塞等到位】才做中点校正: 没走完就发转向会把它顶掉
                         *    (Chassis_Rotate 内部 s_moving=false), 前半段就白走了。
                         *    用 Mission_Coop_Wait 等(不是 HAL_Delay): 期间照刷 yaw、照收 K230。 */
                        while (!Chassis_Task_Is_Complete() &&
                               (HAL_GetTick() - t_half) < (uint32_t)RESCUE_TAIL_HALF_TIMEOUT_MS) {
                            Mission_Coop_Wait(20);
                        }
                        if (!Chassis_Task_Is_Complete()) {
                            MLOG("⚠ 救援: 最后一段前半超时 %lums(未到位) → 强制停车后继续做中点校正",
                                 (unsigned long)(HAL_GetTick() - t_half));
                            Chassis_Stop();
                        }
                        MLOG("救援: 前半走完, yaw=%.1f°(目标 %.1f°) → 中点校航向",
                             (double)Chassis_GetYaw(), (double)RESCUE_HEADING_DEG);

                        /* ① 校航向(顺带重新同步航向基准, 后半段按新基准走直线)。
                         *    ⚠️ 偏差 ≤ TURN_SKIP_DEG(2°) 时 Heading_AlignTo 会自动跳过转向、
                         *       只同步基准; Chassis_WaitTurnDone 此时立即返回。 */
                        Heading_AlignTo(RESCUE_HEADING_DEG);
                        Chassis_WaitTurnDone(RESCUE_TAIL_MID_ALIGN_TIMEOUT_MS);
                        /* ② 往前进 RESCUE_TAIL_MID_STEP_MM(=+12mm, 车头前进; 此刻车头 -90°
                         *    ⇒ 走的是【场地右方】)。Route_MinStep 内部阻塞到位, 自带日志。 */
                        Route_MinStep(RESCUE_TAIL_MID_STEP_MM);

                        MLOG("救援: 中点校正结束(yaw=%.1f°) → 剩下 %dmm【一次性冲到终点】",
                             (double)Chassis_GetYaw(), (int)half2);
                        Chassis_Move_Right_WithBack(half2, -(RESCUE_TAIL_FWD_COMP));
                    }
#else
                    Chassis_Move_Right_WithBack(net_mm, -(RESCUE_TAIL_FWD_COMP));
#endif
                }
                break;
#else
            /* 老做法(两段): 这里只走第 2 段(⑩) —— 它也是"最后一段", 同样按比例扣左分量 */
            case STATE_18_RESCUE_RIGHT_C:
                {
                    int32_t left_mm = (int32_t)((float)ROUTE_18_RIGHT_C_MM
                                                * RESCUE_TAIL_LEFT_COMP + 0.5f);
                    int32_t net_mm  = ROUTE_18_RIGHT_C_MM - left_mm;

                    MLOG("救援(两段走法): 第 2 段右移 %dmm(名义 %dmm, 扣左分量 %dmm) + 前向补 %dmm",
                         (int)net_mm, (int)ROUTE_18_RIGHT_C_MM, (int)left_mm,
                         (int)((float)ROUTE_18_RIGHT_C_MM * RESCUE_TAIL_FWD_COMP + 0.5f));
                    Chassis_Move_Right_WithBack(net_mm, -(RESCUE_TAIL_FWD_COMP));
                }
                break;
#endif
            case STATE_18A_RESCUE_HEADING_CORRECT:
                /* ⭐ 2026-10-08: 统一成 +6 最小一步(原来这里是 -8 = 车头后退) */
                Route_MinStep(ROUTE_RESCUE_LAST_STEP_MM);
                Heading_AlignTo(RESCUE_HEADING_DEG);
                break;
            /* ⑫ 停下 → 结束(转移里置 MISSION_STATE_COMPLETE) */
            case STATE_19_RESCUE_RIGHT_D:
                /* ⭐⭐ 2026-10-11(用户要求): 撤退段降速的【恢复点】—— 任务已到终点, 把车速
                 *    放回默认, 免得影响下次任务/手动模式(见 ROUTE_RESCUE_RETURN_SPEED_MMPS)。 */
                Chassis_SetMaxSpeed(ROUTE_DEFAULT_SPEED_MMPS);
                MLOG("救援⑫: 任务完成, 车速恢复默认 %dmm/s", (int)ROUTE_DEFAULT_SPEED_MMPS);
#if GYRO_BIAS_TAIL_ON
                Chassis_GyroBias_Stop();   /* ⭐ 2026-10-11: 任务走完, 停锚恢复默认(trim 步长也回默认) */
#endif
                break;

            /* ⚠️ 下面这几个状态本流程不再经过(留空防误入) */
            case STATE_16A_RESCUE_HEADING_CORRECT: break;   /* 未使用 */
            case STATE_19A_RESCUE_HEADING_CORRECT: break;   /* 未使用 */
            case STATE_21_RESCUE_RIGHT_E:          break;   /* 未使用 */
            case STATE_21A_RESCUE_HEADING_CORRECT: break;   /* 未使用 */
            case STATE_22_RESCUE_RIGHT_F:          break;   /* 未使用 */

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

        /* 扫码状态(适配最新 K230 main.py + yolo_main.py):
         *   K230 上电后先在 main.py 里用自己的摄像头扫码, 扫到 3 位目标号后:
         *     ① 存进 K230 本地 /sdcard/target.txt(后续识别用它选目标类别);
         *     ② 通过 UART 回一行 "SCAN_OK";
         *     ③ 自己重启进入 yolo_main(从这之后才响应 run_task / start_align)。
         *   ⇒ 新 K230 【不响应 "scan_qr"】, 所以这里【不发任何扫码请求】,
         *      只等 "SCAN_OK"(兼容旧版的 "qr:<data>"); 超时(QR_WAIT_TIMEOUT_MS)
         *      则继续往下走, 防止整场卡死在扫码点。
         *   - 测试模式(NO_VISION): 停车 1s 后模拟二维码 "123" 并推进 */
        case STATE_2_PERFORMING_QR_SCAN:
#if MISSION_TEST_NO_VISION
            /* 测试: 停车 1s 再模拟扫码推进, 否则进入状态后立即跳到 STATE_3, 看不到停车 */
            if (!g_vision_task_in_progress) {
                qr_scan_start = HAL_GetTick();
                g_vision_task_in_progress = 1;
            } 
            else if (HAL_GetTick() - qr_scan_start >= QR_SIM_WAIT_MS) {
                strcpy(g_qr_code_string, "123");
                MLOG("扫码(测试模拟): %s", g_qr_code_string);
                g_vision_task_in_progress = 0;
                Arm_GotoPose(ARM_POSE_SCAN_RESET);   /* 扫码之后: 机械臂收回 */
                g_mission_state++;
            }
#else
            {
                char line[K230_LINE_MAX];

                /* ⭐ 多角度扫码: 每周期试一下能不能换下一个角度。
                 *    收到 SCAN_OK 时本状态马上就会结束, 所以这里多跑一次无害
                 *    (后续的 Arm_GotoPose(SCAN_RESET) 会直接覆盖 ID4)。 */
                Arm_Scan_Poll();

                if (Mission_GetNewLine(line, sizeof(line))) {
                    g_vision_task_in_progress = 0;
                    if (strncmp(line, "SCAN_OK", 7) == 0) {
                        /* 新 K230: 只通知“扫到了”, 不再回传二维码内容
                         * (目标号由 K230 自己保存并用于选目标类别) */
                        strcpy(g_qr_code_string, "OK");
                        MLOG("扫码: 收到 SCAN_OK (K230 扫码完成)");
                    } else {
                        const char *p = line;
                        if (strncmp(p, "qr:", 3) == 0) p += 3;   /* 兼容旧版 "qr:xxx" */
                        strncpy(g_qr_code_string, p, sizeof(g_qr_code_string) - 1);
                        g_qr_code_string[sizeof(g_qr_code_string) - 1] = '\0';
                        MLOG("扫码: 收到旧版回传 %s", g_qr_code_string);
                    }
                    Arm_GotoPose(ARM_POSE_SCAN_RESET);          /* 扫码之后: 机械臂收回 */
                    g_mission_state++;
                } else if (!g_vision_task_in_progress) {
                    g_vision_task_in_progress = 1;
                    s_qr_cmd_tick = HAL_GetTick();
                    MLOG("扫码: 正在等 K230 回 SCAN_OK ...");
                } else if (HAL_GetTick() - s_qr_cmd_tick >= QR_WAIT_TIMEOUT_MS) {
                    MLOG("扫码: 等 SCAN_OK 超时(%dms), 继续往下走(防卡死); "
                         "距上次收到K230数据 %lums",
                         (int)QR_WAIT_TIMEOUT_MS, (unsigned long)K230_RxSilenceMs());
                    /* ⚠️ 必须把任务标志清 0: 否则后面所有视觉任务都会误认为
                     * “已经有任务在跑” → Vision_SendTask 不再打印启动日志、
                     * 不 K230_FlushAll()、也不开【换任务静默期】→ 上个任务的
                     * 残留帧(SCAN_OK / D:../OK)会一直留在队列里, 被下个任务
                     * 当成自己的结果误读(实测桶阶段读到球阶段的 D:0,3xx / OK)。 */
                    g_vision_task_in_progress = 0;
                    K230_FlushAll();   /* 把迟到的 SCAN_OK / 残留帧一起清掉 */
                    Arm_GotoPose(ARM_POSE_SCAN_RESET);
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

        /* STATE_7 左移B(⭐⭐ 2026-10-10 改由 ROUTE_7_LEFT_B_SPLIT_ENABLE 切换, 当前 = 0):
         *    开关=0(当前): 一整段走完 → 直接进中间停顿(下面两个状态不会被进入);
         *    开关=1      : 前半 → B段中间前进 → 后半 → 中间停顿, 每一步到位就顺序推进(++),
         *                  所以枚举里这三个状态必须【紧挨着且顺序一致】(见 MissionControl.h) */
#if ROUTE_7_LEFT_B_SPLIT_ENABLE
        case STATE_7_MOVE_LEFT_B:         if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        case STATE_7B_MOVE_FWD_MID:       if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        case STATE_7_MOVE_LEFT_B2:        if (Chassis_Task_Is_Complete()) g_mission_state++; break;
#else
        /* 一整段左移: 走完直接进中间停顿(跳过中间那两步) */
        case STATE_7_MOVE_LEFT_B:         if (Chassis_Task_Is_Complete()) g_mission_state = STATE_7A_INTERMEDIATE_STOP; break;
        case STATE_7B_MOVE_FWD_MID:       g_mission_state = STATE_7A_INTERMEDIATE_STOP; break;   /* 防误入 */
        case STATE_7_MOVE_LEFT_B2:        g_mission_state = STATE_7A_INTERMEDIATE_STOP; break;   /* 防误入 */
#endif
        case STATE_7A_INTERMEDIATE_STOP:  if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        /* 左移C完成后不是 ++, 而是跳回航向校正状态 */
        case STATE_7B_MOVE_LEFT_C:        if (Chassis_Task_Is_Complete()) g_mission_state = STATE_7A_HEADING_CORRECTION; break;// g_mission_state++; break;//
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

        /* ⭐ 排爆后那次航向校正: 挪步是阻塞的, 所以这里等转向到位即可 */
        case STATE_11A_HEADING_CORRECT:   if (Chassis_Task_Is_Complete()) g_mission_state++; break;

        /* ---------- 打靶走位: 右移 → (中间校正) → 最后校正 → 停稳 → 摆TARGET_LOOK ----------
         * ⭐⭐ 2026-10-10(用户要求): 由两个开关切换(见 ROUTE_12_SPLIT_ENABLE / _MID_CORRECT_ENABLE):
         *   SPLIT=1          : 老走法, 每半段走完都校一次 = 4 次:
         *                      MOVE_A → CORRECT_A → MOVE_A2 → CORRECT_A2
         *                      → MOVE_B → CORRECT_B → MOVE_B2 → CORRECT_B2
         *   SPLIT=0, MID=1(当前): 一条长路对半走, 中间校一次:
         *                      MOVE_A(前半) → CORRECT_B(挪一小步 + 校一次)
         *                      → MOVE_B(后半) → CORRECT_B2(最后一次 + 停稳)
         *   SPLIT=0, MID=0   : 一条路直接走完, 中途不校正, 只剩 CORRECT_B2 那一次。
         * 每次校正之前都会先挪一小步(各处各有自己的宏, 见 ROUTE_12_A_MINSTEP_MM 那一段) */
#if ROUTE_12_SPLIT_ENABLE
        case STATE_12_PART1_MOVE_A:       if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        case STATE_12_PART1_CORRECT_A:    if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        case STATE_12_PART1_MOVE_A2:      if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        case STATE_12_PART1_CORRECT_A2:   if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        case STATE_12_PART1_MOVE_B:       if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        case STATE_12_PART1_CORRECT_B:    if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        case STATE_12_PART1_MOVE_B2:      if (Chassis_Task_Is_Complete()) g_mission_state++; break;
#else
        /* 一条路走法: 前半 → [中间那次校正] → 后半 → 最后一次校正 */
        case STATE_12_PART1_MOVE_A:
            if (Chassis_Task_Is_Complete()) {
                g_mission_state = ROUTE_12_MID_CORRECT_ENABLE ? STATE_12_PART1_CORRECT_B
                                                              : STATE_12_PART1_CORRECT_B2;
            }
            break;
        case STATE_12_PART1_CORRECT_B:
            if (Chassis_Task_Is_Complete()) g_mission_state = STATE_12_PART1_MOVE_B;
            break;
        case STATE_12_PART1_MOVE_B:
            if (Chassis_Task_Is_Complete()) g_mission_state = STATE_12_PART1_CORRECT_B2;
            break;
        /* ⚠️ 下面 4 个在"一条路"走法里【不会被进入】(留空防误入: 万一进去了也直接推到下一步,
         *    不会原地卡住) */
        case STATE_12_PART1_CORRECT_A:    g_mission_state = STATE_12_PART1_CORRECT_B; break;
        case STATE_12_PART1_MOVE_A2:      g_mission_state = STATE_12_PART1_CORRECT_B; break;
        case STATE_12_PART1_CORRECT_A2:   g_mission_state = STATE_12_PART1_CORRECT_B; break;
        case STATE_12_PART1_MOVE_B2:      g_mission_state = STATE_12_PART1_CORRECT_B2; break;
#endif
        /* CORRECT_B2 = 走位链最后一次航向校正: 除了等它转到位
         * (Chassis_Task_Is_Complete), 还要额外原地停稳 TARGET_STOP_SETTLE_MS,
         * 免得底盘还在晃就把臂/摄像头伸出去。
         * (TARGET_STOP_SETTLE_MS 填 0 就回到“转完立刻往下走”的行为) */
        case STATE_12_PART1_CORRECT_B2:
            if (Chassis_Task_Is_Complete() &&
                (HAL_GetTick() - s_target_stop_tick) >= TARGET_STOP_SETTLE_MS) {
                g_mission_state++;
            }
            break;
        /* ⭐ 摆完 TARGET_LOOK(阻塞约 4.5s)后才进打靶视觉子状态机 */
        case STATE_12_PART1_MOVE_C:       if (Chassis_Task_Is_Complete()) g_mission_state = STATE_13_PERFORMING_TARGETING; break;
        case STATE_12_TURN_A:             break;   /* 未使用 */
        case STATE_12_TURN_B:             break;   /* 未使用 */
        case STATE_12_PART2_MOVE_A:       if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        case STATE_12_PART2_MOVE_BACKWARD: if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        /* ⭐ 打靶收尾: 右移(到拐角) + 后退50 到位, 且校完 0° 后, 进救援阶段去转 90° */
        case STATE_12_PART2_CORRECT_A:    if (Chassis_Task_Is_Complete()) g_mission_state = STATE_14_MOVE_FORWARD_B; break;
        case STATE_12_PART2_MOVE_B:       break;   /* 已废弃 */
        case STATE_12_PART2_CORRECT_B:    break;   /* 已废弃 */
        /* 打靶: 进入视觉辅助子状态机(内部处理完会跳到 STATE_12_PART2_MOVE_A 走打靶收尾) */
        case STATE_12_PART2_MOVE_C:       break;   /* 已废弃 */
        case STATE_13_PERFORMING_TARGETING: Handle_Vision_Alignment(2); break;

        case STATE_14_MOVE_FORWARD_B:     if (Chassis_Task_Is_Complete()) g_mission_state++; break;   /* ① 转右90° → ② */
        /* ② 航向校准【到位】→ ③ 原地停稳 3s */
        case STATE_15_TURN_FOR_HOSTAGE:
            if (Chassis_Task_Is_Complete()) {
                g_mission_state++;
            }
            break;

        /* ③ 校准到位后原地停稳 RESCUE_ALIGN_SETTLE_MS, 时间到 → ④ 右移进救援区
         *    (打靶收尾那三段平移已经在转之前走完了, 见 STATE_12_PART2_*) */
        case STATE_15C_RESCUE_ALIGN_SETTLE:
            if ((HAL_GetTick() - s_rescue_align_tick) >= RESCUE_ALIGN_SETTLE_MS) {
                g_mission_state++;
            }
            break;

        /* ③' ⭐ 右移前那次纯航向校正: Route_MinStep(0) 直接返回, Heading_AlignTo 非阻塞
         *    ⇒ 转移条件等转向到位(与其它几处航向校正同一写法) */
        case STATE_15D_RESCUE_RECORRECT:
            if (Chassis_Task_Is_Complete()) g_mission_state++;
            break;

        /* ④ 右移【第一段】ROUTE_14_MID_MM 到位 → ④' 中途的“挪一步 + 航向校准” */
        case STATE_15B_RESCUE_APPROACH_RIGHT:
            if (Chassis_Task_Is_Complete()) g_mission_state++;
            break;

        /* ⭐ ④' 中途校准: Route_MinStep 是阻塞的(内部已等到位), Heading_AlignTo 非阻塞
         *    ⇒ 转移条件等转向到位即可(与 ②⑨⑪ 三处写法完全一致) */
        case STATE_15B2_RESCUE_MID_CORRECT:
            if (Chassis_Task_Is_Complete()) g_mission_state++;
            break;

        /* ④'' 右移【第二段】ROUTE_14_MID2_MM 到位 → ⑤ 停下等 3s */
        case STATE_15B3_RESCUE_APPROACH_RIGHT2:
            if (Chassis_Task_Is_Complete()) g_mission_state++;
            break;

        /* ⑤ 原地停等 RESCUE_STOP_WAIT_MS, 时间到 → ⑥ 摆 HOSTAGE_LOOK */
        case STATE_15A_RESCUE_STOP_WAIT:
            if ((HAL_GetTick() - s_rescue_wait_tick) >= RESCUE_STOP_WAIT_MS) {
                g_mission_state++;
            }
            break;

        /* ---------- 阶段四(救援): 掉头 → 停稳 → 右移 → 对准/抓取 → 右移 x2 + 航向校准 x2 ---------- */
        /* ⑥ 到人质处: 进入动作里已“先退 ROUTE_RESCUE_GRAB_BACK_MM + 摆 HOSTAGE_LOOK”
         *    (都是阻塞的, 返回时已完成) → 直接跳到 ⑦ 视觉识别/对准 */
        case STATE_16_RESCUE_RIGHT_A:
            if (Chassis_Task_Is_Complete()) g_mission_state = STATE_20_PERFORMING_HOSTAGE_RESCUE;
            break;
        /* ⑦ ID1 巡视/识别 + 精对准 + 抓取; 完成后子状态机自己跳到 STATE_17_RESCUE_RIGHT_B */
        case STATE_20_PERFORMING_HOSTAGE_RESCUE: Handle_Vision_Alignment(3); break;
#if RESCUE_TAIL_ONESHOT
        /* ⭐ ⑧ 只校航向(不移动) → 校完直接进 ⑩(⑩ 那里走"合并后的一整段");
         *    ⑩ 一整段走完 → 直接进 ⑫ 停下。★本开关=1 时 ⑨/⑪ 不会被进入。 */
        case STATE_17_RESCUE_RIGHT_B:          if (Chassis_Task_Is_Complete()) g_mission_state = STATE_18_RESCUE_RIGHT_C; break;
        case STATE_17A_RESCUE_HEADING_CORRECT: g_mission_state = STATE_18_RESCUE_RIGHT_C; break;   /* 防误入 */
        case STATE_18_RESCUE_RIGHT_C:          if (Chassis_Task_Is_Complete()) g_mission_state = STATE_19_RESCUE_RIGHT_D; break;
        case STATE_18A_RESCUE_HEADING_CORRECT: g_mission_state = STATE_19_RESCUE_RIGHT_D; break;   /* 防误入 */
#else
        case STATE_17_RESCUE_RIGHT_B:          if (Chassis_Task_Is_Complete()) g_mission_state++; break;   /* ⑧ → ⑨ */
        case STATE_17A_RESCUE_HEADING_CORRECT: if (Chassis_Task_Is_Complete()) g_mission_state++; break;   /* ⑨ → ⑩ */
        case STATE_18_RESCUE_RIGHT_C:          if (Chassis_Task_Is_Complete()) g_mission_state++; break;   /* ⑩ → ⑪ */
        case STATE_18A_RESCUE_HEADING_CORRECT: if (Chassis_Task_Is_Complete()) g_mission_state++; break;   /* ⑪ → ⑫ */
#endif
        /* ⑫ 最后一步: 停下, 整个任务完成 */
        case STATE_19_RESCUE_RIGHT_D:
            MLOG("全部任务完成.");
            Chassis_Stop();
            g_mission_state = MISSION_STATE_COMPLETE;
            break;

        /* ⚠️ 下面这几个状态本流程不再经过(留空防误入) */
        case STATE_16A_RESCUE_HEADING_CORRECT: break;
        case STATE_19A_RESCUE_HEADING_CORRECT: break;
        case STATE_21_RESCUE_RIGHT_E:          break;
        case STATE_21A_RESCUE_HEADING_CORRECT: break;
        case STATE_22_RESCUE_RIGHT_F:          break;

        default: break;   /* IDLE / COMPLETE 等状态无转移条件 */
    }
}
