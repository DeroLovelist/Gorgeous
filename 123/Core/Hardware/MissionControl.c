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
 * 与 K230 的串口协议(需与 K230 端一致, 见 yolo_main3.py):
 *   MCU -> K230 : "scan_qr"         请求扫码
 *               : "run_task:<n>"    1=球(抓小球) 2=靶(打靶) 3=桶(放球)
 *                                   救援=形状(圆柱/腰鼓/圆台), K230 端待实现
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

/* 机械臂动作间停顿 (非专业人员可先不改)
 * hold_time: 相邻两个姿态之间的停顿(ms), 让动作完成后车/臂稳定再走下一步。
 *   取值建议 300~1000, 默认 500。
 * 每个姿态自己的“运动时间”见下面 s_arm_pose_time[]。 */
#if !MISSION_TEST_NO_ARM
static uint16_t hold_time = 500;    /* 机械臂动作间停顿(ms) */
#endif

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
/* ⚠️ 先看两个硬约束(不知道会白调):
 *   ① 底盘有个“最小有效步长”: Chassis.h 的 CH_POS_THRESHOLD_COUNT = 40 计数
 *      ≈ 6mm(CH_COUNTS_PER_MM = 1560/(75π) ≈ 6.62), 即单轮残差 < 6mm 就判“到位”。
 *      ⇒ MIN_MOVE_MM 必须【明显大于 6mm】, 否则指令下去车根本不动。
 *   ② 但步长也不能大: 每次修正都冲过头 → 左右来回横移、永远进不了 ALIGN_TOLERANCE。
 *      “一直接近不了”通常就是 K_GAIN + MIN_MOVE_MM 偏大造成的。 */
#define K_GAIN              0.25f       /* 像素误差 → 移动距离 的比例 (mm/像素)。
                                         * 太大→每帧都冲过头来回振荡(就是“一直左右横移
                                         * 接近不了”的典型原因); 太小→老修不到位。
                                         * ⭐ 标定: 人为把目标挪开已知距离 D(mm), 读 K230
                                         *    回的 |err_x|(px) → 实际比例 a = D/|err_x|
                                         *    (mm/px), 然后取 K_GAIN ≈ 0.5×a(留一半余量)。
                                         * 默认 0.25(原来是 0.5, 偏大) */
#define MIN_MOVE_MM         10          /* 单次修正的最小距离(mm): “暴力起步”用。
                                         * 必须 > 底盘到位死区(≈6mm)才真能动起来;
                                         * 建议 10~20。原来 35 → 只能左右荡 */
#define MAX_MOVE_MM         60          /* 单次修正的最大距离(mm), 防止一开始误差很大时一下
                                         * 冲太远。建议 40~100。原来 120 */
#define ALIGN_TOLERANCE     50          /* 判定“已对准”的像素误差容差。
                                         * 建议 30~80。越小对准越准但越难达成(可能超时) */

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
#define FB_MIN_MM           10          /* 前后单步最小距离(mm): 必须 > 底盘死区≈6mm,
                                         * 否则指令下去车不动。建议 10~15 */
#define FB_MAX_MM           20          /* 前后单步最大距离(mm): 比 MAX_MOVE_MM(60)
                                         * 小得多, 就是防止“一下冲过头”。
                                         * 建议 15~30 */

/* ---- 桶阶段专用的前后步长(单独一份, 只调桶不影响其它阶段) ----
 * 想让桶的前后更小/更大, 只改这三行即可。 */
#define BUCKET_FB_SCALE_PCT FB_SCALE_PCT    /* 桶: 前后步长比例(%) */
#define BUCKET_FB_MIN_MM    FB_MIN_MM       /* 桶: 前后单步最小(mm) */
#define BUCKET_FB_MAX_MM    FB_MAX_MM       /* 桶: 前后单步最大(mm) */

/* =====================================================================
 * ⭐⭐ 视觉方向映射 (关键! 三个阶段“画面左右”对应车体的哪个方向)
 * ---------------------------------------------------------------------
 * 【为什么要管这个】摄像头装在机械臂上, 底座(ID1)转到不同姿态时,
 *   “画面里的左/右” 对应到车体的方向是会变的:
 *
 *   阶段    底座 ID1   相对“球姿态(232)”  画面横向 = 车体
 *   ------  --------  ------------------  -------------------
 *   球      232       0°(基准)            左 / 右   (方向正常)
 *   靶      158       ≈ -6°               —         打靶已改为「只转底座 ID1」
 *   桶      2243      ≈ +177°(≈180°)      左 / 右 【镜像】
 *   救援    1190      ≈ +84°(≈90°)        前 / 后   ← 注意是【前后】!
 *
 *   ⇒ 桶的姿态下: 摄像头报“L(画面左)”时, 车要往【右】移;
 *     救援的姿态下: 摄像头的“左右”就是车的“前后”。
 *     (之前“越修越偏”、“一直左右横移接近不了”就是没区分这个)
 *   ⇒ 打靶(2026-10-04 起)不再动底盘: 画面 L/R 用来小步转机械臂底座 ID1,
 *     方向盘子见上面“任务2 打靶参数”里的 TARGET_ID1_LR_SIGN。
 * ---------------------------------------------------------------------
 * 精对准轴向方案开关:
 *   ALIGN_AXIS_SCHEME_1 = 1 (方案一, 【当前使用】):
 *       排爆(球/桶)、打靶: 精对准【只修横向】
 *       救援            : 精对准【只修前后】(此时画面横向就是车的前后)
 *   ALIGN_AXIS_SCHEME_1 = 0 (方案二):
 *       三个阶段的精对准【前后左右都修】(两个轴都参与)
 *   两种方案都保留 K230 的 start_align/D:x,y 精对准, 只是限制“允许修哪个轴”。
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
 *    因为 Chassis_Move_*(0) 并不是空操作。 */
#define BOMB_C_APPROACH_MM          0//15    /* 目标在正前: 直行接近 */
#define BOMB_L_ADJUST_MM            10//120   /* 目标偏左: 左移调整 */
#define BOMB_R_ADJUST_MM            10//120   /* 目标偏右: 右移调整 */

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
 *    (原来这里用 BUCKET_C_APPROACH_MM 往前走一点, 已按实测删掉) */
#define BUCKET_L_ADJUST_MM          20//60    /* 画面报 L: 小车右移这个距离 */
#define BUCKET_R_ADJUST_MM          20//60    /* 画面报 R: 小车左移这个距离 */

/* ================= 救援任务参数 (2026-10-05 重新定义) ================= */
/* ⭐ 救援“对准”方案开关(两套代码都保留, 改一个数就切):
 *   1 = 方案二【当前使用】: 【小车底盘完全不动】, 按 K230 回的 L/R 小步转
 *       机械臂底座 ID1; 收到 C 就认为对准 → 直接抓取。
 *       (适合“摄像头和夹爪装在同一个可转底座上, 转底座就能同时把镜头
 *        和夹爪对准人质”的机械结构)
 *   0 = 方案一: 与排爆球/桶一样 —— L/R 让【小车前进/后退】一小段
 *       (RESCUE_ADJUST_MM), 再发 start_align 用 D:x,y 精对准
 *       (只修前后, 见 s_align_rescue)。
 * ⚠️ 实测哪种都不对就换另一个值重新编译, 不用改其它代码。
 * ⚠️ 方向约定(见文件头“视觉方向映射”): 臂在 HOSTAGE_LOOK(底座≈1190,
 *    比球姿态多转≈90°) → 画面的“左/右”对应车体的【前/后】。
 *      方案一: L → 小车前进;  R → 小车后退;  C → 不动
 *      方案二: L → ID1 朝一个方向转一步; R → 反向; C → 对准 */
#define RESCUE_SCHEME_ID1       1

/* ---- ③ 停下等待时间(ms): 后退 + 航向校正好之后先停一会儿,
 *      等车体晃动停下来再让机械臂摆出去, 免得抓的时候还在晃 ---- */
#define RESCUE_STOP_WAIT_MS     3000

/* ---- 【方案二】转 ID1 专用参数(和打靶那套完全同构) ---- */
#define RESCUE_ID1_STEP         60      /* 每收到一次 L/R, 底座 ID1 转多少角度码
                                         * (4096 码 = 360°, 60 码 ≈ 5.3°)。
                                         * 建议 30~120 */
#define RESCUE_ID1_LR_SIGN    (-1)      /* “收到 L”时 ID1 的增量符号:
                                         *  -1 = 数值减小; +1 = 数值增大。
                                         * ⚠️ 和打靶一样, 实测转反了只改这个 */
#define RESCUE_ID1_MOVE_MS      300     /* ID1 每步转动时间(ms);
                                         * 这期间把 K230 旧帧全丢掉 */
#define RESCUE_ID1_POS_MIN      0       /* ID1 行程限幅(角度码), 防越界堵转 */
#define RESCUE_ID1_POS_MAX      4095
#define RESCUE_ID1_STEP_MAX     20      /* 🛡防卡死①: 最多转这么多步就强制认为对准 */
#define RESCUE_ID1_TIMEOUT_MS   20000   /* 🛡防卡死②: 整个 L/R 环节最长等这么久(ms),
                                         * 超时强制去抓取(含 K230 完全不回应) */

/* ---- 【方案一】动底盘专用: L/R 时车前进/后退的距离(mm) ---- */
#define RESCUE_ADJUST_MM        15      /* 建议 20~80 */

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
#define TARGET_ID1_STEP         60      /* 每收到一次 L/R, 底座 ID1 转多少角度码。
                                         * 4096 码 = 360°, 所以 1° ≈ 11.4 码,
                                         * 60 码 ≈ 5.3°。建议 30~120:
                                         *   太大 → 一步冲过头、来回摆;
                                         *   太小 → 对准慢、步数多 */
#define TARGET_ID1_LR_SIGN    (-1)      /* “收到 L”时 ID1 的增量符号:
                                         *  -1 = 数值减小 = 向左(当前实测值)
                                         *  +1 = 数值增大 = 向左(装反了就改这个) */
#define TARGET_ID1_MOVE_MS      300     /* ID1 每步的转动时间(ms)。这期间主循环
                                         * 【忽略并丢弃】K230 新帧, 等舵机停稳再取
                                         * 下一帧; 否则会拿“转之前”的旧画面连续累加
                                         * → 直接冲过头。建议 200~500, 要与上面
                                         * TARGET_ID1_STEP 匹配(转得多就要等得久) */
#define TARGET_ID1_POS_MIN      0       /* ID1 行程限幅下限(角度码), 防越界堵转 */
#define TARGET_ID1_POS_MAX      4095    /* ID1 行程限幅上限(角度码) */
#define TARGET_ID1_STEP_MAX     20      /* 🛡防卡死①: 最多转这么多步就强制认为已对准。
                                         * 20 步 × 60 码 = 1200 码 ≈ 105° */
#define TARGET_ID1_TIMEOUT_MS   20000   /* 🛡防卡死②: L/R 调整环节最长等这么久(ms),
                                         * 超时强制走 C 流程(摆 FIRE→LIFT→SCAN_RESET);
                                         * 也兜住“K230 一条都不回”的情况。
                                         * 建议 10000~30000 */
/* ⭐ 2026-10-03: 本工程【不再用单片机控制激光】。
 *    激光由 K230(摄像头模块)自己控制, 单片机只负责“把机械臂摆到
 *    ARM_POSE_TARGET_FIRE 并等运动时间 + hold_time 过去”, 然后摆 TARGET_LIFT。
 *    (原来的 LASER_FIRE_DURATION_MS / Laser_On() / Laser_Off() 已从任务2 移除;
 *     Mission_Init() 里的 Laser_Off() 保留, 只为保证上电时激光是关的) */

/* =====================================================================
 * 路线距离/角度宏 (单位: 距离 mm, 角度 度)
 * —— 想改“某一段走多远/转多少”, 改这里即可, 无需翻下面状态机。
 * ===================================================================== */

/* ---------- 阶段一: 扫码区走位 ---------- */
#define ROUTE_1_TO_QR_MM            625//622    /* 起点 → 二维码扫描点(直行) */
#define ROUTE_3_LEFT_A_MM           557//563    /* 扫码后左移 A 段 */
#define ROUTE_4_DIAG_FWD_MM         110//111 97     /* 左上斜跑: 前进分量(≈45°斜走) */
#define ROUTE_4_DIAG_LEFT_MM        110//111 97     /* 左上斜跑: 左移分量(≈45°斜走) */

/* ---------- 过斜坡段 ---------- */
#define ROUTE_5_TO_RAMP_MM          892    /* 斜坡前直行距离 */
#define ROUTE_7_LEFT_B_MM           875//450    /* 左移 B 段 */
#define ROUTE_7_LEFT_C_MM           403    /* 左移 C 段 */

/* ---------- 排爆区走位 ---------- */
#define ROUTE_8_TO_BOMB_AREA_MM     995//988//940    /* 直行进入排爆区 */
#define ROUTE_8B_RIGHT_MM           660//620    /* 右移微调 */
#define ROUTE_9_TURN_DEG            (0.1)  /* 转向排爆点(相对角度, 负=右转) */
#define ROUTE_9A_LEFT_MM            0      /* 转向后左移微调 */
#define ROUTE_10_APPROACH_MM        0     /* 接近炸弹最后一段直行 */

/* ---------- 打靶路线 (2026-10-04 改版后只剩两段真正在用) ---------- */
#define ROUTE_12_P1_A_MM            850    /* ⭐ 打靶第 1 段右移(mm): 排爆结束后从桶边右移这么多,
                                            * 然后做一个航向校正 */
#define ROUTE_12_P1_B_MM            850    /* ⭐ 打靶第 2 段右移(mm): 航向校正完再右移这么多 */
#define ROUTE_12_P1_C_MM            400    /* (未使用: MOVE_C 改成只摆 TARGET_LOOK, 不再走位) */
#define ROUTE_12_TURN_A_DEG         400    /* (未使用: TURN_A/TURN_B 已不在流程里) */
#define ROUTE_12_P2_A_MM            660/*700*/    /* ⭐ 打靶结束后的收尾右移距离(mm):
                                            * 打完靶、手臂收回 SCAN_RESET 之后右移这么多,
                                            * 再航向校正一次就进入救援阶段 */
#define ROUTE_12_P2_B_MM            200    /* (已废弃) */
#define ROUTE_12_P2_C_MM            200    /* (已废弃) */

/* ⭐ 打靶走位: 第 2 段右移结束后的“停车停稳延时”(ms)
 * 作用状态 = STATE_12_PART1_CORRECT_B。
 * 为什么需要: 这个状态的进入动作只有 Chassis_Stop(), 而转移条件
 *   Chassis_Task_Is_Complete() 在 Stop 之后【立刻】就为真 —— 所以它本来
 *   会被“穿过”(同一周期直接跳到下一步摆 TARGET_LOOK), 等于没停。
 *   加了这个延时: 停车 → 原地停稳这么久 → 再让臂/摄像头伸出去。
 *   免得底盘刹车/麦轮摆动还没停, 机械臂就带着摄像头一起晃。
 * 建议 0(关闭, 回到“穿过”行为) ~ 1500; 默认 800。 */
#define TARGET_STOP_SETTLE_MS   800

/* ---------- 救援(掉头) ---------- */
#define ROUTE_14_TO_HOSTAGE_MM      800    /* 救援前前进距离 */
// #define ROUTE_15_TURN_180_DEG       (-91) /* 原地掉头 180° */

/* ---------- 救援阶段 (2026-10-05 重新定义) ----------
 *  ①后退 ROUTE_14_TO_HOSTAGE_MM → ②航向校准 → ③停等 RESCUE_STOP_WAIT_MS
 *  → ④摆 HOSTAGE_LOOK → ⑤视觉对准 + 抓取
 *  → ⑥后退 ROUTE_17_RIGHT_B_MM → ⑦航向校准
 *  → ⑧后退 ROUTE_18_RIGHT_C_MM → ⑨航向校准 → ⑩停下 */
#define ROUTE_16_RIGHT_A_MM         300    /* (未使用: 该状态已改成“只摆 HOSTAGE_LOOK”) */
#define ROUTE_17_RIGHT_B_MM         600    /* ⭐ 抓完后第 1 段后退(mm) */
#define ROUTE_18_RIGHT_C_MM         600    /* ⭐ 抓完后第 2 段后退(mm) */
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
//id2限幅（50~2300）id3限幅（900~3100）id4限幅（1050~3010）id5限幅（25张开~600闭合）
static uint16_t s_arm_pose_table[ARM_POSE_COUNT][SERVO_COUNT] = {
    /*  名称             ID1   ID2   ID3   ID4   ID5  */
    {  227, 2274, 810, 1413, 93  },   /* HOME          复位(运行时取自 ServoArm, 此行不生效) */
    {  212,  656,1760, 2175, 93  },   /* SCAN          扫码: 车停稳后伸臂给摄像头 */
    {  222, 1843, 878, 1834, 93  },   /* SCAN_RESET    扫码之后复位: 扫到码后把机械臂收回 */
    {  232, 1026,1377, 1176, 93  },   /* BALL_LOOK     看球: 摄像头对准小球(抓球前对准) */
    {  228,  406,1855, 1400, 93  },  /* BALL_PRE       抓夹移动到小球前 */
    {  228,  406,1855, 1400, 600 },   /* BALL_CLOSE    夹爪夹紧小球 */
    {  232, 1160,1795, 1748, 600 },   /* BALL_LIFT     抓到小球后大臂抬起 */
    { 2243,  886,1872, 1650, 600 },   /* BUCKET_CARRY  携带姿态: 端着球, 底盘移动到另一侧 */
    { 2243, 1254,1604, 1131, 600 },   /* BUCKET_LOOK   看桶: 摄像头对准球桶(放置前对准) */
    { 2243,  197,1719, 2594, 600 },   /* PLACE_PRE     机械臂移动到放置小球的位置 */
    { 2243,  197,1719, 2594, 93  },   /* PLACE_OPEN    夹爪松开(放球) */
    { 2168, 1285,2155, 2234, 93  },   /* PLACE_LIFT    放置完之后大臂抬起 */
    {  213, 1925, 988, 1463, 93  },   /* TARGET_READY     转动到准备识别靶子的位置 */
    {  213,  585,1899, 2180, 93  },   /* TARGET_LOOK   识别靶子: 摄像头对准靶子 */
    {  213,  565,1889, 2175, 93  },   /* TARGET_FIRE   激光发射位 */
    {  222, 1843, 878, 1834, 93  },   /* TARGET_LIFT   发射完激光后大臂抬起(与 LOOK 同值) */
    { 1190, 1311,1404, 1302, 93  },   /* HOSTAGE_LOOK  识别人质: 摄像头对准人质 */
    { 1198,  484,1868, 1754, 93  },   /* HOSTAGE_PRE   机械臂准备抱人质 */
    { 1190,  484,1868, 1754, 600  },   /* HOSTAGE_CLOSE 抱紧人质 */
    { 1190, 1892,1040, 2037, 600 }    /* HOSTAGE_LIFT  抱起人质后大臂抬起 */
};

/* ⭐ 每个姿态的“运动时间”(ms): 从上一个姿态走到本姿态用多久。
 *   值大 = 慢而稳; 值小 = 快但容易抖/过冲。想单独调某个动作就改对应行。
 *   下标与 ArmPose_t 一一对应; 下面先给一套默认值, 实测后逐个改。 */
static uint16_t s_arm_pose_time[ARM_POSE_COUNT] = {
    1500,   /* HOME          复位 */
    6000,   /* SCAN          扫码 */
    2500,   /* SCAN_RESET    扫码之后复位 */
    2500,   /* BALL_LOOK     看球 */
    2500,   /* BALL_PRE      抓夹到小球前 */
    2000,   /* BALL_CLOSE    夹紧(只有夹爪动) */
    2500,   /* BALL_LIFT     抓完抬起 */
    2500,   /* BUCKET_CARRY  携带姿态 */
    2500,   /* BUCKET_LOOK   看桶 */
    2500,   /* PLACE_PRE     移到放置位 */
    2000,   /* PLACE_OPEN    松开(只有夹爪动) */
    2500,   /* PLACE_LIFT    放完抬起 */
    2500,   /* TARGET_READY  准备识别靶子 */
    4000,   /* TARGET_LOOK   识别靶子 */
    6000,   /* TARGET_FIRE   激光发射位 */
    4000,   /* TARGET_LIFT   发射完抬起 */
    2500,   /* HOSTAGE_LOOK  识别人质 */
    2500,   /* HOSTAGE_PRE   准备抱人质 */
    2000,   /* HOSTAGE_CLOSE 抱紧(只有夹爪动) */
    2500    /* HOSTAGE_LIFT  抱起抬起 */
};


/* 姿态名表(顺序与 ArmPose_t 一致, 日志/OLED 显示用) */
static const char *const s_arm_pose_names[ARM_POSE_COUNT] = {
    "HOME", "SCAN", "SCAN_RESET", "BALL_LOOK", "BALL_PRE", "BALL_CLOSE", "BALL_LIFT",
    "BUCKET_CARRY", "BUCKET_LOOK", "PLACE_PRE", "PLACE_OPEN", "PLACE_LIFT",
    "TARGET_READY", "TARGET_LOOK", "TARGET_FIRE", "TARGET_LIFT",
    "HOSTAGE_LOOK", "HOSTAGE_PRE", "HOSTAGE_CLOSE", "HOSTAGE_LIFT"
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
 * @brief  摆到指定姿态(阻塞: 等舵机走完该姿态的运动时间 + hold_time 再返回)
 * @param  pose_idx  ARM_POSE_* (见 MissionControl.h)
 * @note   本函数会阻塞主循环数秒; 但底盘闭环在 TIM9 中断里跑, 所以
 *         “先发底盘指令 → 再调本函数”可以让车和臂同时动作, 节省时间。
 */
void Arm_GotoPose(uint8_t pose_idx)
{
    uint16_t *pos = ArmPose_Ptr(pose_idx);

    if (pos == NULL) {
        MLOG("Arm: bad pose %d", (int)pose_idx);
        return;
    }
#if MISSION_TEST_NO_ARM
    MLOG("Arm: pose %s (%ums) skipped (MISSION_TEST_NO_ARM=1)",
         ArmAction_GetName(pose_idx), (unsigned)s_arm_pose_time[pose_idx]);
#else
    uint16_t t = s_arm_pose_time[pose_idx];

    MLOG("Arm: -> %s (%ums)", ArmAction_GetName(pose_idx), (unsigned)t);
    Servos_SetPositions(pos, t);
    /* ⭐ 必须用协作式等待(不能直接用 HAL_Delay): 等摆臂的这几秒里
     * 陀螺仪 yaw 要照刷、K230 收到的行要照收, 否则底盘航向/转向闭环
     * 会读到冻结角度(原地转向会“转不完”→ 车一直自转)。 */
    Mission_Coop_Wait(t + hold_time);
#endif
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
        MLOG("Arm: bad pose %d", (int)pose_idx);
        return;
    }
#if MISSION_TEST_NO_ARM
    (void)first_mask;
    MLOG("Arm: pose %s skipped (MISSION_TEST_NO_ARM=1)",
         ArmAction_GetName(pose_idx));
#else
    {
        uint16_t t  = s_arm_pose_time[pose_idx];
        uint16_t t2 = (uint16_t)(t / 2);
        uint8_t  second_mask;

        if (t2 < ARM_SPLIT_2ND_MIN_MS) t2 = ARM_SPLIT_2ND_MIN_MS;
        if (t2 > t)                    t2 = t;
        second_mask = (uint8_t)((~first_mask) & SERVO_MASK_ALL);

        MLOG("Arm: -> %s (2-step: 1st mask=0x%02X %ums, 2nd mask=0x%02X %ums)",
             ArmAction_GetName(pose_idx), (unsigned)first_mask, (unsigned)t,
             (unsigned)second_mask, (unsigned)t2);

        /* 第 1 步: 底座/大臂/副关节先转到位 */
        Servos_SetPositionsMasked(pos, first_mask, t);
        Mission_Coop_Wait(t + hold_time);

        /* 第 2 步: 前面已停稳, 再动腕部/夹爪 */
        Servos_SetPositionsMasked(pos, second_mask, t2);
        Mission_Coop_Wait(t2 + hold_time);
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
    MLOG("ArmTeach ENTER: move arm by hand; KEY2=next, KEY1=write");
}

void ArmTeach_Exit(void)
{
    s_arm_teach_active = 0;
    for (uint8_t i = SERVO_ARM_ID_MIN; i <= SERVO_ARM_ID_MAX; i++) {
        Servos_SetTorque(i, 1);
    }
    Servos_SetPositions(Servos_GetHomePositions(), 1500);
    MLOG("ArmTeach EXIT: torque on, back home");
}

void ArmTeach_NextAction(void)
{
    s_arm_teach_action = (uint8_t)((s_arm_teach_action + 1) % ARM_POSE_COUNT);
    MLOG("ArmTeach action[%d] = %s", (int)s_arm_teach_action,
         ArmAction_GetName(s_arm_teach_action));
}

void ArmTeach_WriteCurrent(void)
{
    uint16_t pos[SERVO_COUNT];

    Servos_ReadPositions();
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        int32_t p = Servos_GetPosition(i + 1);
        if (p < 0 || p > 4095) {
            MLOG("ArmTeach warn: servo %d read fail -> 0", (int)(i + 1));
            p = 0;
        }
        pos[i] = (uint16_t)p;
    }
    // ArmAction_SetPositions(s_arm_teach_action, pos);
    MLOG("ArmTeach WRITE %s = %d,%d,%d,%d,%d",
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
        MLOG("Vision: mode settle done, dropped %u old line(s)",
             (unsigned)s_vision_drop_cnt);
    }

    if (s_k230_q_count > 0) {
        memcpy(dst, s_k230_q[s_k230_q_head], maxlen - 1);
        dst[maxlen - 1] = '\0';
        s_k230_q_head = (uint8_t)((s_k230_q_head + 1) % K230_QUEUE_DEPTH);
        s_k230_q_count--;
        return 1;
    }

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

/**
 * @brief  排爆第一步: 抓小球(BALL_PRE → 夹紧 → 抬起)
 * @note   调用前机械臂应已在 ARM_POSE_BALL_LOOK(看球), 且底盘已视觉对准
 */
void Arm_Start_Bomb_Grab(void)
{
    MLOG("Arm: Bomb Grab");
    Arm_GotoPose(ARM_POSE_BALL_PRE);
    Arm_GotoPose(ARM_POSE_BALL_CLOSE);
    Arm_GotoPose(ARM_POSE_BALL_LIFT);
}

/**
 * @brief  排爆第二步: 放球(PLACE_PRE → 松开 → 抬起)
 * @note   调用前机械臂应已在 ARM_POSE_BUCKET_LOOK(看桶), 且底盘已对准球桶
 */
void Arm_Start_Bomb_Place(void)
{
    MLOG("Arm: Bomb Place");
    Arm_GotoPose(ARM_POSE_PLACE_PRE);
    Arm_GotoPose(ARM_POSE_PLACE_OPEN);
    /* ⭐ PLACE_LIFT 实测会剥蹭: 拆成两步 —— 先 ID1/ID2/ID3 转到位, 再动 ID4/ID5 */
    Arm_GotoPoseSplit(ARM_POSE_PLACE_LIFT, SERVO_MASK_ARM_BODY);
}

/** @brief 打靶: 摆到激光发射位(调用后由状态机开激光) */
void Arm_Start_Target_Fire(void)
{
    MLOG("Arm: Target Fire");
    Arm_GotoPose(ARM_POSE_TARGET_FIRE);
}

/** @brief 打靶: 发射完把大臂抬起 */
void Arm_Start_Target_Lift(void)
{
    MLOG("Arm: Target Lift");
    Arm_GotoPose(ARM_POSE_TARGET_LIFT);
}

/** @brief 救援: 准备抱人质 → 抱紧 */
void Arm_Start_Rescue_Grab(void)
{
    MLOG("Arm: Rescue Grab");
    Arm_GotoPose(ARM_POSE_HOSTAGE_PRE);
    Arm_GotoPose(ARM_POSE_HOSTAGE_CLOSE);
}

/** @brief 救援: 抱起人质后抬起 */
void Arm_Start_Rescue_Retract(void)
{
    MLOG("Arm: Rescue Retract");
    Arm_GotoPose(ARM_POSE_HOSTAGE_LIFT);
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
    Chassis_SetHeadingRef(0.0f);
    Chassis_Stop();
    Laser_Off();
    LED_OFF();
#if MISSION_TEST_NO_ARM
    MLOG("Arm disabled (test): skip home");
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
        MLOG("--- MISSION START ---");
        g_mission_state = STATE_1_MOVING_TO_QR_SCAN;
    }
}

/**
 * @brief  像素误差 -> 修正距离(【左右】轴)
 * @note   前后轴用后面的 Calculate_Move_Distance_FB() —— 它多一层缩放 +
 *         自己的一套 min/max, 因为前后离目标太近, 必须比左右保守。
 */
static int32_t Calculate_Move_Distance(int pixel_error)
{
    if (abs(pixel_error) < ALIGN_TOLERANCE) return 0;
    int32_t dist = (int32_t)(abs(pixel_error) * K_GAIN);
    if (dist < MIN_MOVE_MM) dist = MIN_MOVE_MM;
    if (dist > MAX_MOVE_MM) dist = MAX_MOVE_MM;
    return dist;
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
} AlignAxisCfg_t;

#if ALIGN_AXIS_SCHEME_1
/* ---- 方案一(当前): 排爆/打靶 只修横向; 救援 只修前后 ----
 * (球/桶的 allow_fb=0, 所以它们那三行前后步长参数不参与运算) */
static const AlignAxisCfg_t s_align_ball   = { 1, 0, 0, 0, FB_SCALE_PCT, FB_MIN_MM, FB_MAX_MM };
static const AlignAxisCfg_t s_align_bucket = { 1, 0, 0, 1, BUCKET_FB_SCALE_PCT, BUCKET_FB_MIN_MM, BUCKET_FB_MAX_MM };
#if MISSION_DEBUG_VISION_TASK
/* 靶: ⚠️ 打靶已改版为“只转底座 ID1”, 不再做 D:x,y 精对准。
 * 本项只给 MISSION_DEBUG_VISION_TASK=2 的单独调试用, 平时包在 #if 里,
 * 免得触发 -Wunused-const-variable 警告 */
static const AlignAxisCfg_t s_align_target = { 1, 0, 0, 0, FB_SCALE_PCT, FB_MIN_MM, FB_MAX_MM };
#endif
static const AlignAxisCfg_t s_align_rescue = { 0, 1, 1, 0, FB_SCALE_PCT, FB_MIN_MM, FB_MAX_MM };
#else
/* ---- 方案二: 前后左右都修(方向映射仍按上表); 桶的前后步长单独限小 ---- */
static const AlignAxisCfg_t s_align_ball   = { 1, 1, 0, 0,
                                               FB_SCALE_PCT, FB_MIN_MM, FB_MAX_MM };
static const AlignAxisCfg_t s_align_bucket = { 1, 1, 0, 1,
                                               BUCKET_FB_SCALE_PCT,
                                               BUCKET_FB_MIN_MM,
                                               BUCKET_FB_MAX_MM };
#if MISSION_DEBUG_VISION_TASK
static const AlignAxisCfg_t s_align_target = { 1, 1, 0, 0,
                                               FB_SCALE_PCT, FB_MIN_MM, FB_MAX_MM };  /* 仅调试用 */
#endif
static const AlignAxisCfg_t s_align_rescue = { 1, 1, 1, 0,
                                               FB_SCALE_PCT, FB_MIN_MM, FB_MAX_MM };
#endif

/**
 * @brief  像素误差 -> 修正距离(【前后】轴专用)
 * @param  pixel_error 前后方向的像素误差
 * @param  cfg         本阶段配置(取 fb_scale_pct / fb_min_mm / fb_max_mm)
 * @note   与 Calculate_Move_Distance() 的差别只有两点:
 *         ① 先乘 fb_scale_pct%(把前后步长整体缩小, 默认 50%);
 *         ② 再用 fb_min_mm / fb_max_mm 限幅(与左右那套 MIN/MAX_MOVE_MM 独立)。
 *         目的: 左右保持合适的手感, 只把“往前贴”的步长压小, 防空桶/撞桶。
 *         为什么必须用独立限幅: 若直接套 MIN_MOVE_MM, 缩小后的值会被
 *           下限又抬回去, 等于没改。
 */
static int32_t Calculate_Move_Distance_FB(int pixel_error, const AlignAxisCfg_t *cfg)
{
    if (abs(pixel_error) < ALIGN_TOLERANCE) return 0;
    int32_t dist = (int32_t)(abs(pixel_error) * K_GAIN);
    dist = (dist * (int32_t)cfg->fb_scale_pct) / 100;   /* ① 前后整体缩小 */
    if (dist < (int32_t)cfg->fb_min_mm) dist = (int32_t)cfg->fb_min_mm;
    if (dist > (int32_t)cfg->fb_max_mm) dist = (int32_t)cfg->fb_max_mm;
    return dist;
}

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
        MLOG("%s: Vision Start (run_task:%d)", tag, (int)task);
    }
    K230_Run_Specific_Task(task);
    g_vision_task_in_progress = 1;
    *p_last_send = HAL_GetTick();
}

/**
 * @brief  进入精对准: 发 start_align 并复位计时/冷却
 */
static void Vision_StartFineAlign(const char *tag, uint32_t *p_state_tick,
                                  uint32_t *p_align_tick, uint32_t *p_cooldown)
{
    MLOG("%s: Request Fine Align", tag);
    K230_Start_Align();
    K230_FlushAll();
    *p_state_tick = HAL_GetTick();
    *p_align_tick = HAL_GetTick();
    *p_cooldown = 0;
    s_align_shift_strafe = 0;   /* 新一次对准: 清累计偏移 */
    s_align_shift_fwd    = 0;
    Chassis_Stop();
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
 *         注意: 左右步长由 Calculate_Move_Distance() 算(K_GAIN + MIN/MAX_MOVE_MM);
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
    if (strncmp(line, "OK", 2) == 0) {
        MLOG("%s Vision OK -> Settle 0.5s", tag);
        MLOG("%s Align shift: L/R=%ldmm F/B=%ldmm", tag,
             (long)s_align_shift_strafe, (long)s_align_shift_fwd);
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

            MLOG("%s Err: %d, %d", tag, err_x, err_y);

            /* ① 画面误差 → 车体误差 */
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

            /* 是否已对准: 只看【允许修的那些轴】 */
            if ((!cfg->allow_lr || abs(lr_px) < ALIGN_TOLERANCE) &&
                (!cfg->allow_fb || abs(fb_px) < ALIGN_TOLERANCE)) {
                MLOG("%s Close Enough -> Settle 0.5s", tag);
                MLOG("%s Align shift: L/R=%ldmm F/B=%ldmm", tag,
                     (long)s_align_shift_strafe, (long)s_align_shift_fwd);
                Chassis_Stop();
                *p_settle = HAL_GetTick() + FINE_TUNE_SETTLE_MS;
                return 1;
            }

            /* ③ 串行修正: 先左右, 后前后 */
            if (cfg->allow_lr && abs(lr_px) >= ALIGN_TOLERANCE) {
                int32_t d = Calculate_Move_Distance(lr_px);
                if (d > 0) {
                    if (lr_px < 0) { Chassis_Move_Right(d); s_align_shift_strafe -= d; }
                    else           { Chassis_Move_Left(d);  s_align_shift_strafe += d; }
                    *p_cooldown = HAL_GetTick() + FINE_TUNE_COOLDOWN_MS;
                }
            } else if (cfg->allow_fb && abs(fb_px) >= ALIGN_TOLERANCE) {
                int32_t d = Calculate_Move_Distance_FB(fb_px, cfg);   /* 前后用独立的小步长 */
                if (d > 0) {
                    MLOG("%s F/B step %ldmm (err %d)", tag, (long)d, fb_px);
                    if (fb_px < 0) { Chassis_Move_Backward(d); s_align_shift_fwd -= d; }
                    else           { Chassis_Move_Forward(d);  s_align_shift_fwd += d; }
                    *p_cooldown = HAL_GetTick() + FINE_TUNE_COOLDOWN_MS;
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
        MLOG("%s Align Timeout!", tag);
        Chassis_Stop();
        return 1;
    }
    if (HAL_GetTick() - align_tick > FORCE_GRAB_AFTER_MS && g_k230_new_data_flag) {
        MLOG("%s 15s Limit!", tag);
        Chassis_Stop();
        return 1;
    }
    return 0;
}

#if !MISSION_TEST_NO_VISION
/* =====================================================================
 * ⭐ 原地只转底座 ID1 (打靶 / 救援 共用; 2026-10-04 新增, 10-05 泛化)
 * ---------------------------------------------------------------------
 * 为什么不用 Arm_GotoPose / 直接改姿态表:
 *   ① Arm_GotoPose 是“整张姿态表一起写”, 会把 ID2~ID5 也重写一遍;
 *   ② 它会阻塞(运动时间 + hold_time) ≈ 3~4.5s, 期间主循环收不到 C/L/R,
 *      不适合“转一点 → 看一眼画面 → 再转一点”的逐个修正。
 * 所以这里只用 Servos_SetPositionsMasked(pose, SERVO_MASK_ID1, ...) 下发 ID1:
 *   位置数组里其余 4 个舵机仍填【基准姿态】那一行(因为没选中, 不会动),
 *   只有 [0]=ID1 被改成新值。下发后立即返回, 由调用方用冷却时间等它停稳。
 * ⚠️ 调用前必须已经用 Arm_GotoPose(基准姿态) 把臂摆好, 否则缓存里记的 ID1
 *    起点跟实际对不上(下发瞬间会先跳一下再走)。
 * ===================================================================== */
static uint16_t s_id1_pos      = 0;   /* ID1 当前指令位置 */
static uint8_t  s_id1_pose_idx = 0;   /* 上面那个位置对应的【基准姿态】 */

/** @brief 把 ID1 位置缓存复位到指定基准姿态的底座值 */
static void Arm_Id1Reset(uint8_t pose_idx)
{
    s_id1_pose_idx = pose_idx;
    s_id1_pos      = s_arm_pose_table[pose_idx][0];
    MLOG("ID1: reset base pose %s -> %d",
         ArmAction_GetName(pose_idx), (int)s_id1_pos);
}

/**
 * @brief  让底座 ID1 相对当前位置转一步(不阻塞)
 * @param  pose_idx 基准姿态(打靶=ARM_POSE_TARGET_LOOK, 救援=ARM_POSE_HOSTAGE_LOOK)
 * @param  delta    角度码增量: 正 = 数值增大, 负 = 数值减小
 * @param  move_ms  本步转动时间(ms)
 * @param  pos_min  限幅下限(角度码), 防越界堵转
 * @param  pos_max  限幅上限(角度码)
 * @note   换了基准姿态时自动重新对齐缓存; 单步增量远小于 2048, 不会触发
 *         飞特舵机的“最短路径反向甩”问题。
 */
static void Arm_Id1Step(uint8_t pose_idx, int32_t delta, uint16_t move_ms,
                        uint16_t pos_min, uint16_t pos_max)
{
    uint16_t pose[SERVO_COUNT];
    int32_t  v;

    if (s_id1_pose_idx != pose_idx) {
        Arm_Id1Reset(pose_idx);          /* 换了阶段/基准姿态: 重新对齐 */
    }
    v = (int32_t)s_id1_pos + delta;
    if (v < (int32_t)pos_min) v = (int32_t)pos_min;
    if (v > (int32_t)pos_max) v = (int32_t)pos_max;
    s_id1_pos = (uint16_t)v;

    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        pose[i] = s_arm_pose_table[pose_idx][i];
    }
    pose[0] = s_id1_pos;

    MLOG("ID1[%s] %+ld -> %d", ArmAction_GetName(pose_idx), (long)delta, (int)s_id1_pos);
    Servos_SetPositionsMasked(pose, SERVO_MASK_ID1, move_ms);
}

/* ---- 打靶 / 救援 各自的封装: 把各自的参数宏收口在一处, 调用点更短 ---- */
/** @brief 打靶: ID1 以 TARGET_LOOK 为基准转一步 */
static void Target_Id1Step(int32_t delta)
{
    Arm_Id1Step(ARM_POSE_TARGET_LOOK, delta,
                TARGET_ID1_MOVE_MS, TARGET_ID1_POS_MIN, TARGET_ID1_POS_MAX);
}

/** @brief 救援: ID1 以 HOSTAGE_LOOK 为基准转一步 */
static void Rescue_Id1Step(int32_t delta)
{
    Arm_Id1Step(ARM_POSE_HOSTAGE_LOOK, delta,
                RESCUE_ID1_MOVE_MS, RESCUE_ID1_POS_MIN, RESCUE_ID1_POS_MAX);
}
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
        BOMB_SETTLE_PLACE,         /* 对准桶后停稳 */
        BOMB_PERFORM_PLACE,        /* 放球 */
        BOMB_COMPLETE
    } BombSubState_t;

    /* 任务2: 打靶(2026-10-04 改版) —— 底盘不动, 只转底座 ID1 对准;
     *       收到 C 之后依次摆 FIRE / LIFT / SCAN_RESET 收尾 */
    typedef enum {
        TARGET_IDLE,            /* 发 run_task:2(靶), 等 K230 回 C/L/R */
        TARGET_ID1_MOVING,      /* 刚转了一步 ID1, 等它走完(TARGET_ID1_MOVE_MS) */
        TARGET_PERFORM,         /* 收到 C: 摆 FIRE → LIFT → SCAN_RESET(三段都阻塞到位) */
        TARGET_COMPLETE
    } TargetSubState_t;

    /* 任务3: 救援(停下给视觉→对准→抓取; K230 任务号 = 4 = K230_TASK_RESCUE)
     * 两种对准方案由 RESCUE_SCHEME_ID1 切换(见文件头“救援任务参数”) */
    typedef enum {
        RESCUE_IDLE,            /* 发 run_task:4(形状), 等 K230 回 C/L/R */
        RESCUE_WAIT_ADJUST,     /* [方案一] 视觉给 L/R → 车前进/后退一小段, 等它走完 */
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

    /* ---- 救援专用(方案二): 底座 ID1 原地对准的状态量 ---- */
    static uint8_t  rescue_id1_inited = 0;      /* 0=本次救援还没开始 L/R 对准 */
    static uint8_t  rescue_heard      = 0;      /* 0=还没收到过 K230 任何回应 */
    static uint16_t rescue_id1_steps  = 0;      /* 本次已转了多少步(防卡死①) */
    static uint32_t rescue_id1_tick   = 0;      /* 本次 L/R 对准起始时刻(防卡死②) */
    static uint32_t rescue_id1_move_tick = 0;   /* 本步 ID1 开始转动时刻 */

    char line[K230_LINE_MAX];

    /* ================= 任务1: 排爆(抓小球 -> 对准桶放置) ================= */
    if (expected_task_number == 1) {
        switch (bomb_sub_state) {
            case BOMB_IDLE:
#if MISSION_TEST_NO_VISION
                /* 测试: 无 K230, 模拟路径 C(直行靠近) */
                bomb_path_taken = 'C';
                MLOG("Task1(Ball) Dir(sim): C");
                if (BOMB_C_APPROACH_MM) Chassis_Move_Forward((int32_t)BOMB_C_APPROACH_MM);
                state_start_tick = HAL_GetTick();
                bomb_sub_state = BOMB_WAIT_INITIAL_MOVE;
#else
                if (Mission_GetNewLine(line, sizeof(line))) {
                    g_vision_task_in_progress = 0;
                    bomb_path_taken = line[0];
                    MLOG("Task1(Ball) Dir: %c", bomb_path_taken);
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
                MLOG("Fine Align(sim): OK");
                Chassis_Stop();
                settle_until = HAL_GetTick() + FINE_TUNE_SETTLE_MS;
                bomb_sub_state = BOMB_SETTLE;
#else
                Vision_StartFineAlign("BALL", &state_start_tick, &align_start_time, &cooldown_until);
                bomb_sub_state = BOMB_WAIT_FINE_ALIGN;
#endif
                break;

            case BOMB_WAIT_FINE_ALIGN:
                if (Vision_FineAlignTimeout("BALL", state_start_tick, align_start_time)) {
                    bomb_sub_state = BOMB_PERFORM_GRAB;
                    break;
                }
                if (Mission_GetNewLine(line, sizeof(line))) {
                    if (Vision_FineAlignProcess(line, "BALL", &cooldown_until, &settle_until,
                                                &s_align_ball)) {
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
                MLOG("State: GRAB");
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
                MLOG("Bucket Dir(sim): C");
                state_start_tick = HAL_GetTick();
                bomb_sub_state = BOMB_WAIT_BUCKET_DIR_MOVE;
#else
                if (Mission_GetNewLine(line, sizeof(line))) {
                    g_vision_task_in_progress = 0;
                    bucket_path_taken = line[0];
                    MLOG("Bucket Dir: %c", bucket_path_taken);
                    /* ⭐ 桶阶段方向(见文件头“视觉方向映射”):
                     *   机械臂已转到 BUCKET_LOOK(底座≈2243, 比球姿态多转≈180°),
                     *   画面左右相对车体【镜像】, 所以:
                     *     L(画面左) → 小车【右】移
                     *     R(画面右) → 小车【左】移
                     *     C(已居中) → 【不往前走】(车头已抵桶边缘, 再往前会过冲) */
                    if (bucket_path_taken == 'C') {
                        /* 不动, 直接交给精对准 */
                    } else if (bucket_path_taken == 'L') {
                        if (BUCKET_L_ADJUST_MM) Chassis_Move_Right(BUCKET_L_ADJUST_MM);
                    } else if (bucket_path_taken == 'R') {
                        if (BUCKET_R_ADJUST_MM) Chassis_Move_Left(BUCKET_R_ADJUST_MM);
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
                MLOG("Bucket Fine(sim): OK");
                Chassis_Stop();
                settle_until = HAL_GetTick() + FINE_TUNE_SETTLE_MS;
                bomb_sub_state = BOMB_SETTLE_PLACE;
#else
                Vision_StartFineAlign("BUCKET", &state_start_tick, &align_start_time, &cooldown_until);
                bomb_sub_state = BOMB_WAIT_BUCKET_ALIGN;
#endif
                break;

            case BOMB_WAIT_BUCKET_ALIGN:
                if (Vision_FineAlignTimeout("BUCKET", state_start_tick, align_start_time)) {
                    bomb_sub_state = BOMB_PERFORM_PLACE;
                    break;
                }
                if (Mission_GetNewLine(line, sizeof(line))) {
                    if (Vision_FineAlignProcess(line, "BUCKET", &cooldown_until, &settle_until,
                                                &s_align_bucket)) {
                        bomb_sub_state = BOMB_SETTLE_PLACE;
                    }
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
                MLOG("State: PLACE");
                Arm_Start_Bomb_Place();
                /* ⭐ 放完球、大臂抬起之后, 直接把臂转到“准备识别靶子”姿态
                 *    (ARM_POSE_TARGET_READY)。之后 STATE_12_PART1_MOVE_A 让小车
                 *    右移; 右移到位后(STATE_12_PART1_CORRECT_A)再摆成 TARGET_LOOK。
                 *    这里是【先摆完停稳、再让车走】(Arm_GotoPose 内部已阻塞
                 *    运动时间+hold_time), 不是车臂并行。 */
                Arm_GotoPose(ARM_POSE_TARGET_READY);
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
    /* ================= 任务2: 打靶 (2026-10-04 改版: 底盘不动, 只转底座 ID1) =====
     * 执行到本函数时: 小车已走完 “右移850 → 航向校正 → 右移850 → 停车”,
     *                 臂已摆到 ARM_POSE_TARGET_LOOK(摄像头对准靶子)
     *
     *   ① TARGET_IDLE        发 run_task:2 → K230 回 C / L / R
     *                        (run_task 只在“还没收到过任何回应”前每 1s 重发,
     *                         收到回应后就不再打扰 K230)
     *                        L → 底座 ID1 向【左】转 TARGET_ID1_STEP 码
     *                        R → 底座 ID1 向【右】转 TARGET_ID1_STEP 码
     *                        C → 已对准, 进 TARGET_PERFORM
     *                        ⚠️ 全程【一条底盘指令都不发】, 小车原地不动
     *                        ⚠️ 这里【不把 g_vision_task_in_progress 清 0】:
     *                           因为要反复回到本状态, 清 0 会导致每次都走
     *                           “Vision Start”分支(重发 run_task + 清空队列)
     *   ② TARGET_ID1_MOVING  等 ID1 走完(TARGET_ID1_MOVE_MS)。期间把 K230
     *                        攒下的行【全部丢弃】, 保证下一帧看到的是“转完之后”
     *                        的画面 —— 否则会拿转之前的旧误差连续累加 → 冲过头
     *   ③ TARGET_PERFORM     收尾三段姿态, 每段都阻塞“该姿态运动时间 + hold_time”:
     *                          ARM_POSE_TARGET_FIRE  → 激光发射位(激光由 K230 控制)
     *                          ARM_POSE_TARGET_LIFT  → 打完把大臂抬起
     *                          ARM_POSE_SCAN_RESET   → 手臂收回, 准备跑路
     *   ④ TARGET_COMPLETE    回主状态机 → 右移 ROUTE_12_P2_A_MM → 航向校正 → 救援
     *
     * 🛡 防卡死: 转满 TARGET_ID1_STEP_MAX 步 或 超过 TARGET_ID1_TIMEOUT_MS
     *            仍未收到 C (含 K230 完全不回应) → 强制当作 C 处理, 保证能往下走
     */

    else if (expected_task_number == 2) {
        switch (target_sub_state) {
            case TARGET_IDLE:
#if MISSION_TEST_NO_VISION
                /* 测试: 无 K230, 模拟“已对准” → 直接去摆发射位 */
                MLOG("Task2 Dir(sim): C");
                target_sub_state = TARGET_PERFORM;
#else
                /* 本次打靶第一次进来: 复位计数/计时, 并把 ID1 缓存对齐到 TARGET_LOOK
                 * (此时 STATE_12_PART1_CORRECT_A 已经执行过 Arm_GotoPose(TARGET_LOOK)) */
                if (!target_id1_inited) {
                    target_id1_inited = 1;
                    target_heard      = 0;
                    target_id1_steps  = 0;
                    target_id1_tick   = HAL_GetTick();
                    Arm_Id1Reset(ARM_POSE_TARGET_LOOK);
                    MLOG("Target: ID1 align start (base=%d)", (int)s_id1_pos);
                }

                /* 🛡防卡死: 步数或时长任一超限 → 强制走 C 流程 */
                if (target_id1_steps >= TARGET_ID1_STEP_MAX ||
                    (HAL_GetTick() - target_id1_tick) > TARGET_ID1_TIMEOUT_MS) {
                    MLOG("Target: ID1 align FORCE C (steps=%u, %lums)",
                         (unsigned)target_id1_steps,
                         (unsigned long)(HAL_GetTick() - target_id1_tick));
                    target_sub_state = TARGET_PERFORM;
                    break;
                }

                if (Mission_GetNewLine(line, sizeof(line))) {
                    target_heard = 1;   /* 已与 K230 建立联系: 之后不再定时重发 */
                    target_path_taken = line[0];
                    MLOG("Task2 Dir: %c", target_path_taken);
                    /* 画面偏差 → 底座 ID1 小步偏转(车不动):
                     *   L → ID1 数值减小(向左) ; R → ID1 数值增大(向右)
                     * (方向由 TARGET_ID1_LR_SIGN 统管, 实测反了只改那个宏) */
                    if (target_path_taken == 'L') {
                        Target_Id1Step((int32_t)TARGET_ID1_LR_SIGN * TARGET_ID1_STEP);
                        target_id1_steps++;
                        target_id1_move_tick = HAL_GetTick();
                        target_sub_state = TARGET_ID1_MOVING;
                    } else if (target_path_taken == 'R') {
                        Target_Id1Step(-(int32_t)TARGET_ID1_LR_SIGN * TARGET_ID1_STEP);
                        target_id1_steps++;
                        target_id1_move_tick = HAL_GetTick();
                        target_sub_state = TARGET_ID1_MOVING;
                    } else {
                        /* 'C'(或其它字符): 认为已对准 */
                        MLOG("Target: C after %u step(s) -> Fire",
                             (unsigned)target_id1_steps);
                        target_sub_state = TARGET_PERFORM;
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

            case TARGET_PERFORM:
                /* 收尾三段姿态。Arm_GotoPose() 内部会阻塞并等待
                 * “该姿态的运动时间 + hold_time”, 也就是你说的
                 * “等待运动时间和保持时间过去”。 */
                MLOG("State: TARGET FIRE");
                Arm_Start_Target_Fire();               /* → ARM_POSE_TARGET_FIRE  */
                Arm_Start_Target_Lift();               /* → ARM_POSE_TARGET_LIFT  */
                Arm_GotoPose(ARM_POSE_SCAN_RESET);     /* → ARM_POSE_SCAN_RESET   */
                target_sub_state = TARGET_COMPLETE;
                break;

            case TARGET_COMPLETE:
                MLOG("Task 2 Done");
                /* 为下一次打靶(如果重跑)复位; 同时清 0, 让下一个任务的
                 * Vision_SendTask 把上面三段摆臂阻塞期间攒下的旧帧 flush 掉 */
                target_id1_inited = 0;
                target_heard      = 0;
                target_sub_state  = TARGET_IDLE;
                g_vision_task_in_progress = 0;
                /* ⭐ 打靶结束 → 右移 400mm(STATE_12_PART2_MOVE_A) → 航向校正 → 救援 */
                g_mission_state = STATE_12_PART2_MOVE_A;
                break;

            default: break;
        }
    }
    /* ================= 任务3: 救援 (视觉对准 + 抓取; K230 任务号=4) =================
     * 执行到本函数时: 小车已走完 “后退800 → 航向校准 → 停等3s”,
     *                 臂已摆到 ARM_POSE_HOSTAGE_LOOK(摄像头对准人质)
     *
     * ⭐ 对准方式两套, 由 RESCUE_SCHEME_ID1 切换(见文件头“救援任务参数”):
     *   【方案二】(=1, 当前) 与打靶同构 —— 【底盘完全不动】, 收到 L/R 就小步转
     *       底座 ID1; 收到 C 就认为对准 → 抓取。
     *   【方案一】(=0) 与排爆球/桶同构 —— L/R 先让车前进/后退一小段,
     *       再发 start_align 用 D:x,y 精对准(只修前后), 对准后抓取。
     * 两套共用的收尾:
     *   ⑥ RESCUE_SETTLE   停车稳定(FINE_TUNE_SETTLE_MS)
     *   ⑦ RESCUE_PERFORM  抓取(HOSTAGE_PRE → CLOSE) + 抱起抬起(HOSTAGE_LIFT)
     *   ⑧ RESCUE_COMPLETE 回主状态机 → 后退 ROUTE_17_RIGHT_B_MM …
     *
     * 🛡 方案二防卡死: 转满 RESCUE_ID1_STEP_MAX 步 或 超过 RESCUE_ID1_TIMEOUT_MS
     *    仍未收到 C(含 K230 完全不回应) → 强制当作 C 处理, 保证能往下走
     */
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
                        MLOG("Rescue: ID1 align start (base=%d)", (int)s_id1_pos);
                    }
                    /* 🛡防卡死: 步数 或 时长 超限 → 强制当作已对准, 直接去抓 */
                    if (rescue_id1_steps >= RESCUE_ID1_STEP_MAX ||
                        (HAL_GetTick() - rescue_id1_tick) > RESCUE_ID1_TIMEOUT_MS) {
                        MLOG("Rescue: ID1 align FORCE C (steps=%u, %lums)",
                             (unsigned)rescue_id1_steps,
                             (unsigned long)(HAL_GetTick() - rescue_id1_tick));
                        Chassis_Stop();
                        settle_until = HAL_GetTick() + FINE_TUNE_SETTLE_MS;
                        rescue_sub_state = RESCUE_SETTLE;
                        break;
                    }
                    if (Mission_GetNewLine(line, sizeof(line))) {
                        rescue_heard = 1;   /* 已联系上 K230: 之后不再定时重发 */
                        rescue_path_taken = line[0];
                        MLOG("Task3(Shape) Dir: %c", rescue_path_taken);
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
                            /* 'C'(或其它字符): 认为已对准 → 停稳后去抓
                             * (不直接上 RESCUE_PERFORM, 而是绕一下 RESCUE_SETTLE,
                             *  等 ID1 完全停稳再夹, 免得还在动就把人质抱歪) */
                            MLOG("Rescue: C after %u step(s) -> Grab",
                                 (unsigned)rescue_id1_steps);
                            Chassis_Stop();
                            settle_until = HAL_GetTick() + FINE_TUNE_SETTLE_MS;
                            rescue_sub_state = RESCUE_SETTLE;
                        }
                    } else if (!rescue_heard &&
                               (!g_vision_task_in_progress ||
                                HAL_GetTick() - vision_cmd_tick >= VISION_CMD_RESEND_MS)) {
                        Vision_SendTask(K230_TASK_RESCUE, "SHAPE", &vision_cmd_tick);
                    }
                } else {
                    /* ============ 方案一: 动底盘(前进/后退) + D:x,y 精对准 ============ */
                    if (Mission_GetNewLine(line, sizeof(line))) {
                        g_vision_task_in_progress = 0;
                        rescue_path_taken = line[0];
                        MLOG("Task3(Shape) Dir: %c", rescue_path_taken);
                        /* ⭐ 救援方向: 画面左右 = 车体前后(见上面“视觉方向映射”)
                         *   L → 前进;  R → 后退;  C → 已在中心, 不动 */
                        if (rescue_path_taken == 'C') {
                            /* 已在中心: 不移动, 直接进精对准(此时只修前后) */
                            Vision_StartFineAlign("SHAPE", &state_start_tick, &align_start_time, &cooldown_until);
                            rescue_sub_state = RESCUE_WAIT_ALIGN;
                        } else if (rescue_path_taken == 'L') {
                            if (RESCUE_ADJUST_MM) Chassis_Move_Forward(RESCUE_ADJUST_MM);
                            rescue_sub_state = RESCUE_WAIT_ADJUST;
                        } else if (rescue_path_taken == 'R') {
                            if (RESCUE_ADJUST_MM) Chassis_Move_Backward(RESCUE_ADJUST_MM);
                            rescue_sub_state = RESCUE_WAIT_ADJUST;
                        }
                    } else if (!g_vision_task_in_progress ||
                               HAL_GetTick() - vision_cmd_tick >= VISION_CMD_RESEND_MS) {
                        if (!g_vision_task_in_progress) {
                            rescue_idle_tick = HAL_GetTick();
                        } else if (HAL_GetTick() - rescue_idle_tick > VISION_RESPONSE_TIMEOUT_MS) {
                            /* K230 形状跟踪尚未实现/未回应: 超时跳过视觉, 盲走营救(防卡死) */
                            MLOG("Rescue: vision no response, blind rescue");
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
                    Vision_StartFineAlign("SHAPE", &state_start_tick, &align_start_time, &cooldown_until);
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
                MLOG("State: RESCUE PERFORM");
                Arm_Start_Rescue_Grab();      /* HOSTAGE_PRE(准备抱) → HOSTAGE_CLOSE(抱紧) */
                Arm_Start_Rescue_Retract();   /* HOSTAGE_LIFT(抱起后抬起) */
                rescue_sub_state = RESCUE_COMPLETE;
                break;

            case RESCUE_COMPLETE:
                MLOG("Task 3 (Rescue) Done");
                /* 复位, 并把 g_vision_task_in_progress 清 0,
                 * 让后面的视觉任务重新走 “Vision Start + 换任务静默期” */
                rescue_id1_inited = 0;
                rescue_heard      = 0;
                rescue_sub_state  = RESCUE_IDLE;
                g_vision_task_in_progress = 0;
                /* ⭐ 抓取完成 → ⑥ 抓完后第 1 段后退(STATE_17_RESCUE_RIGHT_B) */
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

/* 链路监控发送: which 0=reset:0, 1=scan_qr, 2=run_task:1 */
void Mission_DebugVisionLinkSend(uint8_t which)
{
    if (which == 0) {
        K230_Reset();
        MLOG("TX: reset:0");
    } else if (which == 1) {
        K230_Request_QRScan();
        MLOG("TX: scan_qr");
    } else {
        K230_Run_Specific_Task(K230_TASK_BALL);
        MLOG("TX: run_task:%d", (int)K230_TASK_BALL);
    }
}

void Mission_DebugVisionStart(void)
{
    s_dbg_sub = 0;
    s_dbg_done = 0;
    g_vision_task_in_progress = 0;
    Chassis_Stop();
    MLOG("Debug vision: task %d start", (int)MISSION_DEBUG_VISION_TASK);
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
            MLOG("Vision link monitor: KEY1=scan_qr  KEY1long=run_task  KEY2=reset:0");
        }
        while (Mission_GetNewLine(line, sizeof(line))) {
            s_dbg_rx_count++;
            MLOG("RX[%lu]: %s", (unsigned long)s_dbg_rx_count, line);
        }
        return;
    }

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
            MLOG("%s Dir(sim): C", tag);
            if (c_mm) Chassis_Move_Forward(c_mm);
            s_dbg_move_tick = HAL_GetTick();
            s_dbg_sub = 1;
#else
            if (Mission_GetNewLine(line, sizeof(line))) {
                g_vision_task_in_progress = 0;
                char path = line[0];
                MLOG("%s Dir: %c", tag, path);
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
                MLOG("%s Fine(sim): OK", tag);
                Chassis_Stop();
                s_dbg_settle = HAL_GetTick() + FINE_TUNE_SETTLE_MS;
                s_dbg_sub = 3;
#else
                Vision_StartFineAlign(tag, &s_dbg_state_tick, &s_dbg_align_tick, &s_dbg_cooldown);
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
            MLOG("Debug vision: task %d DONE", (int)MISSION_DEBUG_VISION_TASK);
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
        MLOG("Debug arm: busy, ignore");
        return;
    }
    Chassis_Stop();          /* 确认底盘停住; 之后全程不发底盘指令 */
    s_armdbg_step = 0;
    s_armdbg_idle = 0;
    MLOG("Debug arm seq %d: START (chassis locked)", (int)MISSION_DEBUG_ARM_SEQ);
}

//该函数在机械臂的调试模式下，按下按键之后，在空闲阶段s_armdbg_step会增加，动作下一个
void Mission_ChangeStep(void)
{
    /* ⚠️ 原来这里是 if (!s_armdbg_idle) { "busy, ignore"; return; }
     *    —— 条件写反了: 按键时序列恰好正在运行(idle==0), 于是每次都被挡掉,
     *    永远切不到下一个动作。现在改成“已启动才切步”。 */
    s_armdbg_idle = 0;
     if (s_armdbg_idle) {
        MLOG("Debug arm: not started (press KEY1 to start)");
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
            MLOG("Debug arm: HOME初始位置状态");
            Servos_SetPositions(Servos_GetHomePositions(), 1500);
            HAL_Delay(2000);
            // s_armdbg_step = 1;
            break;

        case 1:     /* 扫码 */
            // Arm_GotoPose(ARM_POSE_HOSTAGE_LOOK);//机械臂转向至抓取人质位
            // MLOG("Debug arm: HOSTAGE_LOOK抓取人质状态");
            // Arm_GotoPose(ARM_POSE_BALL_LOOK);
            //  MLOG("Debug arm: Look看球");
            MLOG("Debug arm: PAUSE停顿状态");
            HAL_Delay(500);
            // MLOG("Debug arm: SCAN扫码位置状态");
            // Arm_GotoPose(ARM_POSE_SCAN);        //扫码姿态
            // HAL_Delay(2000);
            // s_armdbg_step = 2;
            break;

        case 2:   /* 抓取序列(张开→下放→闭合→抬臂) */
            // Arm_Start_Rescue_Grab();//抓取人质
            MLOG("Debug arm: GRAB抓取状态");
            Arm_Start_Bomb_Grab();          //排爆抓取小球
            
            // s_armdbg_step = 3;
            break;

        case 3:   /* 停顿(便于观察/换物) */
            MLOG("Debug arm: PAUSE停顿状态");
            HAL_Delay(500);
            // s_armdbg_step = 4;
            break;

        case 4:   /* 放置序列(转向→下放→松开→抬臂) */
            // Arm_Start_Rescue_Retract();//放置人质
            MLOG("Debug arm: PLACE放置状态");
            Arm_Start_Bomb_Place();         //排爆放置小球到球桶
            // s_armdbg_step = 5;
            break;

        case 5:   /* 回SCAN_RESET, 结束 */
            Arm_GotoPose(ARM_POSE_SCAN_RESET);      //扫码后复位
            HAL_Delay(2000);
            MLOG("Debug arm: DONE");
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
    static uint32_t s_qr_cmd_tick = 0;   /* scan_qr 发送时刻(定时重发用) */
    /* 阶段四③: 停下等待的起始时刻(等 RESCUE_STOP_WAIT_MS 再摆臂) */
    static uint32_t s_rescue_wait_tick = 0;
    /* 阶段三: 打靶走位第 2 段右移后“停车停稳”的起始时刻(TARGET_STOP_SETTLE_MS) */
    static uint32_t s_target_stop_tick = 0;

    /* ============================================================
     * 第一段: 状态进入动作 (仅在状态切换瞬间执行一次)
     * ============================================================ */
    if (g_mission_state != last_state) {
        MLOG("State: %d -> %d", (int)last_state, (int)g_mission_state);
        last_state = g_mission_state;   /* 记录新状态, 防止同一状态重复触发 */

        switch (g_mission_state) {
            /* ---------- 阶段一: 扫码并前往排爆区 ---------- */
            case STATE_1_MOVING_TO_QR_SCAN:   Chassis_Move_Forward(ROUTE_1_TO_QR_MM); break;  /* 起点→扫码点(直行) */
            /* 扫码: 车停稳后机械臂先摆到“扫码姿态”(207,425,1988,2135,2542)再让 K230 扫 */
            case STATE_2_PERFORMING_QR_SCAN:  Chassis_Stop(); Arm_GotoPose(ARM_POSE_SCAN); break;

            case STATE_3_MOVE_LEFT_A:         Chassis_Move_Left(ROUTE_3_LEFT_A_MM); break;    /* 扫码后左移 A 段 */
            case STATE_3A_HEADING_CORRECT:    Turn_Angle_Compat(0.1f); break;                 /* 左移后航向校正到 0° */
            case STATE_4_MOVE_TOPLEFT:        Chassis_Move_Diagonal(ROUTE_4_DIAG_FWD_MM, ROUTE_4_DIAG_LEFT_MM); break;  /* 左上斜跑(前/左分量见宏) */
            case STATE_4A_HEADING_CORRECT:    Turn_Angle_Compat(0.1f); break;                 /* 斜跑后航向校正到 0° */
            case STATE_5_STOP_BEFORE_RAMP:    Chassis_Move_Forward(ROUTE_5_TO_RAMP_MM); break;/* 斜坡前直行 */
            case STATE_6_CROSSING_RAMP_A:     Turn_Angle_Compat(0.1f); break;                 /* 过斜坡段: 航向校正 */

            /* ---------- STATE_7 分段: 左移B -> 中间停 -> 左移C -> 航向校正 ---------- */
            case STATE_7_MOVE_LEFT_B:         Chassis_Move_Left(ROUTE_7_LEFT_B_MM); break;    /* 左移阶段B */
            case STATE_7A_INTERMEDIATE_STOP:  Turn_Angle_Compat(0.1f); break;                 /* 中间停顿 + 航向校正 */
            case STATE_7B_MOVE_LEFT_C:        break;//Chassis_Move_Diagonal(ROUTE_7_LEFT_C_MM,ROUTE_7_LEFT_C_MM);Chassis_Move_Left(ROUTE_7_LEFT_C_MM); break;    /* 左移阶段C */
            case STATE_7A_HEADING_CORRECTION: break;//Turn_Angle_Compat(0.1f);                  /* 航向校正 */

            /* ---------- 排爆区走位 ---------- */
            case STATE_8_MOVE_FORWARD_A:      Chassis_Move_Forward(ROUTE_8_TO_BOMB_AREA_MM); break;  /* 直线前进进入排爆区 */
            case STATE_8A_HEADING_CORRECT:    Turn_Angle_Compat(0.1f); break;                 /* 前进后航向校正 */
            case STATE_8B_ADJUST_RIGHT:       Chassis_Move_Right(ROUTE_8B_RIGHT_MM); break;    /* 右移微调 */
            /* 右移到位后(8C)先把机械臂摆到“看球姿态”, 让 STATE_11 的视觉对准一开始
             * 就能看到小球; 先发转向指令再摆臂 → 车转+臂动 同时进行, 省时间 */
            case STATE_8C_HEADING_CORRECT:    Turn_Angle_Compat(0.1f); Arm_GotoPose(ARM_POSE_BALL_LOOK); break;
            case STATE_9_TURN_FOR_BOMB:       break;//Chassis_Rotate(ROUTE_9_TURN_DEG); break;        /* 转向排爆点(角度见宏) */
            case STATE_9A_ADJUST_LEFT:        break;//Chassis_Move_Left(ROUTE_9A_LEFT_MM); break;     /* 左移微调 */
            case STATE_10_APPROACH_BOMB:      break; //Chassis_Move_Left(ROUTE_10_APPROACH_MM); break; /* 接近炸弹 */
            case STATE_11_PERFORMING_BOMB_DISPOSAL: Chassis_Stop(); break;                    /* 停下, 交给视觉子状态机 */

            /* ---------- 阶段二: 打靶 ----------
             * 走位链(2026-10-05 定稿):
             *   MOVE_A(右移 ROUTE_12_P1_A_MM) →  CORRECT_A(航向校正到 0°)
             * → MOVE_B(再右移 ROUTE_12_P1_B_MM) →  CORRECT_B(停车)
             * → MOVE_C(摆 ARM_POSE_TARGET_LOOK) →  STATE_13 打靶视觉对准
             * 打完靶收尾: PART2_MOVE_A(右移) → PART2_CORRECT_A(航向校正) → 救援
             *
             * ⚠️ 写法上的关键点(看下面代码时心里要数这一条):
             *   【进入动作】只在“刚切到这个状态”的那一瞬间执行一次,
             *   之后每个主循环只跑【转移检查】。
             *   ⇒ “航向校正/右移”这种需要时间的动作, 就是靠
             *     “进入时发一次指令 + 转移条件里等 Chassis_Task_Is_Complete()”
             *     配合完成的(绝不能在转移里再发一次, 否则会反复重发)。 */
            case STATE_12_PART1_MOVE_A:       Chassis_Move_Right(ROUTE_12_P1_A_MM); break;  /* ⭐ 第1段右移 */
            /* ⭐ 右移到位后先做航向校正(转到给对 0°), 防止两次右移的累积误差。
             *    之后 MOVE_B 继续右移, CORRECT_B 停车, MOVE_C 才摆臂。 */
            case STATE_12_PART1_CORRECT_A:    Turn_Angle_Compat(0.1f);break;
            case STATE_12_PART1_MOVE_B:       Chassis_Move_Right(ROUTE_12_P1_B_MM); break;  
            case STATE_12_PART1_CORRECT_B:    Chassis_Stop(); s_target_stop_tick = HAL_GetTick(); break;  
            case STATE_12_PART1_MOVE_C:       Arm_GotoPose(ARM_POSE_TARGET_LOOK);break;   

            /* ⚠️ TURN_A/TURN_B 已不在流程里(不可达): 上面 MOVE_C 摆完臂直接跳 STATE_13。
             *    这里置空, 免得万一被进入时车突然跑 400mm */
            case STATE_12_TURN_A:             break;//Chassis_Move_Right(ROUTE_12_TURN_A_DEG);
            case STATE_12_TURN_B:             break;//Turn_Angle_Compat(0.1f);

            /* ⭐ 打靶收尾: 手臂收回 SCAN_RESET 之后右移 ROUTE_12_P2_A_MM,
             *    再航向校正一次 → 进救援。
             *    (这两个状态是复用的, 后面的 MOVE_B/CORRECT_B/MOVE_C 未使用) */
            case STATE_12_PART2_MOVE_A:       Chassis_Move_Right(ROUTE_12_P2_A_MM); break;  /* ⭐ 打靶后右移 ROUTE_12_P2_A_MM */
            case STATE_12_PART2_CORRECT_A:    Turn_Angle_Compat(0.1f); break;               /* ⭐ 航向校正 → 救援 */
            case STATE_12_PART2_MOVE_B:       break;   /* 已废弃 */
            case STATE_12_PART2_CORRECT_B:    break;//Turn_Angle_Compat(0.1f); break;                 /* 航向校正 */
            case STATE_12_PART2_MOVE_C:       break; //Chassis_Move_Right(ROUTE_12_P2_C_MM); break;  /* Part2 第3段 */

            /* 进入打靶视觉子状态机(底盘完全不动, 只转底座 ID1 对准 → 摆 FIRE/LIFT/SCAN_RESET)。
             * ⚠️ 这里不再做航向校正 —— CORRECT_A 已经校过一次, 而且 MOVE_C 摆臂时
             *    底盘是静止的, 不需要再校。 */
            case STATE_13_PERFORMING_TARGETING: break;

            /* ---------- 阶段四: 救援 (2026-10-05 重新定义) ----------
             * 完整流程:
             *   ①后退 ROUTE_14_TO_HOSTAGE_MM → ②航向校准 → ③停下等 RESCUE_STOP_WAIT_MS
             * → ④摆 ARM_POSE_HOSTAGE_LOOK → ⑤视觉对准 + 抓取(子状态机)
             * → ⑥后退 ROUTE_17_RIGHT_B_MM → ⑦航向校准
             * → ⑧后退 ROUTE_18_RIGHT_C_MM → ⑨航向校准 → ⑩停下(任务完成)
             * ⚠️ ⑤ 的对准方式由 RESCUE_SCHEME_ID1 切换(1=转 ID1 车不动 / 0=动底盘)。
             * ------------------------------------------------------------------ */
            case STATE_14_MOVE_FORWARD_B:          Chassis_Move_Backward(ROUTE_14_TO_HOSTAGE_MM); break; /* ① 后退 */
            case STATE_15_TURN_FOR_HOSTAGE:        Turn_Angle_Compat(0.1f); break;                     /* ② 航向校准 */
            /* ③ 停下等 RESCUE_STOP_WAIT_MS: 等车体晃动停稳, 再让机械臂摆出去 */
            case STATE_15A_RESCUE_STOP_WAIT:       Chassis_Stop(); s_rescue_wait_tick = HAL_GetTick(); break;
            /* ④ 摆成“识别人质”姿态(臂阻塞约 3s, 摆完直接去视觉) */
            case STATE_16_RESCUE_RIGHT_A:          Arm_GotoPose(ARM_POSE_HOSTAGE_LOOK); break;
            /* ⑤ 底盘完全不动, 交给救援视觉子状态机(对准 + 抓取) */
            case STATE_20_PERFORMING_HOSTAGE_RESCUE: break;
            /* ⑥⑦⑧⑨ 抓完后的撒退: 后退600 → 航向 → 后退600 → 航向 */
            case STATE_17_RESCUE_RIGHT_B:          Chassis_Move_Backward(ROUTE_17_RIGHT_B_MM); break;
            case STATE_17A_RESCUE_HEADING_CORRECT: Turn_Angle_Compat(0.1f); break;
            case STATE_18_RESCUE_RIGHT_C:          Chassis_Move_Backward(ROUTE_18_RIGHT_C_MM); break;
            case STATE_18A_RESCUE_HEADING_CORRECT: Turn_Angle_Compat(0.1f); break;
            /* ⑩ 停下 → 结束(转移里置 MISSION_STATE_COMPLETE) */
            case STATE_19_RESCUE_RIGHT_D:          break;

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
                Arm_GotoPose(ARM_POSE_SCAN_RESET);   /* 扫码之后: 机械臂收回 */
                g_mission_state++;
            }
#else
            {
                char line[K230_LINE_MAX];
                if (Mission_GetNewLine(line, sizeof(line))) {
                    g_vision_task_in_progress = 0;
                    const char *p = line;
                    if (strncmp(p, "qr:", 3) == 0) p += 3;   /* 兼容 "qr:xxx" 前缀 */
                    strncpy(g_qr_code_string, p, sizeof(g_qr_code_string) - 1);
                    g_qr_code_string[sizeof(g_qr_code_string) - 1] = '\0';
                    MLOG("QR: %s", g_qr_code_string);
                    K230_Send_QRCode_Data(g_qr_code_string);   /* 回传二维码给 K230 */
                    Arm_GotoPose(ARM_POSE_SCAN_RESET);          /* 扫码之后: 机械臂收回 */
                    g_mission_state++;
                } else if (!g_vision_task_in_progress ||
                           HAL_GetTick() - s_qr_cmd_tick >= VISION_CMD_RESEND_MS) {
                    /* 首次发送清空旧数据; 之后定时重发(不 flush, 避免丢掉刚到的 qr:) */
                    if (!g_vision_task_in_progress) {
                        K230_FlushAll();
                    }
                    K230_Request_QRScan();
                    g_vision_task_in_progress = 1;
                    s_qr_cmd_tick = HAL_GetTick();
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

        /* ---------- 打靶走位: 右移850 → 航向校正 → 右移850 → 停 → 摆TARGET_LOOK ---------- */
        case STATE_12_PART1_MOVE_A:       if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        /* CORRECT_A 发起的是“转到绝对 0°”, 等它转完再走第二段 */
        case STATE_12_PART1_CORRECT_A:    if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        case STATE_12_PART1_MOVE_B:       if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        /* CORRECT_B: 停车 Chassis_Stop() 之后底盘【立刻】就算“完成”, 所以原来这里是
         * 被“穿过”的(没起到停稳作用)。现在改成原地停稳 TARGET_STOP_SETTLE_MS
         * 再进 MOVE_C 摆臂, 免得底盘还在晃就把臂/摄像头伸出去。
         * (TARGET_STOP_SETTLE_MS 填 0 就回到原来的“直接穿过”行为) */
        case STATE_12_PART1_CORRECT_B:
            if ((HAL_GetTick() - s_target_stop_tick) >= TARGET_STOP_SETTLE_MS) {
                g_mission_state++;
            }
            break;
        /* ⭐ 摆完 TARGET_LOOK(阻塞约 4.5s)后才进打靶视觉子状态机 */
        case STATE_12_PART1_MOVE_C:       if (Chassis_Task_Is_Complete()) g_mission_state = STATE_13_PERFORMING_TARGETING; break;
        case STATE_12_TURN_A:             break;   /* 未使用 */
        case STATE_12_TURN_B:             break;   /* 未使用 */
        case STATE_12_PART2_MOVE_A:       if (Chassis_Task_Is_Complete()) g_mission_state++; break;
        /* ⭐ 打靶收尾: 右移400到位 + 航向校正完成后, 直接跳进救援阶段 */
        case STATE_12_PART2_CORRECT_A:    if (Chassis_Task_Is_Complete()) g_mission_state = STATE_14_MOVE_FORWARD_B; break;
        case STATE_12_PART2_MOVE_B:       break;   /* 已废弃 */
        case STATE_12_PART2_CORRECT_B:    break;   /* 已废弃 */
        /* 打靶: 进入视觉辅助子状态机(内部处理完会自己跳到 STATE_12_PART2_MOVE_A) */
        case STATE_12_PART2_MOVE_C:       break;   /* 已废弃 */
        case STATE_13_PERFORMING_TARGETING: Handle_Vision_Alignment(2); break;

        case STATE_14_MOVE_FORWARD_B:     if (Chassis_Task_Is_Complete()) g_mission_state++; break;   /* ① → ② */
        /* ② 航向校准完成 → ③ 停下等 3s */
        case STATE_15_TURN_FOR_HOSTAGE:
            if (Chassis_Task_Is_Complete()) {
                g_mission_state++;
            }
            break;

        /* ③ 原地停等 RESCUE_STOP_WAIT_MS, 时间到 → ④ 摆 HOSTAGE_LOOK */
        case STATE_15A_RESCUE_STOP_WAIT:
            if ((HAL_GetTick() - s_rescue_wait_tick) >= RESCUE_STOP_WAIT_MS) {
                g_mission_state++;
            }
            break;

        /* ---------- 阶段四(救援): 对准 → 抓取 → 后退 x2 + 航向校准 x2 ---------- */
        /* ④ 摆臂是阻塞的, 返回时已经摆好 → 直接跳到 ⑤ 视觉对准 */
        case STATE_16_RESCUE_RIGHT_A:
            if (Chassis_Task_Is_Complete()) g_mission_state = STATE_20_PERFORMING_HOSTAGE_RESCUE;
            break;
        /* ⑤ 视觉对准 + 抓取; 完成后子状态机自己跳到 STATE_17_RESCUE_RIGHT_B */
        case STATE_20_PERFORMING_HOSTAGE_RESCUE: Handle_Vision_Alignment(3); break;
        case STATE_17_RESCUE_RIGHT_B:          if (Chassis_Task_Is_Complete()) g_mission_state++; break;   /* ⑥ → ⑦ */
        case STATE_17A_RESCUE_HEADING_CORRECT: if (Chassis_Task_Is_Complete()) g_mission_state++; break;   /* ⑦ → ⑧ */
        case STATE_18_RESCUE_RIGHT_C:          if (Chassis_Task_Is_Complete()) g_mission_state++; break;   /* ⑧ → ⑨ */
        case STATE_18A_RESCUE_HEADING_CORRECT: if (Chassis_Task_Is_Complete()) g_mission_state++; break;   /* ⑨ → ⑩ */
        /* ⑩ 最后一步: 停下, 整个任务完成 */
        case STATE_19_RESCUE_RIGHT_D:
            MLOG("All Done.");
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
