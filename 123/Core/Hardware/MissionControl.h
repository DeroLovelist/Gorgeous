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
 *   非 0 时: 上电按一下 KEY1 直接进入对应视觉目标对准(不跑路线、不动机械臂),
 *   对准完成后停车并打印结果。改完记得重载+构建。
 *   ⚠️ 1~4 都是“底盘横移式对准”(收到 L/R 会横移 50~60mm), 收到 D:x,y 后
 *      用 K_GAIN 算横移量 —— 所以【它们标不了打靶的 ID1/ID4 舵机】。
 *   6 = ⭐ 打靶两轴标定模式【底盘一步不动】:
 *     上电按一下 KEY1 → 机械臂自动摆 TARGET_LOOK → 发 run_task:2 →
 *     收到 C/L/R 后发 start_align → 之后每来一帧 D:x,y 就:
 *       先修横向(ID1) / 后修竖直(ID4), 一次只转一个舵机。
 *     用途: 标定 TARGET_ID1_LR_SIGN / TARGET_ID1_STEP /
 *           TARGET_ID4_DY_SIGN / TARGET_ID4_STEP。
 *     日志前缀是 [TCAL], 详细判定方法见 MissionControl.c 里 mode 6 的注释块。
 *     用法: 把靶子【故意放偏】(放正中间 K230 直接回 OK, 没东西可标)。
 *   5 = 视觉串口链路监控(纯收发打印, 车/臂完全不动):
 *     上电后 KEY1=发 run_task:1(球), KEY1 长按=发 run_task:2(靶), KEY2=发 reset:0,
 *     收到 K230 任何一行打印 "RX: ..."
 *     ⚠️ 新 K230(main.py + yolo_main.py)不支持 "scan_qr", 所以链路监控不再发它 */
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

    /* STATE_7 分段
     * ⭐⭐ 2026-10-10(用户要求): B 段左移【默认一整段走完】, 由
     *    ROUTE_7_LEFT_B_SPLIT_ENABLE 切换(当前 = 0):
     *      0(当前): STATE_7_MOVE_LEFT_B 走整段 ROUTE_7_LEFT_B_MM, 走完直接进 STATE_7A
     *               (下面两个状态【不会被进入】)
     *      1      : 前半 ROUTE_7_LEFT_B_HALF_MM → 车头前进 ROUTE_7_LEFT_B_FWD_MM
     *               → 后半 ROUTE_7_LEFT_B_HALF2_MM (老走法) */
    STATE_7_MOVE_LEFT_B,                    // 左移阶段B(开关=0 时=整段; =1 时=前半)
    STATE_7B_MOVE_FWD_MID,                  // (仅开关=1 用) B 段中间: 车头前进 ROUTE_7_LEFT_B_FWD_MM
    STATE_7_MOVE_LEFT_B2,                   // (仅开关=1 用) 左移阶段B【后半】
    STATE_7A_INTERMEDIATE_STOP,             // 中间停顿
    STATE_7B_MOVE_LEFT_C,                   // 左移阶段C(⚠ 当前不在流程里: 动作被注释, 只穿过)
    STATE_7A_HEADING_CORRECTION,            // 航向校正(⚠ 当前是空动作, 只穿过)

    STATE_8_MOVE_FORWARD_A,                 // 直线前进进入排爆区
    STATE_8A_HEADING_CORRECT,               // [新增] 前进980后航向校正, 保证右移前车头正
    STATE_8B_ADJUST_RIGHT,                  // 右移微调 (原 STATE_8A)
    STATE_8C_HEADING_CORRECT,               // [新增] 右移600后航向校正, 保证 -99° 相对转向从 0° 起算

    /* 阶段二: 排爆 */
    STATE_9_TURN_FOR_BOMB,                  // 转向排爆点
    STATE_9A_ADJUST_LEFT,                   // 左移微调
    STATE_10_APPROACH_BOMB,                 // 接近炸弹
    STATE_11_PERFORMING_BOMB_DISPOSAL,      // 视觉对准并抓取/放置炸弹
    STATE_11A_HEADING_CORRECT,              // ⭐新增: 排爆做完进打靶走位之前: 挪 ROUTE_BOMB_AFTER_STEP_MM 再校到 0°

    /* 阶段三: 打靶 (2026-10-05 定稿; ⭐⭐ 2026-10-10 改为"一条路 + 中间校一次 + 最后一次")
     *   ⭐⭐ 2026-10-10(用户要求): 走位由两个开关切换
     *      (ROUTE_12_SPLIT_ENABLE / ROUTE_12_MID_CORRECT_ENABLE):
     *      SPLIT=0, MID=1(当前): STATE_11A(出发前先校 0°)
     *               → MOVE_A(右移【前半】)
     *               → CORRECT_B【中间那次: 先原地摆正 0°, 再挪一小步(后退)】
     *               → MOVE_B(右移【后半】)
     *               → CORRECT_B2(最后一次校 + 停稳)
     *               → MOVE_C(摆 TARGET_LOOK) → STATE_13 打靶视觉对准
     *               ⇒ 从排爆结束到打靶共 3 次校正(11A + 中间 + 最后), 中途只停一次;
     *                 CORRECT_A / MOVE_A2 / CORRECT_A2 / MOVE_B2 不会被进入。
     *      SPLIT=0, MID=0 : 一条路走完, 中途不校正(只剩 CORRECT_B2 那一次);
     *      SPLIT=1        : 老走法 —— 每段右移再【对半拆开走】, 每一半走完都校一次 = 4 次:
     *                 MOVE_A → CORRECT_A → MOVE_A2 → CORRECT_A2
     *                 → MOVE_B → CORRECT_B → MOVE_B2 → CORRECT_B2 → MOVE_C
     *      ⚠️ 三种走法总距离相同 ⇒ 终点位置不变。
     *   ⚠️ 校正前那一步“挪一小步”各有【自己的】固定步长宏:
     *        CORRECT_A = ROUTE_12_A_MINSTEP_MM    CORRECT_A2 = ROUTE_12_A2_MINSTEP_MM
     *        CORRECT_B = ROUTE_12_B_MINSTEP_MM(★中间那次用它, 当前 -15)
     *        CORRECT_B2 = ROUTE_12_B2_MINSTEP_MM(最后那次, 当前 0 = 不挪)
     *      总开关 = ROUTE_12_MINSTEP_ENABLE(当前 = 1); 第 2 / 第 4 处还能改成
     *      “按偏航角选步长”的方案二(ROUTE_12_A2_STEP_MODE / ROUTE_12_B2_STEP_MODE,
     *      当前都是 0 = 方案一固定值)。详见 MissionControl.c 的宏注释。
     *   视觉对准(任务2): 底盘完全不动, 根据 K230 回的 C/L/R 原地小步转底座 ID1;
     *           收到 C 后依次摆 FIRE → LIFT → SCAN_RESET。
     *   收尾(⭐ 2026-10-07 实测: 顺序不能颠倒): 出了 STATE_13 先走三段平移 ——
     *         PART2_MOVE_A(右移 ROUTE_12_P2_A_MM, 正好到车场【拐角】)
     *         → PART2_MOVE_BACKWARD(后退 ROUTE_12_P2_BACK_MM=50)
     *         → PART2_CORRECT_A(航向校正到 0°)
     *         → 才进救援阶段: STATE_14/15/15C(右转90° + 校准到-90° + 停稳3s)。
     *         ⚠️ 转 90° 必须排在这三段之后: 只有拐角那儿的转弯余量才够。
     * ⚠️ TURN_A/TURN_B 与 PART2_MOVE_B/CORRECT_B/MOVE_C 不在流程里(不可达),
     *    PART1_MOVE_A/A2/B/B2、CORRECT_A/A2/B/B2、MOVE_C 全是【在用】的,
     *    别当废弃删掉。
     * ⚠️ 这几个状态是【顺序推进】的(g_mission_state++), 枚举顺序不可打乱:
     *    MOVE_A → CORRECT_A → MOVE_A2 → CORRECT_A2 → MOVE_B → CORRECT_B
     *          → MOVE_B2 → CORRECT_B2 → MOVE_C */
    STATE_12_PART1_MOVE_A,                   // 右移【前半】(SPLIT=1 时 = 第1段前半)
    STATE_12_PART1_CORRECT_A,                // (仅 SPLIT=1 用) 第1处航向校正(先挪 ROUTE_12_A_MINSTEP_MM)
    STATE_12_PART1_MOVE_A2,                  // (仅 SPLIT=1 用) 第1段右移 后半
    STATE_12_PART1_CORRECT_A2,               // (仅 SPLIT=1 用) 第2处航向校正(先挪 ROUTE_12_A2_MINSTEP_MM)
    STATE_12_PART1_MOVE_B,                   // 右移【后半】(MID=1 时走这里; SPLIT=1 时 = 第2段前半)
    STATE_12_PART1_CORRECT_B,                // 中间那次航向校正(SPLIT=0: 先摆正再挪 ROUTE_12_B_MINSTEP_MM; =1: 第3处)
    STATE_12_PART1_MOVE_B2,                  // (仅 SPLIT=1 用) 第2段右移 后半
    STATE_12_PART1_CORRECT_B2,               // 最后一次航向校正(先挪 ROUTE_12_B2_MINSTEP_MM) + 停稳 TARGET_STOP_SETTLE_MS
    STATE_12_PART1_MOVE_C,                   // 摆 ARM_POSE_TARGET_LOOK
    STATE_12_TURN_A,
    STATE_12_TURN_B,
    STATE_12_PART2_MOVE_A,
    STATE_12_PART2_MOVE_BACKWARD,            // ⭐ 打靶收尾第2段 = 后退 ROUTE_12_P2_BACK_MM
                                             //   (车头此刻还是 0°, 所以是“后退”)
    STATE_12_PART2_CORRECT_A,
    STATE_12_PART2_MOVE_B,
    STATE_12_PART2_CORRECT_B,
    STATE_12_PART2_MOVE_C,
    STATE_13_PERFORMING_TARGETING,          // 视觉: 原地转底座 ID1 对准靶子, 然后摆 FIRE/LIFT/SCAN_RESET

    /* 阶段四: 救援 (2026-10-07 改为「先掉头, 再横移进救援区」)
     * ⭐⭐ 2026-10-07 实测(顺序不能颠倒): ①~③【必须】排在打靶收尾三段平移之后 ——
     *    那段右移到位后小车正好到车场【拐角】, 只有那儿原地转 90° 的余量才够。
     * 流程: ①车头右转 90°(RESCUE_HEADING_DEG, 同时设航向基准)
     *      → ①'【挪一步 ROUTE_RESCUE_AFTER_TURN_MM = 75mm(车头前进)】, 再校准/停稳/右移进救援区
     *      → ②航向校准到 -90°
     *      → ③校准【到位】后原地停稳 RESCUE_ALIGN_SETTLE_MS(3s)
     *      → ③'⭐2026-10-09: 右移前【再纯校一次航向】(不挪步 —— 原地右转 90°
     *         可能把车身带偏, 横移前按绝对角再校一次; 见 ROUTE_RESCUE_PRE_RIGHT_STEP_MM)
     *      → ④右移进救援区(⭐ 2026-10-09: 拆成两段 ROUTE_14_MID_MM + ROUTE_14_MID2_MM,
     *         中途插一次“挪一步 + 航向校准到 -90°”) → ⑤停下等 3s
     *      → ⑥到人质处: 先退 ROUTE_RESCUE_GRAB_BACK_MM(-15) 再摆 HOSTAGE_LOOK
     *      → ⑦视觉对准 + 抓取(⭐ 2026-10-07: 粗对准 C 之后还要发 start_align,
     *         走 D:x,y 自适应精对准, 收敛到 ±RESCUE_ID1_ALIGN_TOL=20px 再抓;
     *         见 RESCUE_USE_FINE_ALIGN)
     *         ⭐⭐ 2026-10-10(用户要求): 抓取【位/抱紧位】还会按 K230 回的
     *         Y 轴(前后)像素误差给机械臂 ID2 补“里程”(伸出去多一点/少一点),
     *         参数 = RESCUE_FB_ARM_*(与抓球/放桶同一机制、独立参数);
     *         同一帧的 x(横向)用于给底座 ID1 补对准残余(RESCUE_GRAB_*)。
     *      ⭐⭐ 2026-10-10(用户要求): 抓完人质 → 【先原地校一次航向】→ 一路走到终点,
     *      由 RESCUE_TAIL_ONESHOT 切换(当前 = 1):
     *        1(当前): ⑧只校航向(不挪步: ROUTE_RESCUE_FWD_MM = 0, 只按绝对角摆正到 -90°)
     *                 → ⑩【一段走完】ROUTE_17_RIGHT_B_MM + ROUTE_18_RIGHT_C_MM
     *                 → ⑫停下(任务完成)。中途不再停车、不再校正, ⑨/⑪ 不会被进入。
     *        0      : ⑧右移 ROUTE_17_RIGHT_B_MM → ⑨航向校准
     *                 → ⑩右移 ROUTE_18_RIGHT_C_MM → ⑪航向校准 → ⑫停下 (老做法)
     * ⭐⭐ 2026-10-10(用户要求): ⑥ 到人质处、开始识别/伸臂【之前】先退一小步 —— 它【不是
     *    单独一个状态】, 而是并进 STATE_16_RESCUE_RIGHT_A 的进入动作里(宏
     *    ROUTE_RESCUE_GRAB_BACK_MM, 默认 -15 = 车头后退 15mm; 0 = 关掉这一步)。
     *    目的: 别让夹爪/车体蹭到人质架, 也给伸臂/识别/抓取留余量(退的这一步会被
     *    “按 Y 误差补 ID2 里程”吸收一部分)。
     * ⭐ 航向纠正之前都先挪一小步(Route_MinStep, 见宏):
     *    ② = ROUTE_RESCUE_AFTER_TURN_MM(转完后车头前进)
     *    ④' = ROUTE_RESCUE_MID_STEP_MM (-15 = 车头后退 ⇒ 这段右移【少走】约 15mm)
     *    ⑨ = ROUTE_RESCUE_FWD_MM   ⑪ = ROUTE_RESCUE_LAST_STEP_MM
     *    ⚠️ 走到这几处时车头已经是 -90° ⇒ 这里的“车头前进”挪的是【场地“右”】方向、
     *       “车头后退”挪的是【场地“左”】方向(与④⑧⑩的右移同向 / 反向), 不是朝场地
     *       前后; 所以 ④' 那一步 -15 等于让这段右移少走约 15mm。
     * ⭐ 为什么“后退”全改成“右移”: 车头右转 90°(顺时针)之后, 车体的【右】方向
     *    正好等于原来的【后】方向 ⇒ 轨迹完全不变, 只是车身姿态转了 90°
     *    (机械臂/摄像头的朝向随之改变)。 */
    STATE_14_MOVE_FORWARD_B,                 // ① 车头右转 90°(名字沿用历史: 原来是“后退”)
    STATE_15_TURN_FOR_HOSTAGE,               // ①'先挪 ROUTE_RESCUE_AFTER_TURN_MM(75mm 前进), 再②航向校准到 -90° + 同步航向基准
    STATE_15C_RESCUE_ALIGN_SETTLE,           // ③ ⭐新增: 校准到位后原地停稳 RESCUE_ALIGN_SETTLE_MS(3s)
    STATE_15D_RESCUE_RECORRECT,              // ⭐新增: ③' 停稳后再【纯校一次】(不挪步, 见 ROUTE_RESCUE_PRE_RIGHT_STEP_MM)
    STATE_15B_RESCUE_APPROACH_RIGHT,         // ④ 右移【第一段】ROUTE_14_MID_MM(到中途校准点)
    STATE_15B2_RESCUE_MID_CORRECT,           // ⭐新增: 中途一次“挪 ROUTE_RESCUE_MID_STEP_MM + 航向校准到 -90°”
    STATE_15B3_RESCUE_APPROACH_RIGHT2,       // ⭐新增: 右移【第二段】ROUTE_14_MID2_MM(走完就进救援区)
    STATE_15A_RESCUE_STOP_WAIT,              // ⑤ 原地停等 RESCUE_STOP_WAIT_MS(3000ms)
    STATE_16_RESCUE_RIGHT_A,                 // ⑥ 到人质处: 进入动作 = 先退 ROUTE_RESCUE_GRAB_BACK_MM + 摆 ARM_POSE_HOSTAGE_LOOK
    STATE_16A_RESCUE_HEADING_CORRECT,        // (未使用) 备用航向校正
    STATE_17_RESCUE_RIGHT_B,                 // ⑧ (开关=1: 只校航向不移动; =0: 第 1 段右移 ROUTE_17_RIGHT_B_MM)
    STATE_17A_RESCUE_HEADING_CORRECT,        // (仅 RESCUE_TAIL_ONESHOT=0 用) ⑨ 先挪 ROUTE_RESCUE_FWD_MM, 再校到 -90°
    STATE_18_RESCUE_RIGHT_C,                 // ⑩ (开关=1: 一段走完 ⑧+⑩ 合并; =0: 第 2 段右移 ROUTE_18_RIGHT_C_MM)
    STATE_18A_RESCUE_HEADING_CORRECT,        // (仅开关=0 用) ⑪ 先挪 ROUTE_RESCUE_LAST_STEP_MM, 再校到 -90°
    STATE_19_RESCUE_RIGHT_D,                 // ⑫ 停下 → MISSION_STATE_COMPLETE
    STATE_19A_RESCUE_HEADING_CORRECT,        // (未使用)
    STATE_20_PERFORMING_HOSTAGE_RESCUE,      // ⑦ 底盘不动, 交给视觉子状态(任务 3=救援)
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
#define ARM_POSE_COUNT  30
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
    ARM_POSE_HOSTAGE_LIFT,  /* 抱起人质后大臂抬起 */

    /* ⭐ 救援抓取位【三选一】(2026-10-06 新增) ----------------------------
     * 救援时底盘不动, 靠底座 ID1 小步转对准人质(HOSTAGE_LOOK 为基准)。
     * 人质偏左/偏右时 ID1 会多转几十~几百码, 若抓取姿态只有一套就抱不准,
     * 所以按 ID1 的【累计偏移量】在下面三个姿态里挑一个执行:
     *     |偏移| ≤ RESCUE_GRAB_MID_RANGE            → GRAB_M(中间抓取位)
     *     偏移与 RESCUE_GRAB_LEFT_SIGN 同号且超范围 → GRAB_L(向左抓取)
     *     否则                                      → GRAB_R(向右抓取)
     * ⚠️ 三行初值都填成 = HOSTAGE_PRE, 所以改装后行为与以前【完全一致】;
     *    实测时用示教模式(KEY2长按进入 → KEY2切换 → KEY1读出)挨个标定。 */
    ARM_POSE_HOSTAGE_GRAB_L,/* 抓取位[左]: 人质偏左 */
    ARM_POSE_HOSTAGE_GRAB_M,/* 抓取位[中]: 人质居中 */
    ARM_POSE_HOSTAGE_GRAB_R,/* 抓取位[右]: 人质偏右 */

    /* ⭐ 上述三个抓取位各自的【抱紧】与【抬起】(2026-10-06 新增) --------
     * 为什么拆成 9 个: 三个方向的手臂姿态差得很大(左/右位 ID1 差 400 码≈35°,
     *   ID2/ID3/ID4 也各不相同), 所以“合夹爪抱紧”和“抱起后抬臂”在这三个
     *   姿态上并不是同一个动作 —— 用一套会拉回中间位或蹭到车架。
     * 执行顺序(共 3 步, 由 Arm_Start_Rescue_Grab / _Retract 驱动):
     *     ① GRAB_x  摆到该方向的抓取位
     *     ② GRAB_x_CLOSE 在原位合夹爪抱紧(通常只 ID5 从张开→闭合,
     *                     其余 4 个应与 GRAB_x 相同或仅降几码)
     *     ③ GRAB_x_LIFT  抱起后抬大臂收尾(之后 → 后退)
     * ⚠️ 初值: CLOSE = GRAB + (HOSTAGE_CLOSE - HOSTAGE_PRE), LIFT = HOSTAGE_LIFT
     *    ⇒ 与拆开前的行为【完全一致】, 等你示教后逐个替换即可。
     * ⚠️ 示教建议: 先示教 GRAB_x → 再在 GRAB_x 上手动合夹爪记录 GRAB_x_CLOSE
     *    → 最后抱起记录 GRAB_x_LIFT(每行只改一行, 别动别的行)。 */
    ARM_POSE_HOSTAGE_GRAB_L_CLOSE, /* 抱紧[左] */
    ARM_POSE_HOSTAGE_GRAB_M_CLOSE, /* 抱紧[中] */
    ARM_POSE_HOSTAGE_GRAB_R_CLOSE, /* 抱紧[右] */
    ARM_POSE_HOSTAGE_GRAB_L_LIFT,  /* 抬起[左] */
    ARM_POSE_HOSTAGE_GRAB_M_LIFT,  /* 抬起[中] */
    ARM_POSE_HOSTAGE_GRAB_R_LIFT,  /* 抬起[右] */

    /* ⭐ 救援回程姿态 (2026-10-06 新增) --------------------------------
     * 抱起人质、大臂抬起之后执行【这一张】收臂姿态, 把臂收到“适合带着
     * 人质后退”的位置(三个方向共用一套)。
     * 顺序: ①GRAB_x → ②GRAB_x_CLOSE → ③GRAB_x_LIFT → ④RETURN → 后退 */
    ARM_POSE_HOSTAGE_RETURN /* 抱起后的回程姿态(收臂, 准备后退) */
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
/* 链路监控(值 5)发送: which 0=reset:0, 1=run_task:1(球), 2=run_task:2(靶) */
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
