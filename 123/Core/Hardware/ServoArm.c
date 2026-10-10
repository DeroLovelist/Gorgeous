/**
 * @file    ServoArm.c
 * @brief   飞特(Feetech) STS/SMS 串行舵机机械臂控制模块
 *
 * 底层: SCSLib(SCServo) -> USART2(PD5/PD6) -> 飞特官方驱动板(纯串口, 无方向脚)
 * 本文件只做"机械臂级"封装, 具体动作序列(抓取/放置)在 MissionControl.c 中编排。
 */

#include "ServoArm.h"
#include "main.h"
#include "SCServo.h"
#include "uart.h"

/* =====================================================================
 * ⚡ 大步长防反转保护 (默认关闭, 要用的时候把下面宏改成 1)
 *
 * 背景: 飞特 SMS/STS 在位置模式(0~4095)下是按【最短路径】转向目标位置的。
 *       目标与“舵机实际所在位置”相差超过 2048 步(半圈)时, 舵机会朝
 *       反方向转(接近一整圈!) → 机械臂可能撞车架/摄像头/限位。
 *       本项目标定值目前最大跳变 2029 步, 离 2048 只差 19 步(≈1.7°),
 *       一旦实际起点与预期不符(上电位置不同/上次动作被打断/手推过/
 *       某条指令丢包), 就会触发反向甩半圈。
 *
 * 方案一 SERVO_LONG_JUMP_WARN  = 1:
 *   跳变超阈值时只在蓝牙日志里打告警, 不动舵机(先排查用)。
 * 方案二 SERVO_LONG_JUMP_SPLIT = 1:
 *   跳变超阈值时自动插入“同方向的中间点”, 分两段发送, 强制舵机按
 *   指令方向转, 不会反向甩半圈(每段会等走完再发下一段, 会慢一点)。
 * ---------------------------------------------------------------------
 * 两个宏可以同时开(SPLIT 打开时 WARN 的代码不再重复编译, SPLIT 内部
 * 已经打了同样的告警日志)。
 * ===================================================================== */
#define SERVO_LONG_JUMP_WARN    0   /* 1=只告警 */
#define SERVO_LONG_JUMP_SPLIT   0   /* 1=自动拆两段(强制方向) */
#define SERVO_LONG_JUMP_LIMIT   1800   /* 判定阈值(步): 建议 1700~2000 */
#define SERVO_LONG_JUMP_SEG_MS  1200   /* 拆分后每段的运动时间(ms) */

/* =====================================================================
 * ⭐ 速度 / 加速度 / 上电时序
 *   4096 步 = 360°, 所以“步/秒”换算角度: 度/秒 = 步/秒 ÷ 4096 × 360。
 * ===================================================================== */
#define SERVO_SPEED_MIN         SERVO_SPEED_MIN_DEF   /* 速度下限(步/秒) ≈ 17.6°/s。
                                        * 用于“短距离/距离算不出来”时的兜底速度。
                                        * ⚠️ 别设太小(原来 50 ≈ 4.4°/s): 上电回位时
                                        * 舵机可能离目标很远, 下限过小会爬几十秒都到不了;
                                        * 也别设太大(>500)免得短动作冲得猛。建议 150~250
                                        * ⭐ 2026-10-10: 数值搬到了 ServoArm.h 的
                                        *    SERVO_SPEED_MIN_DEF —— 调用方要用它当“默认值”
                                        *    传给 Servos_SetPositionsMaskedEx()
                                        *    (精对准微动传更小的下限, 见那里的说明) */
#define SERVO_SPEED_MAX         4095   /* 速度上限(步/秒) = 协议满速 */
#define SERVO_ACC_DEFAULT       SERVO_ACC_DEF   /* 加速度(0~254): 值越小起步越柔。
                                                 * 50 比较温和(定义在 ServoArm.h) */
#define SERVO_POWER_ON_DELAY_MS 500    /* 上电后等舵机/驱动板启动的时间(ms)。
                                        * MCU 往往比舵机先上电, 不等的话最初几条
                                        * 指令会丢(回位指令收不到 → 停在原地) */
#define SERVO_HOME_MOVE_MS      10000  /* 上电/复位“回初始姿态”的运动时间(ms):
                                        * 值大 = 走得慢 = 安全。10000 时约 200 步/秒,
                                        * 和正常姿态切换的速度接近 */

#if SERVO_LONG_JUMP_WARN || SERVO_LONG_JUMP_SPLIT
#include "elog.h"
#define SERVO_LOG_W(...)   elog_w("SERVO", __VA_ARGS__)
#else
#define SERVO_LOG_W(...)   do { } while (0)
#endif

/* 舵机 ID 列表: [0]=ID1(底座) [1]=ID2(大臂) [2]=ID3(辅助) [3]=ID4(辅助) [4]=ID5(夹爪) */
static const uint8_t s_servo_ids[SERVO_COUNT] = {1, 2, 3, 4, 5};

/* 最近一次写入的目标位置, 用于由 time_ms 换算速度 */
static uint16_t s_last_pos[SERVO_COUNT] = {0, 0, 0, 0, 0};

/* 最近一次读到的位置 */
static volatile int32_t s_read_pos[SERVO_COUNT] = {-1, -1, -1, -1, -1};

/* 初始姿态(0~4095): 底座3926 / 大臂2479 / 辅助752 / 辅助1287 / 夹爪800(张开)
 * ⚠️ 非 const: 示教标定 HOME 时通过 Servos_SetHomePositions() 更新 */
//id2限幅（350~1900）id3限幅（900~3100）id4限幅（1050~3010）id5限幅（746张开~1397闭合）
static uint16_t SERVO_POS_HOME[SERVO_COUNT] = {  2052, 2274, 810, 1413, 93 };

/**
 * @brief  由运动距离与时间换算舵机速度(步/秒), 并指定“速度下限”
 * @param  speed_min 本次允许的最低速度(步/秒): 算出来的速度低于它就用它。
 *                   ⭐ 2026-10-10 新增(供精对准微动用): 传一个小的下限
 *                   (如 20)时, 几码的微动才会按 time_ms 慢慢走完, 而不是
 *                   被 200 步/秒的通用下限钳成二十几毫秒的“抽搐”。
 */
static uint16_t ServoArm_CalcSpeedEx(uint16_t from, uint16_t to, uint16_t time_ms,
                                     uint16_t speed_min)
{
    uint32_t dist;
    uint32_t speed;

    if (from > to) {
        dist = (uint32_t)(from - to);
    } else {
        dist = (uint32_t)(to - from);
    }

    if (time_ms == 0) {
        return 4095;
    }

    speed = (dist * 1000U) / time_ms;   /* 步/秒 */
    if (speed < speed_min) {
        speed = speed_min;              /* 兜底: 距离很小/算成 0 时也别用蠕动速度 */
    }
    if (speed > SERVO_SPEED_MAX) {
        speed = SERVO_SPEED_MAX;
    }
    return (uint16_t)speed;
}

/**
 * @brief  由运动距离与时间换算舵机速度(步/秒) —— 用通用速度下限
 */
static uint16_t ServoArm_CalcSpeed(uint16_t from, uint16_t to, uint16_t time_ms)
{
    return ServoArm_CalcSpeedEx(from, to, time_ms, SERVO_SPEED_MIN);
}

/**
 * @brief  初始化机械臂
 */
void ServoArm_Init(void)
{
    Uart_Init(115200);   /* 舵机串口 USART2(纯串口, 无方向脚) */

    /* ⚠️ 舵机/驱动板上电比 MCU 慢, 立刻发指令容易丢(表现就是“上电不动”),
     * 先等它启动完再通信 */
    HAL_Delay(SERVO_POWER_ON_DELAY_MS);

    /* ⭐ 关键: 先读回舵机的【真实当前位置】当起点。
     * 以前这里是直接写 s_last_pos[i] = SERVO_POS_HOME[i], 于是下面回位指令的
     * from == to → 距离算成 0 → 速度被钳到下限, 回位慢得离谱/看起来“到不了位”。 */
    Servos_ReadPositions();
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        int32_t p = Servos_GetPosition(s_servo_ids[i]);
        /* 读失败(舵机未应答)才退回初始姿态, 避免拿 0 当起点算速度 */
        s_last_pos[i] = (p >= 0 && p <= 4095) ? (uint16_t)p : SERVO_POS_HOME[i];
    }

    /* 上电使能扭矩并回到初始姿态(慢一点, 防剐蹭) */
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        Servos_SetTorque(s_servo_ids[i], 1);
    }
    Servos_SetPositions((uint16_t *)SERVO_POS_HOME, SERVO_HOME_MOVE_MS);
}

/**
 * @brief  裸同步写: 只把 mask 选中的舵机写到目标位置(不含任何保护/拆段),
 *         并可【按次】指定速度下限/加速度
 * @param  from      上一次指令位置(用来换算本段速度)
 * @param  to        目标位置(调用方保证已限幅到 0~4095)
 * @param  time_ms   期望本段运动时间(ms), 0=最快
 * @param  mask      舵机选择掩码(SERVO_MASK_ID1..ID5 按位或)
 * @param  speed_min 本次速度下限(步/秒), 见 ServoArm_CalcSpeedEx()
 * @param  acc_val   本次加速度(0~254)
 * @note   只同步更新被写入舵机的 s_last_pos
 */
static void Servos_WriteRawMaskEx(const uint16_t from[SERVO_COUNT],
                                  const uint16_t to[SERVO_COUNT],
                                  uint16_t time_ms, uint8_t mask,
                                  uint16_t speed_min, uint8_t acc_val)
{
    uint8_t ids[SERVO_COUNT];
    int16_t pos[SERVO_COUNT];
    uint16_t spd[SERVO_COUNT];
    uint8_t acc[SERVO_COUNT];
    uint8_t n = 0;

    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        if ((mask & (uint8_t)(1u << i)) == 0) {
            continue;   /* 这一步不动它 */
        }
        ids[n] = s_servo_ids[i];
        pos[n] = (int16_t)to[i];
        spd[n] = ServoArm_CalcSpeedEx(from[i], to[i], time_ms, speed_min);
        acc[n] = acc_val;
        s_last_pos[i] = to[i];
        n++;
    }
    if (n == 0) {
        return;   /* 一个舵机都没选中: 不发空包 */
    }

    SyncWritePosEx(ids, n, pos, spd, acc);
}

/**
 * @brief  裸同步写(用通用速度下限/加速度) —— 与以前完全一致
 * @note   只有“大步长拆两段”(SERVO_LONG_JUMP_SPLIT=1)那个分支用到它,
 *         所以跟着那个开关一起编译, 免得默认配置下报“函数未使用”。
 */
#if SERVO_LONG_JUMP_SPLIT
static void Servos_WriteRawMask(const uint16_t from[SERVO_COUNT],
                                const uint16_t to[SERVO_COUNT],
                                uint16_t time_ms, uint8_t mask)
{
    Servos_WriteRawMaskEx(from, to, time_ms, mask, SERVO_SPEED_MIN, SERVO_ACC_DEFAULT);
}
#endif

/**
 * @brief  设置位置(可只动 mask 选中的舵机), 并可【按次】指定速度下限/加速度
 * @note   默认行为 = 直接发一条同步写指令。
 *         若打开 SERVO_LONG_JUMP_SPLIT(见文件头), 大步长的舵机会自动
 *         插一个“同方向中间点”分两段走, 防止舵机按最短路径反向甩半圈。
 *         speed_min 见 ServoArm_CalcSpeedEx(), acc_val 见协议(0~254)。
 * ⭐ 2026-10-10 新增(供“精对准微动”用): 通用下限 200 步/秒会把几码的微动
 *    钳成二十几毫秒的“抽搐”, 手臂惯性 ⇒ 过冲+余振 ⇒ 画面抖、反复修。
 */
static void Servos_SetPositionsMaskEx(const uint16_t positions[SERVO_COUNT],
                                      uint8_t mask, uint16_t time_ms,
                                      uint16_t speed_min, uint8_t acc_val)
{
    uint16_t from[SERVO_COUNT];
    uint16_t to[SERVO_COUNT];

    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        from[i] = s_last_pos[i];
        to[i]   = (positions[i] > 4095) ? 4095 : positions[i];
    }

#if SERVO_LONG_JUMP_SPLIT
    /* ---- 方案二: 大步长拆两段(强制按指令方向转) ---- */
    {
        uint16_t mid[SERVO_COUNT];
        uint8_t  need_split = 0;

        for (uint8_t i = 0; i < SERVO_COUNT; i++) {
            int32_t d;

            if ((mask & (uint8_t)(1u << i)) == 0) {
                mid[i] = from[i];   /* 这一步不动的舵机, 中间点取原值 */
                continue;
            }
            d = (int32_t)to[i] - (int32_t)from[i];

            if (d > SERVO_LONG_JUMP_LIMIT || d < -SERVO_LONG_JUMP_LIMIT) {
                mid[i] = (uint16_t)((int32_t)from[i] + d / 2);   /* 同方向的中间点 */
                need_split = 1;
                SERVO_LOG_W("ID%d jump %ld steps (%u->%u): split at %u",
                            (int)s_servo_ids[i], (long)d,
                            (unsigned)from[i], (unsigned)to[i], (unsigned)mid[i]);
            } else {
                mid[i] = to[i];
            }
        }

        if (need_split) {
            Servos_WriteRawMask(from, mid, SERVO_LONG_JUMP_SEG_MS, mask);   /* 第一段 */
            HAL_Delay(SERVO_LONG_JUMP_SEG_MS + 100);                       /* 等走完再发第二段 */
            Servos_WriteRawMask(mid, to, SERVO_LONG_JUMP_SEG_MS, mask);     /* 第二段 */
            return;
        }
    }
#elif SERVO_LONG_JUMP_WARN
    /* ---- 方案一: 只告警, 不动舵机 ---- */
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        int32_t d;

        if ((mask & (uint8_t)(1u << i)) == 0) {
            continue;
        }
        d = (int32_t)to[i] - (int32_t)from[i];

        if (d > SERVO_LONG_JUMP_LIMIT || d < -SERVO_LONG_JUMP_LIMIT) {
            SERVO_LOG_W("ID%d jump %ld steps (%u->%u): servo may turn the SHORT way!",
                        (int)s_servo_ids[i], (long)d,
                        (unsigned)from[i], (unsigned)to[i]);
        }
    }
#endif

    Servos_WriteRawMaskEx(from, to, time_ms, mask, speed_min, acc_val);
}

/**
 * @brief  设置位置(可只动 mask 选中的舵机) —— 用通用速度下限/加速度
 * @note   与上面 Servos_SetPositionsMaskEx() 完全等价, 只是参数取默认值。
 */
static void Servos_SetPositionsMask(const uint16_t positions[SERVO_COUNT],
                                    uint8_t mask, uint16_t time_ms)
{
    Servos_SetPositionsMaskEx(positions, mask, time_ms,
                              SERVO_SPEED_MIN, SERVO_ACC_DEFAULT);
}

/**
 * @brief  设置所有舵机位置(同步写)
 */
void Servos_SetPositions(uint16_t positions[SERVO_COUNT], uint16_t time_ms)
{
    Servos_SetPositionsMask(positions, SERVO_MASK_ALL, time_ms);
}

/**
 * @brief  只设置 mask 选中的舵机位置(用于“分步动作”)
 * @param  positions 长度为 SERVO_COUNT 的目标位置数组
 * @param  mask      舵机选择掩码, 见 ServoArm.h 的 SERVO_MASK_*
 * @param  time_ms   期望运动时间(ms), 0=最快
 * @note   未被选中的舵机保持不动; 它们各自的“上次位置”也不更新。
 *         典型用法: 先 SERVO_MASK_ARM_BODY(ID1/2/3) 到位,
 *         再 SERVO_MASK_WRIST_GRIP(ID4/5) —— 见 MissionControl 的 Arm_GotoPoseSplit()。
 */
void Servos_SetPositionsMasked(const uint16_t positions[SERVO_COUNT],
                               uint8_t mask, uint16_t time_ms)
{
    Servos_SetPositionsMask(positions, mask, time_ms);
}

/**
 * @brief  只设置 mask 选中的舵机位置, 并【按次】指定速度下限与加速度
 * @param  positions 长度为 SERVO_COUNT 的目标位置数组 (0~4095)
 * @param  mask      舵机选择掩码, 见 ServoArm.h 的 SERVO_MASK_*
 * @param  time_ms   期望运动时间(ms), 0=最快
 * @param  speed_min 本次速度下限(步/秒), 见 ServoArm.h 的 SERVO_SPEED_MIN_DEF
 * @param  acc       本次加速度(0~254)
 * @note   用途/取舍见 ServoArm.h 里本函数的声明注释。
 */
void Servos_SetPositionsMaskedEx(const uint16_t positions[SERVO_COUNT], uint8_t mask,
                                 uint16_t time_ms, uint16_t speed_min, uint8_t acc)
{
    Servos_SetPositionsMaskEx(positions, mask, time_ms, speed_min, acc);
}

/**
 * @brief  更新上电初始姿态(示教标定 HOME 时同步)
 */
void Servos_SetHomePositions(const uint16_t positions[SERVO_COUNT])
{
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        SERVO_POS_HOME[i] = positions[i];
    }
}

/**
 * @brief  获取上电初始姿态数组指针(唯一来源)
 */
uint16_t *Servos_GetHomePositions(void)
{
    return SERVO_POS_HOME;
}

/**
 * @brief  设置单个舵机位置
 */
void Servos_SetSinglePosition(uint8_t servo_id, uint16_t position, uint16_t time_ms)
{
    uint8_t idx;
    uint16_t speed;

    if (servo_id < SERVO_ARM_ID_MIN || servo_id > SERVO_ARM_ID_MAX) {
        return;
    }
    idx = servo_id - 1;

    if (position > 4095) {
        position = 4095;
    }
    speed = ServoArm_CalcSpeed(s_last_pos[idx], position, time_ms);
    s_last_pos[idx] = position;

    WritePosEx(servo_id, (int16_t)position, speed, 50);
}

/**
 * @brief  所有舵机掉电卸力
 */
void Servos_UnloadAll(void)
{
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        EnableTorque(s_servo_ids[i], 0);
    }
}

/**
 * @brief  读取所有舵机当前位置
 */
void Servos_ReadPositions(void)
{
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        int pos = ReadPos(s_servo_ids[i]);
        s_read_pos[i] = pos;
    }
}

/**
 * @brief  获取最近一次读到的位置
 */
int32_t Servos_GetPosition(uint8_t servo_id)
{
    if (servo_id < SERVO_ARM_ID_MIN || servo_id > SERVO_ARM_ID_MAX) {
        return -1;
    }
    return s_read_pos[servo_id - 1];
}

/**
 * @brief  使能/关闭指定舵机扭矩
 */
void Servos_SetTorque(uint8_t servo_id, uint8_t enable)
{
    if (servo_id < SERVO_ARM_ID_MIN || servo_id > SERVO_ARM_ID_MAX) {
        return;
    }
    EnableTorque(servo_id, enable ? 1 : 0);
}
