#ifndef __PID_H
#define __PID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =================================================================
 * PID 算法模块 (独立库, 不依赖具体硬件)
 * -----------------------------------------------------------------
 * 1) DualPID_Controller : 串级 PID (位置环在外 + 速度环在内), 用于单个电机
 * 2) pid_param_t        : 单环位置式 PID, 用于底盘转向(陀螺仪闭环)
 * ================================================================= */

/**
 * @brief 串级 PID 控制器 (位置环 + 速度环)
 *
 * 控制结构（从外到内两层）:
 *   位置误差(目标-当前编码器计数) --位置环--> 期望速度(计数/周期)
 *                                                   |
 *   期望速度 - 实测速度(编码器增量) --速度环--> PWM 输出(驱电机)
 *
 * ⚠️ 除 init 函数传入的参数外，其余字段均由算法内部维护，请勿手动修改。
 * 以下"取值范围/推荐值"为工程实测经验区间，若需微调请从 Chassis.c 的
 * Chassis_Init() 中的初值开始小幅尝试，每次只改一个量，便于判断影响。
 */
typedef struct
{
    /* ================= 位置环参数（最外层） ================= */
    float pos_Kp, pos_Ki, pos_Kd;

    float pos_integral;      /* [内部] 位置环积分累计，勿手调 */
    float pos_error_prev;    /* [内部] 上一次位置误差，勿手调 */

    /* ================= 速度环参数（内层） ================= */
    float vel_Kp, vel_Ki, vel_Kd;

    float vel_integral;      /* [内部] 速度环积分累计，勿手调 */
    float vel_error_prev;    /* [内部] 上一次速度误差，勿手调 */

    /* ================= 限幅 ================= */
    float max_vel;           /* 位置环输出的最大期望速度 (编码器计数/控制周期) */
    float max_output;        /* 速度环输出的最大 PWM 占空比 (0~99) */

    /* ================= 前馈 ================= */
    float vel_ff;            /* 速度前馈线性增益 (PWM / 每周期计数) */
    float vel_ff_dead;       /* 电机死区补偿 (PWM) */

    /*
     * ───────────────────── 参数详细说明 ─────────────────────
     *
     * 【位置环】(作用: 决定"多快到点/多准")
     *  pos_Kp : 位置误差 → 期望速度 的比例系数。
     *           取值: 建议 0.3~1.5，当前默认 0.6。
     *           作用: 误差越大给出的期望速度越大。
     *           调大: 启动/制动更快、到位更利索，但过大会冲过头/来回震荡；
     *           调小: 移动更柔和，但明显"磨蹭"、到位慢。
     *  pos_Ki : 位置环积分系数。
     *           取值: 建议 0（多数情况不需要），当前默认 0。
     *           作用: 消除长期位置静态误差。调大易累积超调造成终点来回摆。
     *  pos_Kd : 位置环微分系数（对误差变化率）。
     *           取值: 建议 0，当前默认 0。作用: 抑制到点前的超调/抖动，
     *           但会放大编码器噪声，一般不开启。
     *
     * 【速度环】(作用: 决定"实际速度是否听话")
     *  vel_Kp : 速度误差 → PWM 的比例系数。
     *           取值: 建议 0.1~1.0，当前默认 0.4。
     *           作用: 让实测速度快速追上期望速度。
     *           调大: 跟踪更紧，过大会 PWM 抖动、电机啸叫；调小: 响应迟钝。
     *  vel_Ki : 速度环积分系数。
     *           取值: 建议 0~0.2，当前默认 0.05。
     *           作用: 消除负载/坡度造成的速度稳态误差。过大易引起速度振荡。
     *  vel_Kd : 速度环微分系数。取值: 建议 0，当前默认 0（易放大编码器噪声）。
     *
     * 【限幅】
     *  max_vel   : 单周期内允许的最大期望速度，等于移动速度上限。
     *              取值: 建议 5~200（计数/周期，约对应 40~1500 mm/s），
     *              默认由 Chassis_Init 设 40，也可用 Chassis_SetMaxSpeed 改。
     *              调小→走得很慢；调大→起步冲、到点刹不住(惯性大)。
     *  max_output: PWM 输出上限，一般等于 PWM 满量程(工程为 99)，勿调大超过满量程。
     *
     * 【前馈】(作用: 让"起步/匀速"不依赖反馈硬顶，走得更直更顺)
     *  vel_ff      : 把期望速度直接换算成 PWM 的增益。
     *                取值: 建议 0.1~0.5，默认 0.3(在 Chassis.c 里设)。
     *                调大→起步更跟手，但过大会起步猛冲；调小→起步肉。
     *  vel_ff_dead : 低速时额外补偿电机/驱动死区(静摩擦)的 PWM 偏置。
     *                取值: 建议 0~30，默认 10(在 Chassis.c 里设)。
     *                过小→低速启不动/爬行抖动；过大→停车后仍缓慢蠕动。
     * ──────────────────────────────────────────────────────────
     */
} DualPID_Controller;

/**
 * @brief 单环位置式 PID (底盘转向闭环用: 输入角度误差, 输出转向修正量)
 *
 * ⚠️ 除 kp/ki/kd/imax 外均为算法内部状态，勿手动修改。
 * 实际初值在 Chassis.c 的 Chassis_Init() 中通过 pid_param_init(&s_steer,...) 设置。
 */
typedef struct
{
    /* ===== 可调参数 ===== */
    float kp, ki, kd;        /* P/I/D 系数, 详见下方说明 */
    float imax;              /* 积分累计上限, 防积分饱和 */

    /* ===== 内部状态 (勿手调) ===== */
    float integrator;        /* [内部] 误差累计(积分) */
    float last_error;        /* [内部] 上一次误差 */
    float out_p, out_i, out_d, out;  /* [内部] P/I/D 分量与总输出(调试用) */

    /*
     * ───────────────────── 参数详细说明 ─────────────────────
     *  输入 error = 剩余转向角度(度)，输出为转向修正量(计数/周期, 再叠加到四轮目标位置)。
     *  角度符号约定见 Chassis.h: 本工程正角度 = 逆时针(向左转)。
     *
     *  kp : 比例系数。剩余角度越大，修正越猛。
     *       取值: 建议 0.5~3.0，当前默认 1.5。
     *       调大→转得快、小角度也修正得动，但过大在目标附近来回抖；
     *       调小→转向慢/末段停不准(残留角度大)。
     *  ki : 积分系数。用于消除持续外力(如轮子打滑)造成的稳态角度偏差。
     *       取值: 建议 0，当前默认 0(本工程转向靠 P+D，不依赖积分)。
     *       调大→能消残余角，但容易转向过头再回摆。
     *  kd : 微分(阻尼)系数。转向接近目标时'刹车'，抑制过冲震荡。
     *       取值: 建议 0.5~3.0，当前默认 1.5。
     *       调大→停得干脆不甩尾，过大会转向发肉/末段修正力不足。
     *  imax: 积分累计上限(度)。防积分项无限增大导致饱和后猛冲。
     *        取值: 建议 200~1000，当前默认 500。一般不用改。
     * ────────────────────────────────────────────────────────
     */
} pid_param_t;

/* ---------------- 函数声明 ---------------- */

/**
 * @brief 初始化单个串级 PID 控制器
 */
void DualPID_Init(DualPID_Controller *pid,
                  float pos_Kp, float pos_Ki, float pos_Kd,
                  float vel_Kp, float vel_Ki, float vel_Kd,
                  float max_vel, float max_output,
                  float vel_ff, float vel_ff_dead);

/**
 * @brief 修改串级 PID 的最大目标速度 (速度上限)
 */
void DualPID_SetMaxVel(DualPID_Controller *pid, float max_vel);

/**
 * @brief 串级 PID 计算
 * @param target_pos  目标位置 (编码器计数)
 * @param current_pos 当前位置 (编码器计数)
 * @param current_vel 当前速度 (编码器计数/控制周期)
 * @param dt          控制周期 (秒)
 * @param vel_bias    速度环偏置 (计数/控制周期, 底盘航向保持注入)
 * @return PWM 输出
 */
float DualPID_Update(DualPID_Controller *pid,
                     float target_pos, float current_pos,
                     float current_vel, float dt, float vel_bias);

/**
 * @brief 初始化单环 PID 控制器
 */
void pid_param_init(pid_param_t *pid, float kp, float ki, float kd, float imax);

/**
 * @brief 单环位置式 PID 计算
 * @param error 误差 = 目标值 - 当前值
 * @return 控制量
 */
float PidLocCtrl(pid_param_t *pid, float error);

/**
 * @brief 复位串级 PID (清零积分与误差历史)
 */
void DualPID_Reset(DualPID_Controller *pid);

/**
 * @brief 复位单环 PID (清零积分与误差历史)
 */
void pid_param_reset(pid_param_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */
