#include "main.h"
#include "Chassis.h"
#include "pid.h"
#include "motor.h"
#include "ENCODER.h"
#include "elog.h"
#include <math.h>

/* =====================================================================
 * 车轮索引 (内部): 0=FL 1=FR 2=BL 3=BR
 * ===================================================================== */
enum { W_FL = 0, W_FR = 1, W_BL = 2, W_BR = 3, W_NUM = 4 };

/* 每轮电机号、电机极性、编码器极性 (来自 Chassis.h 的宏) */
static const uint8_t s_motor[W_NUM] = { CH_MOTOR_FL, CH_MOTOR_FR, CH_MOTOR_BL, CH_MOTOR_BR };
static const int8_t  s_motor_dir[W_NUM] = { CH_MOTOR_DIR_FL, CH_MOTOR_DIR_FR, CH_MOTOR_DIR_BL, CH_MOTOR_DIR_BR };
static const int8_t  s_enc_dir[W_NUM] = { CH_ENC_DIR_FL, CH_ENC_DIR_FR, CH_ENC_DIR_BL, CH_ENC_DIR_BR };

/* 运动学符号: 前进(所有轮 +), 左移(FL-,FR+,BL+,BR-) */
static const int8_t s_fwd_sign[W_NUM]    = { +1, +1, +1, +1 };
static const int8_t s_strafe_sign[W_NUM] = { -1, +1, +1, -1 };

/* =====================================================================
 * 内部状态
 * ===================================================================== */
static DualPID_Controller s_pid[W_NUM];      /* 四轮串级 PID */
static pid_param_t        s_steer;           /* 转向 PID */
static int32_t            s_target[W_NUM];   /* 目标位置 (编码器计数) */
static int32_t            s_pos[W_NUM];      /* 当前位置 (编码器计数) */
static int16_t            s_speed[W_NUM];    /* 当前速度 (计数/周期) */
static int16_t            s_last_pwm[W_NUM]; /* 最近一次输出 PWM (软启动斜坡基准 + 调试用) */

static const float *s_yaw = NULL;            /* 航向角数据源 (度) */
static bool  s_inited = false;
static bool  s_suspended = false;            /* 挂起标志(手动测试时置位) */
static bool  s_moving = false;               /* 平移进行中 */

static bool  s_turn_open = false;            /* 转向进行中 */
static float s_turn_remaining = 0.0f;        /* 剩余转向角度 */
static float s_turn_prev_yaw = 0.0f;        // 上一周期航向 (度, PD 阻尼用)
static uint16_t s_turn_stable = 0;          // 转向停车稳定计数
static float s_steer_i_peak = 0.0f;         /* 转向环 I 项峰值(本次转向内保持, 调试用) */
static uint16_t s_move_stable = 0;           /* 平移停车稳定计数 */
static int32_t  s_move_start_pos[W_NUM];     /* 本段平移的起点位置 (算"实走多少"用, 见 MOVE 日志) */
static uint16_t s_move_cycles = 0;           /* 本段平移已运行的控制周期数 (超时兜底用) */

static float s_heading_target = 0.0f;       /* 平移时的目标航向 (度) */
static float s_heading_prev_yaw = 0.0f;     /* 上一周期航向 (度, PD 阻尼用) */
static float s_heading_integral = 0.0f;     /* 航向积分 (度·秒, 抗饱和) */
static float s_heading_err_last  = 0.0f;    /* 最近一次航向误差 (度, 调试) */
static float s_heading_corr_last = 0.0f;    /* 最近一次航向修正 (计数/周期, 调试) */
static int16_t s_vel_bias_last[W_NUM] = { 0, 0, 0, 0 }; /* 最近一次速度偏置 (调试) */

/* ⭐ 航向保持的"位置目标微调"(转治横移走不直, 2026-09-27 新增)
 *   s_heading_trim 单位=编码器计数: 正 = 想让车头顺时针转 → FL/BL 目标 +trim,
 *   FR/BR 目标 -trim (与航向 PID 的 corr 正方向一致)。
 *   为什么不能只用速度偏置: 见 Chassis.h 的 CH_HEADING_TRIM_MAX 说明。 */
static int32_t s_heading_trim = 0;
static const int8_t s_hdg_rot_sign[W_NUM] = { +1, -1, +1, -1 };

/* 每轮 counts/mm 标定系数 (来自 Chassis.h, 补偿左右轮轮径/编码器差异) */
static const float s_wheel_scale[W_NUM] = { CH_WHEEL_SCALE_FL, CH_WHEEL_SCALE_FR, CH_WHEEL_SCALE_BL, CH_WHEEL_SCALE_BR };

/* 每轮 PWM 补偿系数 (来自 Chassis.h, 分前进/后退, 补偿电机增益差异) */
static const float s_pwm_fwd_scale[W_NUM] = { CH_PWM_SCALE_FWD_FL, CH_PWM_SCALE_FWD_FR, CH_PWM_SCALE_FWD_BL, CH_PWM_SCALE_FWD_BR };
static const float s_pwm_rev_scale[W_NUM] = { CH_PWM_SCALE_REV_FL, CH_PWM_SCALE_REV_FR, CH_PWM_SCALE_REV_BL, CH_PWM_SCALE_REV_BR };

/* =====================================================================
 * ⭐ 分段 PID 参数 (直行 / 平移 各一套)  —— 只改这里
 * ---------------------------------------------------------------------
 * 为什么分两套: 直行(前进/后退)与平移(左移/右移)时, 四轮负载/摩擦/
 * 陀螺仪漂移特性不同, 同一套 PID 无法两边都最优。
 * 生效规则 (见 add_move):
 *   纯前进 / 后退 / 斜行 → s_cfg_straight
 *   纯左移 / 右移        → s_cfg_strafe
 * ⚠️ 直行已调得很好 → 不要动 s_cfg_straight;
 *    平移走不直   → 只调 s_cfg_strafe (初始值与直行相同)。
 * 平移走不直 → 优先调 hd_*(航向保持): hd_Kp 调大纠偏更狠,
 *              hd_Kd 调大抑制摆动, hd_max 调大纠偏力度上限。
 * ===================================================================== */
typedef struct
{
    /* --- 四轮串级 PID (位置环 + 速度环) --- */
    float pos_Kp, pos_Ki, pos_Kd;   /* 位置环: 决定"多快到点/到点多准" */
    float vel_Kp, vel_Ki, vel_Kd;   /* 速度环: 决定"实际速度是否听话" */
    float vel_ff, vel_ff_dead;      /* 前馈增益 / 死区补偿 */
    /* --- 航向保持 PID (运动时用陀螺仪纠偏; 平移走不直优先调这组) --- */
    float hd_Kp, hd_Ki, hd_Kd;      /* 航向 P/I/D */
    float hd_max, hd_imax;          /* 修正量限幅 / 积分限幅 */
    float hd_dead;                  /* 航向误差死区(度) */
    float hd_trim;                  /* 修正量投入"位置目标微调"的比例 (0~1, 见 Chassis.h 的
                                     * CH_HEADING_TRIM_MAX 说明): 0=只用速度偏置(直行用),
                                     * 1=全部投到位置微调(平移用, 能真正修住横移漂移) */
} ChassisPidCfg_t;

/* 直行模式 (前进/后退/斜行) —— 当前实测很直, 保持不动
 * 2026-09-27 只改了两处(其余勿动):
 *   pos_Ki 0 → 0.25 : 位置环积分(只在接近目标时累积, 见 pid.c), 消除"轮子被静摩擦
 *                     卡住停在残差处"导致的四轮停车不同步/车姿偏移
 *   hd_Kp 4.0 → 1.8, hd_max 12 → 25 : 原 hd_Kp×hd_max 在误差≥3° 就饱和 → 航向保持
 *                     是开关式的, 满输出=左右差速±90mm/s, 造成行驶中来回抽动(晃)。
 *                     改后线性区扩到 ±14°, 修正是比例的而不是 bang-bang。 */
static const ChassisPidCfg_t s_cfg_straight = {
    .pos_Kp = 0.6f,  .pos_Ki = 0.25f,  .pos_Kd = 0.0f,
    .vel_Kp = 0.4f,  .vel_Ki = 0.05f,  .vel_Kd = 0.0f,
    .vel_ff = 0.3f,  .vel_ff_dead = 14.0f,
    .hd_Kp = 1.8f,   .hd_Ki = 1.0f,    .hd_Kd = 1.5f,
    .hd_max = 25.0f, .hd_imax = 50.0f, .hd_dead = 0.5f,
    .hd_trim = 0.0f   /* 直行实测很直 → 航向保持保持原样(只用速度偏置), 不动 */
};

/* 平移模式 (纯左移/右移) —— 横移漂移优先调这里
 * 2026-09-27: pos_Ki 0→0.25, hd_Kp 4.5→2.0, hd_max 12→25 (理由同 s_cfg_straight)
 * 2026-09-27(二修, 治"横移走不直"): 横移时辊子侧向刮地被地面拧着转,
 *   四轮位置环会把"速度环偏置"抵消掉 → 改成 hd_trim=1.0(把修正量投入位置
 *   目标微调) 才真能修住; 同时把增益降下来(机制变有效了, 不改会过冲)。 */
static const ChassisPidCfg_t s_cfg_strafe = {
    .pos_Kp = 0.6f,  .pos_Ki = 0.25f,  .pos_Kd = 0.0f,
    .vel_Kp = 0.4f,  .vel_Ki = 0.05f,  .vel_Kd = 0.0f,
    .vel_ff = 0.3f,  .vel_ff_dead = 12.0f,
    .hd_Kp = 1.4f,   .hd_Ki = 0.8f,    .hd_Kd = 1.0f,
    .hd_max = 12.0f, .hd_imax = 40.0f, .hd_dead = 0.3f,
    .hd_trim = 1.0f
};

static const ChassisPidCfg_t *s_cfg = &s_cfg_straight;  /* 当前生效的分段参数集 */
static float s_max_vel = 40.0f;   /* 当前速度上限(计数/周期), 由 Chassis_SetMaxSpeed 维护 */
#define CH_PID_MAX_OUTPUT   99.0f  /* PWM 输出上限 (= PWM_ARR 满量程), 一般不改 */

/* =====================================================================
 * 内部工具
 * ===================================================================== */
static int32_t iabs(int32_t v) { return v < 0 ? -v : v; }

//角度归一化
static float wrap_180(float a)
{
    while (a > 180.0f)  a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

static int32_t mm_to_counts(int32_t mm)
{
    float f = (float)mm * CH_COUNTS_PER_MM;             //CH_COUNTS_PER_MM表示每毫米对应的计数值
    return (int32_t)(f >= 0.0f ? f + 0.5f : f - 0.5f);  //四舍五入取整
}

/* 本轮"生效目标位置" = 累计目标 + 航向微调
 * (到位判定 / MOVE 日志的 e: / 位置环都用这个, 否则 trim 会被当成"残差") */
static int32_t eff_tgt(int i)
{
    return s_target[i] + (int32_t)s_hdg_rot_sign[i] * s_heading_trim;
}

/* 读取四路编码器增量, 换算为"向前为正"的位置与速度 */
static void read_encoders(void)
{
    for (int i = 0; i < W_NUM; i++)
    {
        int16_t raw = Encoder_GetDelta(s_motor[i]);
        int16_t d = (int16_t)(raw * s_enc_dir[i]);
        s_speed[i] = d;
        s_pos[i] += d;
    }
}

/* 输出四路 PWM (换算为"电机正方向", 并做阶梯递增软启动) */
static void output_pwm(const int16_t pwm[W_NUM])
{
    for (int i = 0; i < W_NUM; i++)
    {
        float scale = (pwm[i] >= 0) ? s_pwm_fwd_scale[i] : s_pwm_rev_scale[i];
        int16_t want = (int16_t)(pwm[i] * s_motor_dir[i] * scale);  /* PID 想要最终输出的 PWM */
        int16_t last = s_last_pwm[i];
        int16_t out  = want;

        /* ---- 阶梯递增 (软启动) ----
         * 只在"输出幅值变大"(起步/加速)时限速: 每控制周期最多增加 CH_PWM_RAMP_STEP。
         * 减速(幅值变小)/反向不受限 → 刹车与换向仍然干脆, 只消除起步猛冲。
         * CH_PWM_RAMP_STEP <= 0 时关闭软启动(一步到位)。 */
#if CH_PWM_RAMP_STEP > 0
        if (iabs(want) > iabs(last))
        {
            if (want > last) out = (int16_t)(last + CH_PWM_RAMP_STEP);
            else             out = (int16_t)(last - CH_PWM_RAMP_STEP);
            /* 最后一步不要越过目标值 */
            if ((want > last && out > want) || (want < last && out < want)) out = want;
        }
#endif
        s_last_pwm[i] = out;
        Set_PWM(s_motor[i], out);
    }
}

//电机停止
static void stop_motors(void)
{
    for (int i = 0; i < W_NUM; i++)
    {
        s_last_pwm[i] = 0;   /* 同步软启动斜坡基准, 否则下次起步会从旧值跳变(仍会猛冲) */
        Set_PWM(s_motor[i], 0);
    }
}

/* ⭐ 到位/停止时"冻结"(2026-09-27 新增)
 * 做什么: ① 四轮目标位置 = 当前位置(丢掉残差) ② 清 PID 积分与航向微调
 *         ③ 立刻输出 PWM=0
 * 为什么: 原来本段到位后只是把 s_moving 清 0, 位置环还在拿着"没走完的残差"
 *         继续驱动电机(残差最多 CH_POS_THRESHOLD_COUNT), 而状态机已经切走、
 *         甚至已进入视觉/等待状态 → 车会自己再拱一下(四轮残差不一致时还会
 *         被拧一下), 用户看到的就是"停下了轮子又动一下/没停稳"。
 * 代价: 本段残差(≤CH_POS_THRESHOLD_COUNT≈3mm)不再带入下一段。若发现"越走
 *       越偏"说明残差是同向的 → 调小 CH_POS_THRESHOLD_COUNT 或加大 pos_Ki。 */
static void chassis_freeze(void)
{
    for (int i = 0; i < W_NUM; i++)
    {
        s_target[i] = s_pos[i];
        s_move_start_pos[i] = s_pos[i];
        DualPID_Reset(&s_pid[i]);
    }
    s_heading_trim = 0;
    s_heading_integral = 0.0f;
    s_heading_corr_last = 0.0f;
    for (int i = 0; i < W_NUM; i++)
    {
        s_vel_bias_last[i] = 0;
    }
    stop_motors();
}

/* =====================================================================
 * 初始化
 * ===================================================================== */
void Chassis_SetYawSource(const float *yaw_addr)
{
    s_yaw = yaw_addr;
}

void Chassis_SetMaxSpeed(float mm_per_sec)
{
    /* 把“mm/s”换算成“编码器计数/控制周期”, 换算公式:
     *   counts/周期 = mm/s × (计数/mm) × (控制周期秒数)
     * 参数: mm_per_sec —— 期望最大平移速度(毫米/秒)。
     *   取值范围/推荐: 40~1500 mm/s, 当前主循环里用的是 200(见 main.c)。
     *   影响: 越大移动越快, 但起步越冲、到点刹停距离越长、越容易甩尾;
     *         越小走得越慢越稳。实际超过约 1500 电机会跟不动。
     * 下面两个钳位是软件硬边界(一般不改): 5 计数/周期 ≈ 38 mm/s,
     * 200 计数/周期 ≈ 1510 mm/s。传参超出会被自动夹回该范围。 */
    float counts_per_cycle = mm_per_sec * CH_COUNTS_PER_MM * CH_CTRL_PERIOD_MS / 1000.0f;
    if (counts_per_cycle < 5.0f)   counts_per_cycle = 5.0f;
    if (counts_per_cycle > 200.0f) counts_per_cycle = 200.0f;

    s_max_vel = counts_per_cycle;   /* 记录, 供切换分段参数时重新套用 */
    for (int i = 0; i < W_NUM; i++)
    {
        DualPID_SetMaxVel(&s_pid[i], counts_per_cycle);
    }
}

/* 应用一套分段参数到四轮 PID (同时复位积分与航向积分) */
static void apply_cfg(const ChassisPidCfg_t *cfg)
{
    s_cfg = cfg;
    for (int i = 0; i < W_NUM; i++)
    {
        DualPID_Init(&s_pid[i], cfg->pos_Kp, cfg->pos_Ki, cfg->pos_Kd,
                     cfg->vel_Kp, cfg->vel_Ki, cfg->vel_Kd,
                     s_max_vel, CH_PID_MAX_OUTPUT,
                     cfg->vel_ff, cfg->vel_ff_dead);
    }
    s_heading_integral = 0.0f;
}


/* =====================================================================
 * ⭐ 底盘运动手感调参区 (非专业人员请只改这一函数里的初值)
 * ---------------------------------------------------------------------
 * 下面 const 是四轮串级 PID 的默认参数(对应 pid.h 里的 DualPID_Controller)。
 * 调参原则: 一次只改一个量, 烧录实测后再改下一个, 方便判断影响。
 * 改完记得: EIDE 重载工程 → 重新构建 → 烧录(改宏/常量不会自动热更新)。
 *
 * 现象速查(仅供对照):
 *   - 起步“冲一下/猛”:      调小 vel_ff (0.3→0.25)
 *   - 到点前小幅震荡/来回摆:  调小 vel_ff, 或调大 pos_Kp 的同时调小 vel_Kp
 *   - 觉得走得慢/跟不上:     调大 vel_ff (→0.35) 或调大 max_vel / Chassis_SetMaxSpeed
 *   - 到点刹不住、过头回摆:  调小 max_vel、调小 vel_ff、或调大 pos_Kd
 *   - 直线走斜/横移漂移:     先调 Chassis.h 的 CH_HEADING_* 航向保持参数
 * ===================================================================== */
void Chassis_Init(void)
{
    /* 速度上限初值 40 计数/周期(≈300mm/s), 之后 main.c 用 Chassis_SetMaxSpeed() 覆盖。
     * 四轮 PID 参数不在这里写死了, 改为按"直行/平移"分段取用,
     * 见文件上方 ⭐ 分段 PID 参数 区 (ChassisPidCfg_t / s_cfg_straight / s_cfg_strafe)。 */
    s_max_vel = 40.0f;
    apply_cfg(&s_cfg_straight);   /* 上电默认: 直行参数 */

    for (int i = 0; i < W_NUM; i++)
    {
        s_target[i] = 0;
        s_pos[i] = 0;
        s_speed[i] = 0;
        s_move_start_pos[i] = 0;
    }

    /* 转向 PID (对应 pid.h 的 pid_param_t): 输入剩余转向角度(度), 输出转向修正量。
     * 参数: kp=1.5(比例)  ki=0(积分)  kd=1.5(阻尼)  imax=500(积分限幅)。
     * 调参速查:
     *   - 转向转不到位/末段残留角度大: 调大 kp(1.5→2.0) 或调小 Chassis.h 的
     *     CH_ANGLE_ERR_THRESHOLD
     *   - 到目标附近来回震荡/甩尾过冲: 调大 kd(1.5→2.5) 阻尼, 或调小 kp
     *   - 大角度转弯转太慢: 调大 Chassis.h 的 CH_MAX_TURN_ADJUST(30→40)
     * 注: kp 1.0→1.5 是为加强小角度修正, 需配合更小的到位阈值(1.5°)使用。 */
    pid_param_init(&s_steer, 1.5f, 0.006f, 1.5f, 500.0f);

    stop_motors();
    read_encoders();
    for (int i = 0; i < W_NUM; i++)
    {
        s_target[i] = s_pos[i];
    }

    s_moving = false;
    s_turn_open = false;
    s_move_stable = 0;
    s_move_cycles = 0;
    s_heading_integral = 0.0f;
    s_heading_trim = 0;
    s_heading_target = 0.0f;   /* 默认基准 0°; 任务层也可用 Chassis_SetHeadingRef() 改 */
    s_heading_prev_yaw = (s_yaw != NULL) ? *s_yaw : 0.0f;
    s_inited = true;
}

void Chassis_Suspend(void)
{
    s_suspended = true;
    s_moving = false;
    s_turn_open = false;
    stop_motors();
}

//底盘复位
void Chassis_Resume(void)
{
    /* 重新同步编码器软件状态并清零位置目标,
     * 避免手动测试期间 Encoder_ResetCount 造成的计数跳变 */
    for (int i = 0; i < W_NUM; i++)
    {
        Encoder_Start(s_motor[i]);
        s_pos[i] = 0;
        s_speed[i] = 0;
        s_target[i] = 0;
        s_move_start_pos[i] = 0;
    }
    s_moving = false;
    s_turn_open = false;
    s_turn_stable = 0;
    s_move_stable = 0;
    s_move_cycles = 0;
    s_heading_integral = 0.0f;
    s_heading_trim = 0;
    s_heading_prev_yaw = (s_yaw != NULL) ? *s_yaw : 0.0f;
    s_suspended = false;
}

/* =====================================================================
 * 编码器静态标定 (人工推车) —— 详细说明见 Chassis.h 的 Chassis_CalibStart
 * ---------------------------------------------------------------------
 * 测什么: 人工推车 D mm, 看四轮各自报了多少计数 →
 *         CH_WHEEL_SCALE_x = (该轮计数 / D) / CH_COUNTS_PER_MM
 * ⚠️ 为什么不能看正常跑车的日志: 位置闭环会把每轮都控到各自的计数目标,
 *    四轮打印的计数当然差不多(MOVE 日志的 d: 也是), 根本看不出真实差异。
 * ⚠️ 只在两种情况下才用静态标定:
 *    ① 同方向推两次 / 正反各推一次, 结果稳定且某轮明显偏离 1.000;
 *    ② 若"有时 M1 多、有时 M2 多"或正反不一致 → 不是 counts/mm 问题
 *       (是打滑/静摩擦/负载转移), 静态标定修不了。
 * ===================================================================== */
static bool    s_calib_active = false;
static int32_t s_calib_accum[W_NUM];

void Chassis_CalibStart(void)
{
    Chassis_Suspend();   /* 停闭环: 电机不输出, 也不会有人跟本函数抢 Encoder_GetDelta */
    for (int i = 0; i < W_NUM; i++)
    {
        s_calib_accum[i] = 0;
    }
    s_calib_active = true;
}

void Chassis_CalibPoll(void)
{
    if (!s_calib_active || !s_suspended)
    {
        return;   /* 未开始标定, 或底盘没挂起(闭环在跑) → 不能抢编码器增量 */
    }
    for (int i = 0; i < W_NUM; i++)
    {
        /* Encoder_GetDelta 内部已处理 16/32 位回绕; 再乘 enc_dir 变成"前进为正" */
        s_calib_accum[i] += (int32_t)Encoder_GetDelta(s_motor[i]) * (int32_t)s_enc_dir[i];
    }
}

int32_t Chassis_CalibAccum(int idx)
{
    if (idx < 0 || idx >= W_NUM)
    {
        return 0;
    }
    return s_calib_accum[idx];
}

void Chassis_CalibReport(float distance_mm)
{
    if (distance_mm <= 1.0f)
    {
        return;
    }

    /* 建议系数 = 该轮实测 counts/mm ÷ 标称 counts/mm, 乘 1000 打成整数
     *   CH_WHEEL_SCALE_x = (累计计数 / 推动距离) / CH_COUNTS_PER_MM
     * 取绝对值: 正推反推都对(推反方向时计数是负的, 但 k 的绝对值不变) */
    long s[W_NUM];
    for (int i = 0; i < W_NUM; i++)
    {
        float f = (fabsf((float)s_calib_accum[i]) / distance_mm) / CH_COUNTS_PER_MM * 1000.0f;
        s[i] = (long)(f + 0.5f);
    }

    elog_i("CALIB", "push=%.1fmm counts: FL=%ld FR=%ld BL=%ld BR=%ld (前进为正)",
           (double)distance_mm,
           (long)s_calib_accum[W_FL], (long)s_calib_accum[W_FR],
           (long)s_calib_accum[W_BL], (long)s_calib_accum[W_BR]);
    elog_i("CALIB", "CH_WHEEL_SCALE 建议: FL=%ld FR=%ld BL=%ld BR=%ld (除1000填宏, 1000=不改)",
           s[W_FL], s[W_FR], s[W_BL], s[W_BR]);
}

void Chassis_CalibStop(void)
{
    s_calib_active = false;
    Chassis_Resume();   /* 重新同步编码器 + 清零位置目标, 恢复正常闭环 */
}

/* =====================================================================
 * 周期调用
 * ===================================================================== */
void Chassis_Tick(void)
{
    static uint8_t div = 0;
    if (!s_inited || s_suspended)
    {
        return;
    }
    if (++div >= CH_CTRL_PERIOD_MS)
    {
        div = 0;
        Chassis_Update_Control();
    }
}

/* ⭐ 每段平移结束时打印一行(只在结束时打, 不占用 500ms 周期日志):
 *   d:FL/FR/BL/BR = 本段四轮实际走的编码器计数;
 *   e:FL/FR/BL/BR = 结束瞬间四轮残差(目标-实际, 计数);
 *   yaw           = 结束瞬间航向(度);
 *   行首 "TO"    = 超时兜底强制结束(说明该段被卡住, 没走到位)。
 * 怎么看:
 *   - 四轮 d 应该基本相等(差<1%)。某轮 d 长期偏小/偏大 → 该轮电机/轮径/编码器
 *     需要标定(CH_WHEEL_SCALE_x / CH_PWM_SCALE_x), 或者该轮在打滑。
 *   - 四轮 e 应该都 < CH_POS_THRESHOLD_COUNT。若某轮 e 很大 → 它被静摩擦
 *     卡住了(调 pos_Ki); 若四轮 e 符号/大小不一致 → 到位判定太松(已收紧)。
 *   - ⚠️ d: 四轮【不再】应该完全相等: 横移时航向保持会故意让左右两侧多走/
 *     少走一点(位置目标微调 s_heading_trim)来把车头转正, 差值 = 2×其绝对值。
 *     所以看 d 要看"同侧两轮之和"与另一侧之和的差是否与偏差角匹配, 不要以为
 *     差几拾计数就是轮子不齐。
 *   - 各段 yaw 应该都接近 0°(航向基准)。若每段都比上一段偏一点 → 航向基准或
 *     转向到位阈值有问题(已用 Chassis_SetHeadingRef + CH_ANGLE_ERR_THRESHOLD 修)。
 *   - 出现 TO → 该段 10s 没走完, 看 d/e 定位是哪个轮卡住。 */
static void move_finish_log(const char *why)
{
    elog_i("MOVE", "%s d:FL=%ld FR=%ld BL=%ld BR=%ld e:FL=%ld FR=%ld BL=%ld BR=%ld yaw=%.2f",
           (why != NULL) ? why : "",
           (long)(s_pos[W_FL] - s_move_start_pos[W_FL]), (long)(s_pos[W_FR] - s_move_start_pos[W_FR]),
           (long)(s_pos[W_BL] - s_move_start_pos[W_BL]), (long)(s_pos[W_BR] - s_move_start_pos[W_BR]),
           (long)(eff_tgt(W_FL) - s_pos[W_FL]), (long)(eff_tgt(W_FR) - s_pos[W_FR]),
           (long)(eff_tgt(W_BL) - s_pos[W_BL]), (long)(eff_tgt(W_BR) - s_pos[W_BR]),
           (double)((s_yaw != NULL) ? *s_yaw : 0.0f));
}

void Chassis_Update_Control(void)
{
    if (!s_inited || s_suspended)
    {
        return;
    }

    read_encoders();

    float dt = CH_CTRL_PERIOD_MS / 1000.0f;

    /* ---- 0. 平移时的航向保持 (陀螺仪 PID, 输出速度环偏置) ---- */
    int16_t vel_bias[W_NUM] = { 0, 0, 0, 0 };
    if (s_moving && s_yaw != NULL)
    {
        float yaw = *s_yaw;
        float yaw_err = wrap_180(yaw - s_heading_target);
        float dyaw = wrap_180(yaw - s_heading_prev_yaw);
        s_heading_prev_yaw = yaw;

        /* 航向误差死区: 误差足够小时不再修正, 避免停车前微调甩尾 */
        const ChassisPidCfg_t *cfg = s_cfg;   /* 分段参数: 直行/平移各一套 */
        float corr = 0.0f;
        if (fabsf(yaw_err) >= cfg->hd_dead)
        {
            /* 积分 (带抗饱和), 消除持续漂移下的稳态航向误差 */
            s_heading_integral += yaw_err * dt;
            if (s_heading_integral >  cfg->hd_imax) s_heading_integral =  cfg->hd_imax;
            if (s_heading_integral < -cfg->hd_imax) s_heading_integral = -cfg->hd_imax;

            corr = cfg->hd_Kp * yaw_err + cfg->hd_Ki * s_heading_integral + cfg->hd_Kd * dyaw;
            if (corr >  cfg->hd_max) corr =  cfg->hd_max;
            if (corr < -cfg->hd_max) corr = -cfg->hd_max;
        }

        /* ⭐ 把修正量按 hd_trim 比例"积分"进位置目标微调 (平移的真正纠偏手段)。
         * 单位与 corr 相同(=计数/周期): 每周期让 FL/BL 目标多走 inc、FR/BR 少走
         * inc → 左右两侧轮子走的距离不同 → 车头真的转回来。
         * 为什么速度偏置不够: 位置环看到某轮落后就会自动加大输出把偏置抵消掉,
         * 所以只靠 vel_bias 修不动横移时的"辊子侧向刮地被拧着转"(实测 vb 已到
         * ±10(≈±75mm/s 差速) 车头仍漂 1~3°/s)。详见 Chassis.h 的同名宏说明。
         * 速率限幅 CH_HEADING_TRIM_STEP 防过冲, 绝对限幅 CH_HEADING_TRIM_MAX 防跑飞。
         * ⚠️ 只在"还没进入停稳计数"时更新: 一旦开始停稳计数(车已基本停下)就冻结 trim,
         *    否则微调量持续增长会让轮子一直以 2~6 计数/周期蠕动, 永远判不到"停稳"
         *    (表现为\"停下时还在慢慢转\")。末段剩的那点角度交给状态机后面的航向校正。 */
        if (cfg->hd_trim > 0.0f && s_move_stable == 0)
        {
            int32_t inc = (int32_t)(corr * cfg->hd_trim);   /* 正 = 顺时针方向差速 */
            if (inc >  (int32_t)CH_HEADING_TRIM_STEP) inc =  (int32_t)CH_HEADING_TRIM_STEP;
            if (inc < -(int32_t)CH_HEADING_TRIM_STEP) inc = -(int32_t)CH_HEADING_TRIM_STEP;
            s_heading_trim += inc;
            if (s_heading_trim >  (int32_t)CH_HEADING_TRIM_MAX) s_heading_trim =  (int32_t)CH_HEADING_TRIM_MAX;
            if (s_heading_trim < -(int32_t)CH_HEADING_TRIM_MAX) s_heading_trim = -(int32_t)CH_HEADING_TRIM_MAX;
        }

        /* 正 corr = 顺时针修正: 左轮 +, 右轮 - (速度偏置, 注入速度环) */
        int32_t c = (int32_t)(corr >= 0.0f ? corr + 0.5f : corr - 0.5f);
        vel_bias[W_FL] = (int16_t)c;
        vel_bias[W_FR] = (int16_t)(-c);
        vel_bias[W_BL] = (int16_t)c;
        vel_bias[W_BR] = (int16_t)(-c);

        /* 记录调试量 (供 Chassis_HeadingDebugLog 主循环打印) */
        s_heading_err_last  = yaw_err;
        s_heading_corr_last = corr;
        s_vel_bias_last[W_FL] = vel_bias[W_FL];
        s_vel_bias_last[W_FR] = vel_bias[W_FR];
        s_vel_bias_last[W_BL] = vel_bias[W_BL];
        s_vel_bias_last[W_BR] = vel_bias[W_BR];
    }

    //s_yaw != NULL 时, 下面的转向闭环会用到 s_yaw, 所以要在航向保持之后再读一次；s_yaw表示陀螺仪数据源, 由 Chassis_SetYawSource() 注入, 指向 jy61p->var.yaw 的地址, 单位: 度
    /* ---- 1. 转向闭环 (陀螺仪) ---- */
    if (s_turn_open)
    {
        float yaw = s_yaw ? *s_yaw : 0.0f;
        float dyaw = wrap_180(yaw - s_turn_prev_yaw);           //得到相对变化的角度
        s_turn_prev_yaw = yaw;
        s_turn_remaining -= dyaw;

        /* 到位阈值外: 正常位置 PID; 死区内: 角速度阻尼刹车(防切状态时"多转") */
        float adj = 0.0f;
        if (fabsf(s_turn_remaining) >= CH_ANGLE_ERR_THRESHOLD)//ch-angle_err_threshold: 转向闭环的死区阈值, 单位: 度, 默认 1.5°; 误差小于该值时不再修正, 避免在目标角度附近来回震荡
        {
            adj = PidLocCtrl(&s_steer, s_turn_remaining) * CH_TURN_SIGN;//s_steer转向PID
            if (adj > CH_MAX_TURN_ADJUST)  adj = CH_MAX_TURN_ADJUST;    //转向限幅
            if (adj < -CH_MAX_TURN_ADJUST) adj = -CH_MAX_TURN_ADJUST;

            /* 记录本次转向中 |I 项| 的峰值 (调试): I 项只在转向且剩余角≥死区时才有值,
             * OLED 100ms 刷新容易错过, 故用峰值保持方便事后观察 */
            if (fabsf(s_steer.out_i) > fabsf(s_steer_i_peak))
            {
                s_steer_i_peak = s_steer.out_i;
            }
        }
        else if (CH_TURN_BRAKE_KD > 0.0f)
        {
            /* ---- 死区内角速度阻尼刹车 ----
             * 残余角已很小, 不用位置 PID(否则会在目标附近来回蹭); 但车可能仍带角速度,
             * 这里给一个与"本周期实测转角 dyaw"反向的修正, 主动把惯性刹住。
             * dyaw 与 adj 同符号约定(正=逆时针), 故乘 CH_TURN_SIGN 与上面保持一致。
             * 若无此项 → adj=0 → 放任滑行, 而状态已切走 → 表现为"多转"。*/
            adj = -CH_TURN_BRAKE_KD * dyaw * CH_TURN_SIGN;
            if (adj > CH_MAX_TURN_ADJUST)  adj = CH_MAX_TURN_ADJUST;
            if (adj < -CH_MAX_TURN_ADJUST) adj = -CH_MAX_TURN_ADJUST;
        }

        int32_t a = (int32_t)adj;

        /* 逆时针(向左转): 左轮后退, 右轮前进 */
        s_target[W_FL] -= a;
        s_target[W_FR] += a;
        s_target[W_BL] -= a;
        s_target[W_BR] += a;

        if (fabsf(s_turn_remaining) < CH_ANGLE_ERR_THRESHOLD &&
            fabsf(dyaw) < CH_TURN_RATE_THRESHOLD)
        {
            if (++s_turn_stable >= CH_TURN_STABLE_COUNT)
            {
                s_turn_open = false;
                s_turn_stable = 0;
                Chassis_Stop();
            }
        }
        else
        {
            s_turn_stable = 0;
        }
    }
    /* ---- 2. 平移到位判断 (位置 + 速度都达标, 连续多周期才算停稳) ---- */
    else if (s_moving)
    {
        bool settled = true;
        for (int i = 0; i < W_NUM; i++)
        {
            if (iabs(eff_tgt(i) - s_pos[i]) >= CH_POS_THRESHOLD_COUNT ||
                iabs(s_speed[i]) >= CH_STOP_SPEED_THRESHOLD)
            {
                settled = false;
                break;
            }
        }
        if (settled)
        {
            if (++s_move_stable >= CH_STOP_STABLE_COUNT)
            {
                s_move_stable = 0;
                move_finish_log("");   /* 先打日志: 要的是"到位瞬间"的 d/e */
                s_moving = false;
                chassis_freeze();       /* ⭐ 冻结: 目标=当前位置 + 输出 0,
                                        * 否则状态机切走后位置环还会去找那点残差,
                                        * 四轮残差不等时就把车拧一下(轮子又动一下) */
            }
        }
        else
        {
            s_move_stable = 0;
        }

        /* ⭐ 安全兜底: 单段平移超时就强制结束并打 MOVE 行(带 TO)。
         * 阈值收紧后万一某轮被卡住, 状态机不会永远停在原地。 */
        if (s_moving && ++s_move_cycles >= (uint16_t)(CH_MOVE_TIMEOUT_MS / CH_CTRL_PERIOD_MS))
        {
            move_finish_log("TO");
            s_moving = false;
            s_move_stable = 0;
            chassis_freeze();       /* 超时兜底也要冻结: 残留可能很大(e: 几十~上百),
                                     * 不冻结车会自己猛地拱一下 */
        }
    }

    /* ---- 3. 串级 PID 输出 (注入航向保持速度偏置) ---- */
    int16_t pwm[W_NUM];
    for (int i = 0; i < W_NUM; i++)
    {
        pwm[i] = (int16_t)DualPID_Update(&s_pid[i],
                                         (float)eff_tgt(i), (float)s_pos[i],
                                         (float)s_speed[i], dt,
                                         (float)vel_bias[i]);
    }
    //s_target是由转向环PID作用得到的，vel_bias是由航向保持PID作用得到的，
    //s_pos和s_speed是由编码器得到的，dt是控制周期时间，DualPID_Update()函数会根据这些参数计算出每个轮子的PWM输出值
    output_pwm(pwm);
}

/* =====================================================================
 * 平移指令
 * 参数：fwd_mm —— 前进距离(毫米, 负数表示后退)
 *       strafe_mm —— 左移距离(毫米, 负数表示右移)
 * 直接让车走多少毫米
 * ===================================================================== */
static void add_move(int32_t fwd_mm, int32_t strafe_mm)
{
    int32_t fwd_cnt = mm_to_counts(fwd_mm);
    int32_t strafe_cnt = mm_to_counts(strafe_mm) * CH_STRAFE_SIGN;          //CH_STRAFE_SIGN: 左移为正, 右移为负, 见 Chassis.h

    /* ---- 分段 PID: 纯左移/右移用"平移参数", 其余(前进/后退/斜行)用"直行参数" ---- */
    const ChassisPidCfg_t *cfg = (fwd_mm == 0 && strafe_mm != 0) ? &s_cfg_strafe : &s_cfg_straight;
    if (cfg != s_cfg)
    {
        apply_cfg(cfg);   /* 切换参数集(内部已复位积分) */
    }
    else
    {
        /* 同一套参数: 每次新动作清零 PID 积分/历史, 避免上一步的积分记忆导致起步不同步 */
        for (int i = 0; i < W_NUM; i++)
        {
            DualPID_Reset(&s_pid[i]);
        }
    }

    /* 按每轮 counts/mm 标定系数缩放目标, 补偿左右轮轮径/编码器差异 */
    for (int i = 0; i < W_NUM; i++)
    {
        float delta = (float)(s_fwd_sign[i] * fwd_cnt + s_strafe_sign[i] * strafe_cnt) * s_wheel_scale[i];
        s_target[i] += (int32_t)(delta >= 0.0f ? delta + 0.5f : delta - 0.5f);
        s_move_start_pos[i] = s_pos[i];   /* 记本段起点, 结束时算"实走多少"(见 MOVE 日志) */
    }

    /* ⚠️ 2026-09-27: 这里【不再】重设航向基准 s_heading_target(原来 = *s_yaw)。
     * 原来每段都重设 → 上一段航向校正剩下的 ~1.5° 误差被当成"正确朝向"继承下去,
     * 误差只累积不回收(实测 yaw 一路漂到 ±5°, 直接造成"停不回原来的朝向")。
     * 现在基准由任务层一次性设定, 见 Chassis_SetHeadingRef() / Mission_Init()。 */
    if (s_yaw != NULL)
    {
        s_heading_prev_yaw = *s_yaw;      /* 只同步"上一周期角度", 防止首个周期 dyaw 突变 */
    }

    s_heading_integral = 0.0f;
    s_turn_open = false;
    s_move_stable = 0;
    s_move_cycles = 0;
    s_moving = true;
}

void Chassis_Move_Forward(int32_t distance_mm)
{
    add_move(distance_mm, 0);
}

void Chassis_Move_Backward(int32_t distance_mm)
{
    add_move(-distance_mm, 0);
}

void Chassis_Move_Left(int32_t distance_mm)
{
    add_move(0, distance_mm);
}

void Chassis_Move_Right(int32_t distance_mm)
{
    add_move(0, -distance_mm);
}

void Chassis_Move_Diagonal(int32_t fwd_mm, int32_t strafe_mm)
{
    add_move(fwd_mm, strafe_mm);
}

/* =====================================================================
 * 转向指令
 * ===================================================================== */
void Chassis_Rotate(float delta_deg)
{
    if (s_yaw == NULL)
    {
        return;   /* 未注入航向角, 不支持转向 */
    }
    for (int i = 0; i < W_NUM; i++)
    {
        DualPID_Reset(&s_pid[i]);
    }
    pid_param_reset(&s_steer);
    s_steer_i_peak = 0.0f;   /* 清 I 项峰值, 本次转向重新记录 */

    s_turn_remaining = delta_deg;
    s_turn_prev_yaw = *s_yaw;
    s_turn_stable = 0;
    s_moving = false;
    s_turn_open = true;
}

void Chassis_Rotate_To(float target_deg)
{
    if (s_yaw == NULL)
    {
        return;
    }
    float delta = wrap_180(target_deg - *s_yaw);
    Chassis_Rotate(delta);
}

/* =====================================================================
 * 停止 / 状态
 * ===================================================================== */
void Chassis_Stop(void)
{
    s_moving = false;
    s_turn_open = false;
    s_turn_stable = 0;
    s_move_stable = 0;
    s_move_cycles = 0;
    s_heading_integral = 0.0f;
    /* ⭐ 2026-09-27 二改: 这里恢复"真正停住"——冻结(目标=当前位置)+输出 0。
     * 上一版为了不丢累积残差, 这里只清标志、不管电机 → 结果"停下"以后位置环
     * 还在驱动电机去找残差, 用户看到"停下了轮子又动一下/没停稳"; 而残差本身
     * 已被 CH_POS_THRESHOLD_COUNT 收到 20 计数(≈3mm), 丢掉代价很小。
     * 需要保留累积量时(重新开始/示教)用 Chassis_SyncTarget()。 */
    chassis_freeze();
}

/* 把四轮目标位置对齐到当前位置(丢弃累积位置误差) —— 只在"重新开始/复位"时用 */
void Chassis_SyncTarget(void)
{
    for (int i = 0; i < W_NUM; i++)
    {
        s_target[i] = s_pos[i];
        s_move_start_pos[i] = s_pos[i];
    }
    s_heading_trim = 0;      /* 航向微调也清零(否则生效目标 = 目标+微调 会与当前位置差一截) */
    s_move_stable = 0;
    s_move_cycles = 0;
}

/* 设置平移时的航向基准(见 Chassis.h 的说明) */
void Chassis_SetHeadingRef(float deg)
{
    s_heading_target = deg;
    if (s_yaw != NULL)
    {
        s_heading_prev_yaw = *s_yaw;   /* 同步上一周期角度, 避免下个周期 dyaw 突变 */
    }
    s_heading_integral = 0.0f;
}

bool Chassis_Task_Is_Complete(void)
{
    return (!s_turn_open && !s_moving);
}

/* =====================================================================
 * 演示: 前进300 -> 左移300 -> 右移300 -> 旋转180 -> 后退300 (循环)
 * 非阻塞, 放在主循环里调用; KEY1 触发
 * ===================================================================== */
static uint8_t  s_demo_step = 0;
static uint32_t s_demo_last_ms = 0;

void Chassis_Demo_Reset(void)
{
    s_demo_step = 0;
    s_demo_last_ms = 0;
    Chassis_SyncTarget();   /* 重新开始: 丢弃累积残差 */
    Chassis_Stop();
}

void Chassis_Demo(void)
{
    uint32_t now = HAL_GetTick();

    if (!Chassis_Task_Is_Complete())
    {
        return;
    }
    if (now - s_demo_last_ms < 1000)
    {
        return;
    }
    s_demo_last_ms = now;

    switch (s_demo_step)
    {
    case 0: Chassis_Move_Forward(300);  break;
    case 1: Chassis_Move_Left(300);     break;
    case 2: Chassis_Move_Right(300);    break;
    case 3: Chassis_Rotate(180.0f);     break;
    case 4: Chassis_Move_Backward(300); break;
    default: s_demo_step = 0; break;
    }
    s_demo_step = (s_demo_step + 1) % 5;
}

void Chassis_DebugLog(void)
{
    elog_i("CHS", "step=%d move=%d turn=%d rem=%.1f",
           s_demo_step, (int)s_moving, (int)s_turn_open, (double)s_turn_remaining);
    elog_i("POS", "FL=%ld FR=%ld BL=%ld BR=%ld",
           (long)s_pos[W_FL], (long)s_pos[W_FR], (long)s_pos[W_BL], (long)s_pos[W_BR]);
    elog_i("TGT", "FL=%ld FR=%ld BL=%ld BR=%ld",
           (long)s_target[W_FL], (long)s_target[W_FR], (long)s_target[W_BL], (long)s_target[W_BR]);
    elog_i("PWM", "FL=%d FR=%d BL=%d BR=%d",
           (int)s_last_pwm[W_FL], (int)s_last_pwm[W_FR], (int)s_last_pwm[W_BL], (int)s_last_pwm[W_BR]);
}

void Chassis_HeadingDebugLog(void)
{
    elog_i("HDG", "err=%.1f int=%.1f corr=%.1f trim=%ld vb=%d,%d,%d,%d",
           (double)s_heading_err_last,
           (double)s_heading_integral,
           (double)s_heading_corr_last,
           (long)s_heading_trim,
           (int)s_vel_bias_last[W_FL], (int)s_vel_bias_last[W_FR],
           (int)s_vel_bias_last[W_BL], (int)s_vel_bias_last[W_BR]);
}

/* ---- 转向环 PID 调试量 ---- */
float Chassis_GetSteerITerm(void)
{
    return s_steer.out_i;      /* = ki * integrator (当前 ki=0 → 恒为 0) */
}

float Chassis_GetSteerIntegrator(void)
{
    return s_steer.integrator;
}

/**
 * @brief 读取本次转向中 |I 项| 的峰值 (峰值保持)
 * @note  仅转向时会刷新; 到下一次 Chassis_Rotate 才清零。
 *        转向环不运行时恒为 0。
 */
float Chassis_GetSteerIPeak(void)
{
    return s_steer_i_peak;
}

void Chassis_SteerDebugLog(void)
{
    elog_i("STEER", "on=%d err=%.2f P=%.2f I=%.2f D=%.2f out=%.2f integ=%.1f",
           (int)s_turn_open,
           (double)s_steer.last_error,
           (double)s_steer.out_p, (double)s_steer.out_i, (double)s_steer.out_d,
           (double)s_steer.out, (double)s_steer.integrator);
}
