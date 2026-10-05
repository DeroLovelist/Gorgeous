#include "main.h"
#include "Chassis.h"
#include "pid.h"
#include "motor.h"
#include "ENCODER.h"
#include "elog.h"
#include <math.h>
#include <stdio.h>   /* snprintf (MOVE 日志把超时时长打出来) */
#include <string.h>  /* memcpy (MOVE 日志从 ISR 搬出来时用) */

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
static int32_t            s_speed_filt[W_NUM];/* |速度| 低通值 (计数/周期, 判"停稳"用) */
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
static uint16_t s_arrived_cycles = 0;        /* 位置已达标持续的周期数 (兜底: 到位了却一直判不到停稳) */
static int32_t  s_move_start_pos[W_NUM];     /* 本段平移的起点位置 (算"实走多少"用, 见 MOVE 日志) */
static uint16_t s_move_cycles = 0;           /* 本段平移已运行的控制周期数 (超时兜底用) */
static uint16_t s_move_timeout_cycles = 0;   /* 本段平移的超时周期数 (按距离自适应, 见 add_move) */
static uint16_t s_stall_cycles = 0;          /* "停住但没到位"连续周期数 (见 Chassis.h 的 CH_STALL_DETECT_CYCLES) */

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
    .vel_ff = 0.18f,  .vel_ff_dead = 14.0f,
    .hd_Kp = 1.8f,   .hd_Ki = 1.0f,    .hd_Kd = 1.5f,
    .hd_max = 25.0f, .hd_imax = 50.0f, .hd_dead = 0.5f,
    .hd_trim = 0.0f   /* 直行实测很直 → 航向保持保持原样(只用速度偏置), 不动 */
};

/* 平移模式 (纯左移/右移) —— 横移漂移优先调这里
 * 2026-09-27: pos_Ki 0→0.25, hd_Kp 4.5→2.0, hd_max 12→25 (理由同 s_cfg_straight)
 * 2026-09-27(二修, 治"横移走不直"): 横移时辊子侧向刮地被地面拧着转,
 *   四轮位置环会把"速度环偏置"抵消掉 → 改成 hd_trim=1.0(把修正量投入位置
 *   目标微调) 才真能修住; 同时把增益降下来(机制变有效了, 不改会过冲)。
 * ⚠️ 2026-09-29: hd_Ki 0.8 → **0**。原因: hd_trim 这条路本身就是个积分器
 *   (trim 是"累加"出来的), 再叠 PID 的 I 项 = 双积分 → 积分饱和后大过冲。
 *   实测铁证: 横移途中 yaw 从 -0.4° 自己甩到 +7.3°, 而同时 trim 顶在 -250
 *   (已经要求转 10° 而车才偏 0.4°) → 就是它。横移段的稳态误差由 trim 的
 *   积分作用消化, 不需要 I 项。(若仍残留固定偏差, 可给 0.1~0.2 一点点) */
static const ChassisPidCfg_t s_cfg_strafe = {
    .pos_Kp = 0.6f,  .pos_Ki = 0.25f,  .pos_Kd = 0.0f,
    .vel_Kp = 0.4f,  .vel_Ki = 0.05f,  .vel_Kd = 0.0f,//0.4
    .vel_ff = 0.3f,  .vel_ff_dead = 12.0f,
    .hd_Kp = 1.60f,   .hd_Ki = 1.0f,    .hd_Kd = 1.5f,//1.4，0.0f，1.0
    .hd_max = 16.0f, .hd_imax = 50.0f, .hd_dead = 0.3f,//16，10,40
    .hd_trim = 0.0f//1.0f
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

        /* |速度| 一阶低通 (时间常数≈4 个控制周期=80ms):
         * 轮子静止时编码器仍有 ±1~2 计数抖动(日志实测 M1=61557→61558→61558),
         * 直接用瞬时速度判"停稳"会永远判不到。
         * ⚠⚠️ 步长必须"四舍五入": 若写成 s_speed_filt += (|d| - s_speed_filt) / 4,
         *   整数除法在 |d|=0 时会把值卡在 2~3 下不去(3/4=0) → 速度判据永远不满足
         *   → 车早就到位却每段都干等到超时(实测每段白等 3~12s, MOVE 行全是 TO)。 */
        {
            int32_t dabs = iabs(d);
            int32_t diff = dabs - s_speed_filt[i];
            s_speed_filt[i] += (diff >= 0) ? (diff + 2) / 4 : -((-diff + 2) / 4);
        }
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
    s_move_stable = 0;
    s_arrived_cycles = 0;
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
        s_speed_filt[i] = 0;
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
    s_arrived_cycles = 0;
    s_move_cycles = 0;
    s_move_timeout_cycles = (uint16_t)(CH_MOVE_TIMEOUT_MIN_MS / CH_CTRL_PERIOD_MS);
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
        s_speed_filt[i] = 0;
        s_target[i] = 0;
        s_move_start_pos[i] = 0;
    }
    s_moving = false;
    s_turn_open = false;
    s_turn_stable = 0;
    s_move_stable = 0;
    s_arrived_cycles = 0;
    s_move_cycles = 0;
    s_move_timeout_cycles = (uint16_t)(CH_MOVE_TIMEOUT_MIN_MS / CH_CTRL_PERIOD_MS);
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
 *   - 出现 TO(后面带本段允许时长, 如 TO(11900ms)) → 本段是被超时切断的,
 *     e: 还很大说明没走完: 这时"走了多少"≈速度×超时, 与 ROUTE_x_MM 无关!!
 *     (实踩: 超时设 3000ms 时 900mm 段只走 ~380mm、990mm 段只走 ~660mm,
 *      日志全是 TO。) 先检查 CH_MOVE_TIMEOUT_MIN_MS 会不会偏小, 再调 pos_Ki。 */
/* ⭐ 2026-09-30: 本函数是在 1ms 定时中断里被调用的(调用链:
 *   TIM1_BRK_TIM9_IRQHandler -> HAL_TIM_PeriodElapsedCallback -> Chassis_Tick
 *   -> Chassis_Update_Control -> move_finish_log)
 * 而 elog 是【同步阻塞 + __disable_irq】的串口输出(USART3@115200, 一行约 10ms,
 * 见 easylogger/elog_port.c)。在中断里打印的后果:
 *   ① 控制周期从 20ms 被拉长到 30ms+, 而 pid 里 dt 是写死的 0.02
 *      → 积分/微分按错误的 dt 计算;
 *   ② 这 10ms 内所有中断被关闭(SysTick/编码器/UART 全停),
 *      HAL_GetTick 也不涨(状态机全靠它计时)。
 * 所以这里【只把这一行格式化进缓冲区并置标志】, 真正的打印交给主循环的
 * Chassis_FlushPendingLog()。日志的内容与取值时刻(d/e/yaw)完全不变。 */
static char             s_move_log_buf[160];
static volatile uint8_t s_move_log_pending = 0;

static void move_finish_log(const char *why)
{
    char tag[24];

    if (why != NULL && why[0] != '\0')
    {
        /* 带原因的行(TO/PV)附上本段【实际用时】(ms), 方便看"停多久":
         *   TO(用时) = 超时切断(距离被切短了);
         *   PV(用时) = 位置已到位但速度判据不满足 → 强制结束。 */
        snprintf(tag, sizeof(tag), "%s(%ums)", why,
                 (unsigned)s_move_cycles * CH_CTRL_PERIOD_MS);
    }
    else
    {
        tag[0] = '\0';
    }

    if (s_move_log_pending) return;   /* 上一条还没被主循环取走: 不覆盖(宁丢不叠) */

    snprintf(s_move_log_buf, sizeof(s_move_log_buf),
             "%s d:FL=%ld FR=%ld BL=%ld BR=%ld e:FL=%ld FR=%ld BL=%ld BR=%ld yaw=%.2f",
             tag,
             (long)(s_pos[W_FL] - s_move_start_pos[W_FL]), (long)(s_pos[W_FR] - s_move_start_pos[W_FR]),
             (long)(s_pos[W_BL] - s_move_start_pos[W_BL]), (long)(s_pos[W_BR] - s_move_start_pos[W_BR]),
             (long)(eff_tgt(W_FL) - s_pos[W_FL]), (long)(eff_tgt(W_FR) - s_pos[W_FR]),
             (long)(eff_tgt(W_BL) - s_pos[W_BL]), (long)(eff_tgt(W_BR) - s_pos[W_BR]),
             (double)((s_yaw != NULL) ? *s_yaw : 0.0f));
    s_move_log_pending = 1;
}

/* 由【主循环】调用: 打印中断里攒下的 MOVE 行。
 * 这里才做真正的串口输出(阻塞 ~10ms), 但此时不影响控制中断。 */
void Chassis_FlushPendingLog(void)
{
    char buf[sizeof(s_move_log_buf)];

    if (!s_move_log_pending) return;

    /* 拷贝期间关中断, 避免 ISR 正好在覆盖缓冲区 → 打出半新半旧的行 */
    __disable_irq();
    memcpy(buf, s_move_log_buf, sizeof(buf));
    s_move_log_pending = 0;
    __enable_irq();

    elog_i("MOVE", "%s", buf);
}

void Chassis_Update_Control(void)
{
    if (!s_inited || s_suspended)
    {
        return;
    }

    read_encoders();

    float dt = CH_CTRL_PERIOD_MS / 1000.0f;

    /* ⭐⭐ 2026-09-30/10-01: "卡住"检测 (位置环与航向环都用它)
     * 判定: 连续 CH_STALL_DETECT_CYCLES 个周期都满足
     *        "四轮速度都 < CH_STALL_SPEED_COUNT 且 位置还没到位" → stalled=true。
     * 为什么需要:
     *   ① 航向环: 段末车已停住(但残差 > CH_POS_THRESHOLD_COUNT 判不到位)时, 车头
     *      即使歪着也转不动 → 航向积分只能一直涨(实测 4.5s 内 corr 3.6→22.8 限幅),
     *      而速度偏置一涨就会: ⓐ pos_lim = max_vel-|bias| 把位置环权限掐到只剩 8;
     *      ⓑ "偏置不许反向"把 FR/BR 直接夹成 0 → 四轮锁死、残差永远收不掉。
     *      所以 stalled 时本轮不做纠偏(清积分 + corr=0), 把权限全还给位置环。
     *   ② 位置环: stalled 持续到 CH_STALL_FINISH_CYCLES 就直接结束本段(日志打 "ST"),
     *      不再干等到超时(实测每段白等 3.5~4.5s)。
     * ⚠️ 速度阈值取 CH_STALL_SPEED_COUNT(4)而不是 CH_STOP_SPEED_THRESHOLD(2):
     *    段末常有某一轮以 1~3 计数/周期"爬"(实测 M1 57492→57502), 用 2 判不出
     *    "停住" → 卡住检测失效、偏置照样涨。正常行驶四轮都在 20~30 计数/周期。 */
    bool stalled;
    bool hdg_cut = false;   /* ⭐ 2026-10-01: 段末"停偏置"标志, 见 Chassis.h 的 CH_HDG_CUT_ERR_COUNT */
    {
        bool quiet = true;
        bool not_arrived = false;
        int32_t max_err = 0;   /* 四轮中最大的剩余残差 (计数) */
        for (int i = 0; i < W_NUM; i++)
        {
            int32_t e = iabs(eff_tgt(i) - s_pos[i]);
            if (s_speed_filt[i] >= CH_STALL_SPEED_COUNT) quiet = false;
            if (e >= CH_POS_THRESHOLD_COUNT) not_arrived = true;
            if (e > max_err) max_err = e;
        }
        if (quiet && not_arrived)
        {
            if (s_stall_cycles < 0xFFFFu) s_stall_cycles++;
        }
        else
        {
            s_stall_cycles = 0;
        }
        stalled = (s_stall_cycles >= CH_STALL_DETECT_CYCLES);

        /* ⭐⭐ 2026-10-01 新增: 段末"停偏置"(主动让权给位置环), 详见 Chassis.h 同名宏。
         * 四轮最大残差已经很小(最后十几毫米) → 本周期不做航向纠偏, corr 强制 0。
         * 为什么: 段末 corr 常常顶在 hd_max 饱和, 而饱和的 vel_bias 会让
         *   ⓐ pos_lim = max_vel-|bias| 白扣掉位置环权限; ⓑ "偏置不许反向"把
         *   残差小的轮子夹成 0 → "一条对角还在推、另一条已停" = 纯转动。
         *   日志铁证: 864mm 段末一拍 FL=-46 BR=+4(停) 而 FR=+227 BL=+110(走),
         *   反算 wk=+42 计数 ≈ +1.6°, 与实测 yaw 1.6°→4.3° 吻合。
         * 若 CH_HDG_CUT_ERR_COUNT 设为 0 → 恒为 false, 恢复原行为。 */
        hdg_cut = (s_moving && max_err < (int32_t)CH_HDG_CUT_ERR_COUNT);
    }
    if (stalled || hdg_cut)
    {
        /* 车都不动 / 段末让权: 航向积分没有意义, 清掉, 免得轮子一动就"猛纠一下" */
        s_heading_integral = 0.0f;
    }

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
        /* ⭐ hdg_cut = 段末让权(残差已很小): 本拍 corr 强制 0、vel_bias 全 0,
         * 把最后十几毫米完全交给位置环, 四轮才能同时减速同时停(见 Chassis.h)。 */
        if (!stalled && !hdg_cut && fabsf(yaw_err) >= cfg->hd_dead)
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
        bool pos_ok = true;    /* 四轮位置都在容差内(真正"到位") */
        bool vel_ok = true;    /* 四轮速度都小于"停稳"阈值(用低通速度, 滤掉静止抖动) */
        for (int i = 0; i < W_NUM; i++)
        {
            if (iabs(eff_tgt(i) - s_pos[i]) >= CH_POS_THRESHOLD_COUNT) pos_ok = false;
            if (s_speed_filt[i] >= CH_STOP_SPEED_THRESHOLD)              vel_ok = false;
        }

        if (pos_ok)
        {
            if (s_arrived_cycles < 0xFFFFu) s_arrived_cycles++;
        }
        else
        {
            s_arrived_cycles = 0;
        }

        if (pos_ok && vel_ok)
        {
            if (++s_move_stable >= CH_STOP_STABLE_COUNT)
            {
                s_move_stable = 0;
                s_arrived_cycles = 0;
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

            /* ⭐ 兜底(2026-09-29): 位置已经到位, 但速度/阈值判据迟迟不满足 →
             * 最多再等 CH_STOP_HOLD_MAX_MS 就强制结束本段, 不许白等到超时。
             * 卡过两次同类的坑: ① CH_STOP_SPEED_THRESHOLD 设 1 要求"速度恰好0";
             * ② 低通速度的整数除法卡在 2~3 下不去 → 表现都是"车到了却停着不走/干等
             * 3~12s, MOVE 行带 TO 而 e: 已经很小"。日志里强制结束的行打 "PV"。 */
            if (pos_ok && s_arrived_cycles >= (uint16_t)(CH_STOP_HOLD_MAX_MS / CH_CTRL_PERIOD_MS))
            {
                s_arrived_cycles = 0;
                move_finish_log("PV");
                s_moving = false;
                chassis_freeze();
            }
            /* ⭐ 2026-10-01 兑底: "卡住"太久(车确实不动了, 但位置始终判不到位)
             * → 直接结束本段(日志打 "ST"), 不再干等到超时(实测每段白等 3.5~4.5s)。
             * 典型场景: 横移段末某一轮(实测 FR)落后 ~180 计数, 其余三轮都已到位
             *   停住, 它单独推不动整车 → 四轮都不动 → 判卡住。 */
            else if (s_stall_cycles >= CH_STALL_FINISH_CYCLES)
            {
                s_stall_cycles = 0;
                move_finish_log("ST");
                s_moving = false;
                chassis_freeze();
            }
        }

        /* ⭐ 安全兜底: 单段平移超时就强制结束并打 MOVE 行(带 TO)。
         * 超时周期数是 add_move 里按本段距离算出来的(见 Chassis.h), 不能用固定值:
         * 固定值一旦比"本段应耗时"短, 本段就会被切断 → 车走的距离变成
         * "速度×超时", 与 ROUTE_x_MM 无关(踩过)。 */
        if (s_moving && ++s_move_cycles >= s_move_timeout_cycles)
        {
            move_finish_log("TO");
            s_moving = false;
            s_move_stable = 0;
            s_arrived_cycles = 0;
            chassis_freeze();       /* 超时兜底也要冻结: 残留可能很大(e: 几十~上百),
                                     * 不冻结车会自己猛地拱一下 */
        }
    }

    /* ---- 3. 串级 PID 输出 (注入航向保持速度偏置) ----
     * ⭐⭐ 2026-09-30 关键修复: 【只在"正在平移或正在转向"时才驱动电机】。
     * 原来这一段是无条件执行的: 一段走完(s_moving=0)后, 第 2 段刚把电机置 0,
     * 这里立刻又按"冻结的位置目标"算了一遍 PWM 输出 → 停止后底盘一直挂着一个
     * 很硬的四轮独立位置伺服, 20ms 一次不停地顶残差。
     * 为什么这会造成"停下时抖动 + 斜着漂移":
     *   ① 只要有一点点残差, pid.c 的死区前馈会直接给出 ±vel_ff_dead(12~14)PWM;
     *      而 PWM 只有 0~99 级(1% 一级), 十几个 PWM 足够"顶开"静摩擦 →
     *      轮子一冲就过头 → 反向再顶 → 极限环(肉眼=抖);
     *   ② 四轮静摩擦/PWM 补偿/悬挂载荷都不同 → 不会同时被顶动, 先动的那两个
     *      轮子合成一个固定方向的蠕动(实测现象: M1(BL) 与 M4(FR) 同向前进
     *      = 麦轮的纯 45° 平移, 车头不转);
     *   ③ 机械臂一摆, 反作用力推车 → 位置环立刻"顶回去" → 停机等待期间自己挪。
     * 别人的车停下就是断输出(滑行/短刹车), 所以不会这样。
     * ⚠️ read_encoders() 必须在上面照常每周期调用(否则增量累积, 下次起步猛冲),
     *    这里只是不【驱动电机】。
     * 代价: 停机期间不再"保持位置"(被外力推了不会自己顶回来)。若某段对准
     *      确实需要顶住, 把 CH_IDLE_HOLD_ENABLE 置 1 可恢复原行为对比。 */
#if !CH_IDLE_HOLD_ENABLE
    if (!s_moving && !s_turn_open)
    {
        stop_motors();   /* 也同步软启动斜坡基准(s_last_pwm=0) */
        return;
    }
#endif

    int16_t pwm[W_NUM];

    /* ⭐⭐ 2026-10-01 (C): 四轮"同步降速"。先算四轮剩余距离的平均值, 供下面
     * 对"领先轮"追加反向偏置用。详见 Chassis.h 的 CH_SYNC_GAIN / CH_SYNC_MAX。
     * 只在 CH_SYNC_GAIN > 0 (功能开启) 且正在平移时才算, 免得白花时间。 */
    int32_t err_avg = 0;
    if (CH_SYNC_GAIN > 0.0f && s_moving)
    {
        for (int i = 0; i < W_NUM; i++)
        {
            err_avg += iabs(eff_tgt(i) - s_pos[i]);
        }
        err_avg /= W_NUM;
    }

    for (int i = 0; i < W_NUM; i++)
    {
        /* ⭐ 冻结/结束后不再注入航向速度偏置: vel_bias 是第 0 段(s_moving 还是 1 时)
         * 算出来的, 若原样用到这里, 会在"刚停稳"这一拍再给一次偏航踢(FL/BL +, FR/BR -)。 */
        float bias = s_moving ? (float)vel_bias[i] : 0.0f;

        /* ⭐⭐ C: 同步降速 —— 领先轮(r_i < 平均值)追加一个【反向】偏置, 让它慢
         * 下来等落后的轮子。为什么用偏置而不是改位置目标:
         *   ① 偏置经 pid.c 的 pos_lim = max_vel-|bias| 会把该轮速度压到约
         *      max_vel-2×C, 是"减速", 不会让它反向;
         *   ② 位置目标不动 → 最终每轮仍走到自己的计数目标 → 距离精度不受影响。
         * 典型场景(实测): 右移段 BR 比 FR 快 15~20%, BR 先到位停住, 只剩
         * FR/BL 推车 → 绕车心转动 → 段末甩 3° 且车身斜飘 30mm。
         * 关掉本功能: 把 Chassis.h 的 CH_SYNC_GAIN 置 0。 */
        if (s_moving && CH_SYNC_GAIN > 0.0f)
        {
            int32_t e    = eff_tgt(i) - s_pos[i];
            int32_t lead = err_avg - iabs(e);   /* >0 = 本轮的剩余比平均小 = 领先 */
            if (lead > 0)
            {
                int32_t c = (int32_t)((float)lead * CH_SYNC_GAIN);
                if (c > (int32_t)CH_SYNC_MAX) c = (int32_t)CH_SYNC_MAX;
                if (c > 0)
                {
                    if (e >= 0) bias -= (float)c;   /* 正向运动 → 给反向(减速)偏置 */
                    else        bias += (float)c;   /* 反向运动同理 */
                }
            }
        }

        pwm[i] = (int16_t)DualPID_Update(&s_pid[i],
                                         (float)eff_tgt(i), (float)s_pos[i],
                                         (float)s_speed[i], dt,
                                         bias);
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
    int32_t max_cnt = 0;   /* 本段四轮目标增量的最大绝对值(算超时用) */
    for (int i = 0; i < W_NUM; i++)
    {
        float delta = (float)(s_fwd_sign[i] * fwd_cnt + s_strafe_sign[i] * strafe_cnt) * s_wheel_scale[i];
        int32_t dc = (int32_t)(delta >= 0.0f ? delta + 0.5f : delta - 0.5f);
        s_target[i] += dc;
        s_move_start_pos[i] = s_pos[i];   /* 记本段起点, 结束时算"实走多少"(见 MOVE 日志) */
        if (iabs(dc) > max_cnt) max_cnt = iabs(dc);
    }

    /* ⭐ 超时按本段距离自适应 (见 Chassis.h 的 CH_MOVE_MIN_SPEED_MMPS 说明)。
     * 固定超时一旦小于"本段应耗时"就会把本段切断 → 车走的距离变成"速度×超时",
     * 用户会看到"不管距离宏改多少, 车都走同一个值"。 */
    {
        uint32_t ms = 2000u + (uint32_t)((float)max_cnt / CH_COUNTS_PER_MM
                                        / CH_MOVE_MIN_SPEED_MMPS * 1000.0f);
        if (ms < CH_MOVE_TIMEOUT_MIN_MS) ms = CH_MOVE_TIMEOUT_MIN_MS;
        if (ms > CH_MOVE_TIMEOUT_MAX_MS) ms = CH_MOVE_TIMEOUT_MAX_MS;
        s_move_timeout_cycles = (uint16_t)(ms / CH_CTRL_PERIOD_MS);
        if (s_move_timeout_cycles == 0) s_move_timeout_cycles = 1;
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
    s_arrived_cycles = 0;
    s_move_cycles = 0;
    s_stall_cycles = 0;      /* 新一段开始: 清"卡住"计数(起步瞬间车还没动, 免得误判) */
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
    s_arrived_cycles = 0;
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
