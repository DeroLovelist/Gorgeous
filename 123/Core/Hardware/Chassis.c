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
static int16_t            s_last_pwm[W_NUM]; /* 最近一次输出 PWM (调试用) */

static const float *s_yaw = NULL;            /* 航向角数据源 (度) */
static bool  s_inited = false;
static bool  s_suspended = false;            /* 挂起标志(手动测试时置位) */
static bool  s_moving = false;               /* 平移进行中 */

static bool  s_turn_open = false;            /* 转向进行中 */
static float s_turn_remaining = 0.0f;        /* 剩余转向角度 */
static float s_turn_prev_yaw = 0.0f;        // 上一周期航向 (度, PD 阻尼用)
static uint16_t s_turn_stable = 0;          // 转向停车稳定计数
static uint16_t s_move_stable = 0;           /* 平移停车稳定计数 */

static float s_heading_target = 0.0f;       /* 平移时的目标航向 (度) */
static float s_heading_prev_yaw = 0.0f;     /* 上一周期航向 (度, PD 阻尼用) */
static float s_heading_integral = 0.0f;     /* 航向积分 (度·秒, 抗饱和) */
static float s_heading_err_last  = 0.0f;    /* 最近一次航向误差 (度, 调试) */
static float s_heading_corr_last = 0.0f;    /* 最近一次航向修正 (计数/周期, 调试) */
static int16_t s_vel_bias_last[W_NUM] = { 0, 0, 0, 0 }; /* 最近一次速度偏置 (调试) */

/* 每轮 counts/mm 标定系数 (来自 Chassis.h, 补偿左右轮轮径/编码器差异) */
static const float s_wheel_scale[W_NUM] = { CH_WHEEL_SCALE_FL, CH_WHEEL_SCALE_FR, CH_WHEEL_SCALE_BL, CH_WHEEL_SCALE_BR };

/* 每轮 PWM 补偿系数 (来自 Chassis.h, 分前进/后退, 补偿电机增益差异) */
static const float s_pwm_fwd_scale[W_NUM] = { CH_PWM_SCALE_FWD_FL, CH_PWM_SCALE_FWD_FR, CH_PWM_SCALE_FWD_BL, CH_PWM_SCALE_FWD_BR };
static const float s_pwm_rev_scale[W_NUM] = { CH_PWM_SCALE_REV_FL, CH_PWM_SCALE_REV_FR, CH_PWM_SCALE_REV_BL, CH_PWM_SCALE_REV_BR };

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

/* 输出四路 PWM (换算为"电机正方向") */
static void output_pwm(const int16_t pwm[W_NUM])
{
    for (int i = 0; i < W_NUM; i++)
    {
        float scale = (pwm[i] >= 0) ? s_pwm_fwd_scale[i] : s_pwm_rev_scale[i];
        s_last_pwm[i] = (int16_t)(pwm[i] * s_motor_dir[i] * scale);
        Set_PWM(s_motor[i], s_last_pwm[i]);
    }
}

//电机停止
static void stop_motors(void)
{
    for (int i = 0; i < W_NUM; i++)
    {
        Set_PWM(s_motor[i], 0);
    }
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

    for (int i = 0; i < W_NUM; i++)
    {
        DualPID_SetMaxVel(&s_pid[i], counts_per_cycle);
    }
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
    /* ---- 位置环 (外环): 多快到点/到点多准 ---- */
    const float pos_Kp = 0.6f, pos_Ki = 0.0f, pos_Kd = 0.0f;
    /*   pos_Kp 建议 0.3~1.5 (默认 0.6): 调大接近更快但易超调, 调小更柔更慢
     *   pos_Ki 建议 0 (默认 0): 位置环一般不积分, 防到点来回摆
     *   pos_Kd 建议 0 (默认 0): 抑制到点前抖动, 但放大编码器噪声 */

    /* ---- 速度环 (内环): 实际速度是否听话 ---- */
    const float vel_Kp = 0.4f, vel_Ki = 0.05f, vel_Kd = 0.0f;
    /*   vel_Kp 建议 0.1~1.0 (默认 0.4): 调大跟踪更紧, 过大会 PWM 抖动/电机啸叫
     *   vel_Ki 建议 0~0.2 (默认 0.05): 消除负载/坡度造成的速度稳态误差
     *   vel_Kd 建议 0 (默认 0): 一般不开启 */

    const float max_vel = 40.0f;       /* 默认最大速度 (计数/周期 ≈ 300 mm/s),
                                        *   之后可用 Chassis_SetMaxSpeed() 覆盖 */
    const float max_output = 99.0f;    /* PWM 上限 (= PWM_ARR 满量程), 一般不改 */
    const float vel_ff = 0.3f;         /* 前馈线性增益 (PWM / 计数每周期),
                                        *   见上方“现象速查”的起步调法 */
    const float vel_ff_dead = 10.0f;   /* 死区补偿 (PWM): 低速启不动→调大(≤20),
                                        *   停车后仍缓慢蠕动→调小(≈5) */

    for (int i = 0; i < W_NUM; i++)
    {
        DualPID_Init(&s_pid[i], pos_Kp, pos_Ki, pos_Kd,
                     vel_Kp, vel_Ki, vel_Kd, max_vel, max_output,
                     vel_ff, vel_ff_dead);
        s_target[i] = 0;
        s_pos[i] = 0;
        s_speed[i] = 0;
    }

    /* 转向 PID (对应 pid.h 的 pid_param_t): 输入剩余转向角度(度), 输出转向修正量。
     * 参数: kp=1.5(比例)  ki=0(积分)  kd=1.5(阻尼)  imax=500(积分限幅)。
     * 调参速查:
     *   - 转向转不到位/末段残留角度大: 调大 kp(1.5→2.0) 或调小 Chassis.h 的
     *     CH_ANGLE_ERR_THRESHOLD
     *   - 到目标附近来回震荡/甩尾过冲: 调大 kd(1.5→2.5) 阻尼, 或调小 kp
     *   - 大角度转弯转太慢: 调大 Chassis.h 的 CH_MAX_TURN_ADJUST(30→40)
     * 注: kp 1.0→1.5 是为加强小角度修正, 需配合更小的到位阈值(1.5°)使用。 */
    pid_param_init(&s_steer, 1.5f, 0.0f, 1.5f, 500.0f);

    stop_motors();
    read_encoders();
    for (int i = 0; i < W_NUM; i++)
    {
        s_target[i] = s_pos[i];
    }

    s_moving = false;
    s_turn_open = false;
    s_move_stable = 0;
    s_heading_integral = 0.0f;
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
    }
    s_moving = false;
    s_turn_open = false;
    s_turn_stable = 0;
    s_move_stable = 0;
    s_heading_integral = 0.0f;
    s_heading_prev_yaw = (s_yaw != NULL) ? *s_yaw : 0.0f;
    s_suspended = false;
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
        float corr = 0.0f;
        if (fabsf(yaw_err) >= CH_HEADING_DEADZONE)
        {
            /* 积分 (带抗饱和), 消除持续漂移下的稳态航向误差 */
            s_heading_integral += yaw_err * dt;
            if (s_heading_integral >  CH_HEADING_INTEGRAL_MAX) s_heading_integral =  CH_HEADING_INTEGRAL_MAX;
            if (s_heading_integral < -CH_HEADING_INTEGRAL_MAX) s_heading_integral = -CH_HEADING_INTEGRAL_MAX;

            corr = CH_HEADING_KP * yaw_err + CH_HEADING_KI * s_heading_integral + CH_HEADING_KD * dyaw;
            if (corr >  CH_HEADING_MAX) corr =  CH_HEADING_MAX;
            if (corr < -CH_HEADING_MAX) corr = -CH_HEADING_MAX;
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

        /* 到位阈值内不再修正(死区), 避免在目标角度附近来回震荡 */
        float adj = 0.0f;
        if (fabsf(s_turn_remaining) >= CH_ANGLE_ERR_THRESHOLD)//ch-angle_err_threshold: 转向闭环的死区阈值, 单位: 度, 默认 1.5°; 误差小于该值时不再修正, 避免在目标角度附近来回震荡
        {
            adj = PidLocCtrl(&s_steer, s_turn_remaining) * CH_TURN_SIGN;//s_steer转向PID
            if (adj > CH_MAX_TURN_ADJUST)  adj = CH_MAX_TURN_ADJUST;    //转向限幅
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
            if (iabs(s_target[i] - s_pos[i]) >= CH_POS_THRESHOLD_COUNT ||
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
                s_moving = false;   /* 停稳: 保持目标位置 */
            }
        }
        else
        {
            s_move_stable = 0;
        }
    }

    /* ---- 3. 串级 PID 输出 (注入航向保持速度偏置) ---- */
    int16_t pwm[W_NUM];
    for (int i = 0; i < W_NUM; i++)
    {
        pwm[i] = (int16_t)DualPID_Update(&s_pid[i],
                                         (float)s_target[i], (float)s_pos[i],
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

    /* 每次新动作清零 PID 积分/历史, 避免上一步的积分记忆导致起步不同步 */
    for (int i = 0; i < W_NUM; i++)
    {
        DualPID_Reset(&s_pid[i]);
    }

    /* 按每轮 counts/mm 标定系数缩放目标, 补偿左右轮轮径/编码器差异 */
    for (int i = 0; i < W_NUM; i++)
    {
        float delta = (float)(s_fwd_sign[i] * fwd_cnt + s_strafe_sign[i] * strafe_cnt) * s_wheel_scale[i];
        s_target[i] += (int32_t)(delta >= 0.0f ? delta + 0.5f : delta - 0.5f);
    }

    /* 记录平移起始航向, 供航向保持使用 */
    if (s_yaw != NULL)
    {
        s_heading_target = *s_yaw;
        s_heading_prev_yaw = *s_yaw;
    }

    s_heading_integral = 0.0f;
    s_turn_open = false;
    s_move_stable = 0;
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
    s_heading_integral = 0.0f;
    for (int i = 0; i < W_NUM; i++)
    {
        s_target[i] = s_pos[i];
    }
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
    elog_i("HDG", "err=%.1f int=%.1f corr=%.1f vb=%d,%d,%d,%d",
           (double)s_heading_err_last,
           (double)s_heading_integral,
           (double)s_heading_corr_last,
           (int)s_vel_bias_last[W_FL], (int)s_vel_bias_last[W_FR],
           (int)s_vel_bias_last[W_BL], (int)s_vel_bias_last[W_BR]);
}
