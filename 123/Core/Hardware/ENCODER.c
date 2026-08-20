#include "ENCODER.h"

/* ==================== 内部变量(每通道一份) ==================== */
static uint32_t Encoder_LastCount[ENC_CH_MAX] = {0}; /* 上一次读取的计数值 */
static int32_t  Encoder_Speed[ENC_CH_MAX]     = {0}; /* 当前转速 (count/s) */
static uint32_t Encoder_SpeedTick[ENC_CH_MAX] = {0}; /* 上次速度计算时刻 */
static int32_t  Encoder_StepAcc[ENC_CH_MAX]   = {0}; /* 步数累加器(除4取整,消除余数误差) */

/* ==================== 内部工具 ==================== */

/* 电机号(1~4) -> 编码器通道索引; 无效返回 ENC_CH_MAX */
static ENC_Channel_t Enc_GetCh(uint8_t n)
{
	switch (n)
	{
	case 1:  return ENC_CH1; /* 电机1: TIM1 */
	case 2:  return ENC_CH2; /* 电机2: TIM5 */
	case 3:  return ENC_CH3; /* 电机3: TIM4 */
	case 4:  return ENC_CH4; /* 电机4: TIM3 */
	default: return ENC_CH_MAX;
	}
}

/* 电机号(1~4) -> 定时器句柄指针; 无效返回 NULL */
static TIM_HandleTypeDef *Enc_GetTim(uint8_t n)
{
	switch (n)
	{
	case 1:  return &htim1; /* 电机1 */
	case 2:  return &htim5; /* 电机2 */
	case 3:  return &htim4; /* 电机3 */
	case 4:  return &htim3; /* 电机4 */
	default: return NULL;
	}
}

/* 是否为 32 位计数器 (TIM1/TIM5); 否则为 16 位 (TIM3/TIM4) */
static uint8_t Enc_Is32Bit(ENC_Channel_t ch)
{
	return (ch == ENC_CH1 || ch == ENC_CH2);
}

/* 按计数器宽度计算有符号增量 (自动处理溢出回绕) */
static int32_t Enc_CalcDelta(ENC_Channel_t ch, uint32_t now)
{
	uint32_t last = Encoder_LastCount[ch];

	if (Enc_Is32Bit(ch))
	{
		/* 32 位差值: 回绕自动还原 (例 last=0xFFFFFFF0, now=5 -> 21) */
		return (int32_t)(now - last);
	}
	else
	{
		/* 16 位差值: 例 last=65530, now=5 -> (int16_t)(uint16_t)(5-65530)=11
		 *           例 last=5, now=65530 -> (int16_t)65525 = -11 */
		return (int16_t)(uint16_t)(now - last);
	}
}

/* ==================== 初始化 ==================== */

/**
 * @brief  启动全部四路编码器计数
 * @note   编码器引脚的复用(AF)配置已由 CubeMX 生成的 HAL_TIM_Encoder_MspInit()
 *         在 MX_TIMx_Init() -> HAL_TIM_Encoder_Init() 时自动完成, 此处无需重复配置,
 *         只需启动计数。本函数由 motor 模块的 Motor_Init() 调用。
 */
void Encoder_Init(void)
{
	/* 启动全部四路编码器 (CubeMX 只初始化定时器寄存器, 不会启动计数) */
	Encoder_Start(1);
	Encoder_Start(2);
	Encoder_Start(3);
	Encoder_Start(4);
}

/**
 * @brief  启动指定电机编码器计数
 * @param  n 电机号 1~4
 */
void Encoder_Start(uint8_t n)
{
	TIM_HandleTypeDef *htim = Enc_GetTim(n);
	ENC_Channel_t ch = Enc_GetCh(n);

	if (htim == NULL || ch >= ENC_CH_MAX)
	{
		return;
	}

	HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);

	/* 记录初始值并复位软件状态 */
	Encoder_LastCount[ch] = __HAL_TIM_GET_COUNTER(htim);
	Encoder_SpeedTick[ch] = HAL_GetTick();
	Encoder_Speed[ch] = 0;
}

/**
 * @brief  停止指定电机编码器计数
 * @param  n 电机号 1~4
 */
void Encoder_Stop(uint8_t n)
{
	TIM_HandleTypeDef *htim = Enc_GetTim(n);

	if (htim == NULL)
	{
		return;
	}

	HAL_TIM_Encoder_Stop(htim, TIM_CHANNEL_ALL);
}

/* ==================== 读取 ==================== */

/**
 * @brief  读取指定电机编码器绝对计数值
 * @param  n 电机号 1~4
 * @return 当前计数值 (TIM1/TIM5 32 位, TIM3/TIM4 16 位), 溢出自动回绕
 */
uint32_t Encoder_Read(uint8_t n)
{
	TIM_HandleTypeDef *htim = Enc_GetTim(n);

	if (htim == NULL)
	{
		return 0;
	}

	return __HAL_TIM_GET_COUNTER(htim);
}

/**
 * @brief  读取指定电机编码器有符号计数值
 * @param  n 电机号 1~4
 * @return 有符号计数值 (反转时计数器递减, 有符号解读以区分方向)
 */
int32_t Encoder_GetCount(uint8_t n)
{
	TIM_HandleTypeDef *htim = Enc_GetTim(n);

	if (htim == NULL)
	{
		return 0;
	}

	return (int32_t)__HAL_TIM_GET_COUNTER(htim);
}

/**
 * @brief  获取本次读取相对于上次的变化量 (自动处理回绕)
 * @param  n 电机号 1~4
 * @return 有符号增量, >0=正转(CW), <0=反转(CCW)
 */
int16_t Encoder_GetDelta(uint8_t n)
{
	ENC_Channel_t ch = Enc_GetCh(n);
	TIM_HandleTypeDef *htim = Enc_GetTim(n);
	uint32_t now;
	int32_t delta;

	if (htim == NULL || ch >= ENC_CH_MAX)
	{
		return 0;
	}

	now = __HAL_TIM_GET_COUNTER(htim);
	delta = Enc_CalcDelta(ch, now);

	/* 更新速度计算 (每 100ms 更新一次) */
	uint32_t tick_now = HAL_GetTick();
	uint32_t dt = tick_now - Encoder_SpeedTick[ch];
	if (dt >= 100)
	{
		/* speed = delta / dt * 1000  (count/s) */
		if (dt > 0)
		{
			Encoder_Speed[ch] = (int32_t)delta * 1000 / (int32_t)dt;
		}
		Encoder_SpeedTick[ch] = tick_now;
	}

	Encoder_LastCount[ch] = now;
	return (int16_t)delta;
}

/**
 * @brief  获取指定电机编码器转速 (count/s)
 * @param  n 电机号 1~4
 * @return 每秒计数变化量
 */
int32_t Encoder_GetSpeed(uint8_t n)
{
	ENC_Channel_t ch = Enc_GetCh(n);

	if (ch >= ENC_CH_MAX)
	{
		return 0;
	}

	return Encoder_Speed[ch];
}

/* ==================== 清零 ==================== */

/**
 * @brief  清零指定电机编码器硬件计数器
 * @param  n 电机号 1~4
 */
void Encoder_ResetCount(uint8_t n)
{
	TIM_HandleTypeDef *htim = Enc_GetTim(n);

	if (htim == NULL)
	{
		return;
	}

	__HAL_TIM_SET_COUNTER(htim, 0);
}

/**
 * @brief  清零指定电机编码器计数器并重置内部软件状态
 * @param  n 电机号 1~4
 */
void Encoder_Reset(uint8_t n)
{
	ENC_Channel_t ch = Enc_GetCh(n);
	TIM_HandleTypeDef *htim = Enc_GetTim(n);

	if (htim == NULL || ch >= ENC_CH_MAX)
	{
		return;
	}

	__HAL_TIM_SET_COUNTER(htim, 0);
	Encoder_LastCount[ch] = 0;
	Encoder_Speed[ch] = 0;
	Encoder_SpeedTick[ch] = HAL_GetTick();
	Encoder_StepAcc[ch] = 0;
}

/* ==================== 步数 (已除 4 倍频) ==================== */

/**
 * @brief  获取指定电机编码器步数增量 (已除 4, 每物理格 = 1 步)
 * @param  n 电机号 1~4
 * @return 有符号步数增量
 *
 * 原理:
 *   TI12 4倍频下, Encoder_GetDelta() 返回原始计数增量.
 *   使用累加器 Encoder_StepAcc 累积余数, 避免连续微小误差.
 *
 *   例: 连续 5 次读到 delta=1:
 *     stepAcc: 1→2→3→4→0 (第 4 次时 step=1, stepAcc=0)
 *     最终 step=1, 而不是 5 次都返回 0.
 */
int16_t Encoder_GetStepDelta(uint8_t n)
{
	ENC_Channel_t ch = Enc_GetCh(n);
	int16_t delta;
	int16_t step;

	if (ch >= ENC_CH_MAX)
	{
		return 0;
	}

	delta = Encoder_GetDelta(n);
	step = 0;

	Encoder_StepAcc[ch] += delta;

	/* 整数除法: 商 = 步数, 余数留在累加器中 */
	step = (int16_t)(Encoder_StepAcc[ch] / (int32_t)ENCODER_PPR);
	Encoder_StepAcc[ch] = Encoder_StepAcc[ch] % (int32_t)ENCODER_PPR;

	return step;
}

/**
 * @brief  获取指定电机编码器当前累计步数 (对计数值直接除 4)
 * @param  n 电机号 1~4
 * @return 当前步数
 */
int32_t Encoder_GetStep(uint8_t n)
{
	ENC_Channel_t ch = Enc_GetCh(n);
	TIM_HandleTypeDef *htim = Enc_GetTim(n);

	if (htim == NULL || ch >= ENC_CH_MAX)
	{
		return 0;
	}

	/* 加上累加器中的余数再除, 保持一致性 */
	return ((int32_t)__HAL_TIM_GET_COUNTER(htim) + Encoder_StepAcc[ch]) / ENCODER_PPR;
}
