/**
 * @file    ServoArm.h
 * @brief   飞特(Feetech) STS/SMS 串行舵机机械臂控制模块
 *
 * 说明:
 *   - 舵机走 USART2(PD5/PD6), 直连飞特官方驱动板(纯串口, 无方向脚)。
 *   - 底层协议由 SCSLib(SCServo) 提供(WritePosEx / SyncWritePosEx / ReadPos ...)。
 *   - 本模块把机械臂封装成高层接口, 供 MissionControl 调用。
 *
 * 舵机编号(共 5 个, ID 1~5):
 *   1 = 底座旋转, 2 = 大臂升降, 5 = 夹爪开合, 3/4 = 辅助关节。
 *   (动作值均为 0~4095, 对应 0~360°, 具体姿态需实测标定)
 */

#ifndef __SERVOARM_H
#define __SERVOARM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SERVO_COUNT   5      /* 机械臂舵机数量 */
#define SERVO_ARM_ID_MIN  1
#define SERVO_ARM_ID_MAX  5

/* 舵机选择掩码(bit0=ID1 底座, bit1=ID2 大臂, bit2=ID3 副关节,
 *                bit3=ID4 腕部, bit4=ID5 夹爪)
 * 用途: “分步动作” —— 一次只让其中几个舵机动, 避免整臂同时扫动剐蹭车架 */
#define SERVO_MASK_ID1        0x01u
#define SERVO_MASK_ID2        0x02u
#define SERVO_MASK_ID3        0x04u
#define SERVO_MASK_ID4        0x08u
#define SERVO_MASK_ID5        0x10u
#define SERVO_MASK_ALL        0x1Fu
#define SERVO_MASK_ARM_BODY   (SERVO_MASK_ID1 | SERVO_MASK_ID2 | SERVO_MASK_ID3)  /* 底座+大臂+副关节 */
#define SERVO_MASK_WRIST_GRIP (SERVO_MASK_ID4 | SERVO_MASK_ID5)                   /* 腕部+夹爪 */

/* 上电/复位“回初始姿态”的运动时间(ms): 值大 = 慢 = 安全(防剐蹭)。
 * 同时被 ServoArm_Init() 和 Mission_Init() 使用, 保证两处一致 */
#define SERVO_HOME_MOVE_MS    10000

/* ---- 通用“速度下限 / 加速度”默认值 -----------------------------------
 * 这是 Servos_SetPositions()/Servos_SetPositionsMasked() 一直在用的值;
 * 单独列出来是为了能【按次】指定更柔的参数 —— 见下面
 * Servos_SetPositionsMaskedEx()。
 *   speed_min: 速度下限(步/秒)。短距离动作算出来的速度会被它抬到这里:
 *              默认 200 步/秒 ⇒ 转 5 码只要 25ms = 一次“抽搐”, 手臂惯性
 *              下容易过冲+余振(画面跟着抖)。微动用 20~60 会柔得多。
 *   acc:       加速度(0~254, 协议值, 越小起步/停住越柔)。
 * ⚠️ 只是默认值, 不会因为改了这里而影响任何动作 —— 要用请显式传参。 */
#define SERVO_SPEED_MIN_DEF   200
#define SERVO_ACC_DEF          50

/**
 * @brief  初始化机械臂: 初始化舵机串口、等舵机上电、读回实际位置、
 *         使能扭矩并回到初始姿态
 * @note   需在 main() 中 SCServo 串口(USART2)初始化完成后调用
 */
void ServoArm_Init(void);

/**
 * @brief  设置所有 SERVO_COUNT 个舵机到指定位置(同步写, 舵机同时运动)
 * @param  positions 长度为 SERVO_COUNT 的目标位置数组 (0~4095)
 * @param  time_ms   期望运动时间(ms), 内部换算为舵机速度; 0 表示最快
 * @note   本函数为阻塞发送, 但不等待舵机真正到位; 调用方自行延时/判断
 */
void Servos_SetPositions(uint16_t positions[SERVO_COUNT], uint16_t time_ms);

/**
 * @brief  只设置 mask 选中的舵机位置(其余舵机保持不动)
 * @param  positions 长度为 SERVO_COUNT 的目标位置数组 (0~4095)
 * @param  mask      舵机选择掩码, 见上面 SERVO_MASK_*
 * @param  time_ms   期望运动时间(ms), 0 表示最快
 * @note   用于“分步动作”: 先让大臂等到位, 再动腕部/夹爪。
 *         例: 先 Servos_SetPositionsMasked(pos, SERVO_MASK_ARM_BODY, 2500);
 *             再 Servos_SetPositionsMasked(pos, SERVO_MASK_WRIST_GRIP, 1200);
 */
void Servos_SetPositionsMasked(const uint16_t positions[SERVO_COUNT],
                               uint8_t mask, uint16_t time_ms);

/**
 * @brief  只设置 mask 选中的舵机位置, 并【按次】指定速度下限与加速度
 * @param  positions 长度为 SERVO_COUNT 的目标位置数组 (0~4095)
 * @param  mask      舵机选择掩码, 见上面 SERVO_MASK_*
 * @param  time_ms   期望运动时间(ms), 0 表示最快
 * @param  speed_min 本次速度下限(步/秒)。传 SERVO_SPEED_MIN_DEF(200)
 *                   = 与 Servos_SetPositionsMasked() 完全等价
 * @param  acc       本次加速度(0~254)。传 SERVO_ACC_DEF(50) = 默认
 * @note   用途: “精对准微动”。通用接口的速度下限会把几码的微动钳成
 *         200 步/秒(二十几毫秒冲到位) ⇒ 手臂被“抽”一下, 过冲+余振,
 *         摄像头跟着抖、下一帧误差不准 ⇒ 反复修 = 看起来一直在晃。
 *         这里传一个小下限(如 20)就能让它按 time_ms 慢慢走完。
 * ⚠️ 只用于【短距离微动】: 下限给小了再用于长距离动作会慢得离谱。
 *
 * 例(打靶精对准, 5 码微步走 250ms 而不是 25ms):
 *   Servos_SetPositionsMaskedEx(pose, SERVO_MASK_ID1, 600, 20, 20);
 */
void Servos_SetPositionsMaskedEx(const uint16_t positions[SERVO_COUNT], uint8_t mask,
                                 uint16_t time_ms, uint16_t speed_min, uint8_t acc);

/**
 * @brief  更新上电初始姿态(示教标定 HOME 时同步, 下次 ServoArm_Init 生效)
 * @param  positions 长度为 SERVO_COUNT 的初始位置数组 (0~4095)
 */
void Servos_SetHomePositions(const uint16_t positions[SERVO_COUNT]);

/**
 * @brief  获取上电初始姿态数组指针(唯一来源, 供 MissionControl 的 HOME 动作共用)
 * @retval 指向内部 SERVO_POS_HOME 的指针(可直接写入)
 */
uint16_t *Servos_GetHomePositions(void);

/**
 * @brief  设置单个舵机位置(阻塞, 等待应答)
 * @param  servo_id  舵机 ID (1~5)
 * @param  position  目标位置 (0~4095)
 * @param  time_ms   期望运动时间(ms)
 */
void Servos_SetSinglePosition(uint8_t servo_id, uint16_t position, uint16_t time_ms);

/**
 * @brief  让所有舵机掉电卸力(可手动转动)
 */
void Servos_UnloadAll(void);

/**
 * @brief  读取所有舵机当前位置(阻塞, 逐个读)
 */
void Servos_ReadPositions(void);

/**
 * @brief  获取最近一次读取到的舵机位置
 * @param  servo_id 舵机 ID (1~5)
 * @retval 位置值; 失败返回 -1
 */
int32_t Servos_GetPosition(uint8_t servo_id);

/**
 * @brief  使能/关闭指定舵机扭矩
 * @param  servo_id 舵机 ID
 * @param  enable   1=上电使能, 0=掉电卸力
 */
void Servos_SetTorque(uint8_t servo_id, uint8_t enable);

#ifdef __cplusplus
}
#endif

#endif /* __SERVOARM_H */
