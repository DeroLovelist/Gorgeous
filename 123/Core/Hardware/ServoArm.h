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

/**
 * @brief  初始化机械臂: 初始化舵机串口、使能扭矩并回到初始姿态
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
