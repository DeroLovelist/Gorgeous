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

/* 舵机 ID 列表: [0]=ID1(底座) [1]=ID2(大臂) [2]=ID3(辅助) [3]=ID4(辅助) [4]=ID5(夹爪) */
static const uint8_t s_servo_ids[SERVO_COUNT] = {1, 2, 3, 4, 5};

/* 最近一次写入的目标位置, 用于由 time_ms 换算速度 */
static uint16_t s_last_pos[SERVO_COUNT] = {0, 0, 0, 0, 0};

/* 最近一次读到的位置 */
static volatile int32_t s_read_pos[SERVO_COUNT] = {-1, -1, -1, -1, -1};

/* 初始姿态(0~4095): 底座640 / 大臂2360 / 辅助1260 / 辅助3400 / 夹爪800(张开) */
static const uint16_t SERVO_POS_HOME[SERVO_COUNT] = {640, 2360, 1260, 3400, 800};

/**
 * @brief  由运动距离与时间换算舵机速度(步/秒)
 */
static uint16_t ServoArm_CalcSpeed(uint16_t from, uint16_t to, uint16_t time_ms)
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
    if (speed < 50) {
        speed = 50;
    }
    if (speed > 4095) {
        speed = 4095;
    }
    return (uint16_t)speed;
}

/**
 * @brief  初始化机械臂
 */
void ServoArm_Init(void)
{
    Uart_Init(115200);   /* 舵机串口 USART2(纯串口, 无方向脚) */

    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        s_last_pos[i] = SERVO_POS_HOME[i];
        s_read_pos[i] = -1;
    }

    /* 上电使能扭矩并回到初始姿态 */
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        Servos_SetTorque(s_servo_ids[i], 1);
    }
    Servos_SetPositions((uint16_t *)SERVO_POS_HOME, 1500);
}

/**
 * @brief  设置所有舵机位置(同步写)
 */
void Servos_SetPositions(uint16_t positions[SERVO_COUNT], uint16_t time_ms)
{
    uint8_t ids[SERVO_COUNT];
    int16_t pos[SERVO_COUNT];
    uint16_t spd[SERVO_COUNT];
    uint8_t acc[SERVO_COUNT];

    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        uint16_t p = positions[i];
        if (p > 4095) {
            p = 4095;
        }
        ids[i] = s_servo_ids[i];
        pos[i] = (int16_t)p;
        spd[i] = ServoArm_CalcSpeed(s_last_pos[i], p, time_ms);
        acc[i] = 50;
        s_last_pos[i] = p;
    }

    SyncWritePosEx(ids, SERVO_COUNT, pos, spd, acc);
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
