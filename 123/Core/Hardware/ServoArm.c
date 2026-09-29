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

/* 初始姿态(0~4095): 底座3926 / 大臂2479 / 辅助752 / 辅助1287 / 夹爪800(张开)
 * ⚠️ 非 const: 示教标定 HOME 时通过 Servos_SetHomePositions() 更新 */
static uint16_t SERVO_POS_HOME[SERVO_COUNT] = {227,2454,885,1256,3029};

//227,2454,885,1256,3029复位参数（小车初始化参数）
//207,425,1988,2135,2542扫码参数（直行一定距离机械臂停下扫码）
//232,1026,1377,1176,2528看球参数（直行一定距离机械臂停下看球）【ok】
//217,143,1995,1623,2198	抓夹移动到小球前的参数
//217,143,1995,1623,2666	抓夹抓取小球
//217,2073,649,1979,2666  大臂抬起
//2243,886,1872,1650,2666	底盘移动到另外一侧准备放球
//2243,1086,1593,1206,2666 摄像头识别球桶
//2243,28,2155,2234,2666   机械臂移动到放置小球的位置
//2243,28,2155,2234,2198	 机械臂放置小球
//2168,1285,1469,1703,2198 机械臂抬起
//181,1357,1505,1865,2212  机械臂转动到准备识别靶子的位置
//158,1190,1712,1385,2214	 摄像头识别靶子
//176,717,1995,1535,2219	 发射激光
//158,1190,1712,1385,2214	 大臂抬起
//1190,1311,1404,1302,2216 摄像头识别人质
//1190,448,1547,2419,2219	 机械臂准备求助人质
//1190,448,1547,2419,2430	 求助成功人质
//1190,1892,1040,2037,2430 大臂抬起
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
    // Servos_SetPositions((uint16_t *)SERVO_POS_HOME, 1500);
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
