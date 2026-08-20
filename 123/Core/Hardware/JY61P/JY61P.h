#ifndef __JY61P_H
#define __JY61P_H
#include <stdint.h>
#include "MyI2C.h"

/* =====================================================================
 * JY61P 驱动（维特智能 六轴姿态传感器）
 * ---------------------------------------------------------------------
 * 说明：JY61P 与 JY901 共用同一套 WIT 标准 I2C 协议与寄存器映射，
 *       因此本驱动结构完全复刻 JY901.c，仅修改：
 *         1. 结构体/函数统一命名为 JY61P 前缀，避免与 JY901 链接冲突
 *         2. Write2 写入去掉 0xFF 0xAA 前缀（符合官方 I2C 写时序）
 *         3. JY61P 为 6 轴（无磁力计），HX/HY/HZ 寄存器保留但无效
 * ===================================================================== */

typedef struct JY61PVAR JY61PVAR;
typedef struct JY61PDATA JY61PDATA;
typedef struct JY61PFUN JY61PFUN;
typedef struct JY61PPERE JY61PPERE;
typedef struct JY61P_Driver JY61P_Driver;
typedef struct JY61P_PID JY61P_PID;

struct JY61P_PID
{
    float p;
    float i;
    float d;
    float out;
    float err;
    float err_last;
    float target;
    float err_sum;
};

/*变量*/
struct JY61PVAR
{
    float yaw;
    float yaw_last;
    float roll;
    float pitch;
    float gx;
    float gy;
    float gz;
    JY61P_PID pid;
};
/*内部函数和参数*/
struct JY61PPERE
{
    uint8_t Address;   /* 7 位 I2C 地址，JY61P 默认 0x50 */
    int16_t (*Read)(JY61P_Driver *self,uint8_t RegAddress);
    void (*Write)(JY61P_Driver *self,uint8_t RegAddress, uint8_t WriteData);
    void (*Write2)(JY61P_Driver *self,uint8_t RegAddress, uint8_t* WriteData);
    MyI2C_Driver_t *i2c_driver;
};
/*寄存器地址（与 JY901 完全一致）*/
struct JY61PDATA
{
    uint8_t Roll;
    uint8_t Pitch;
    uint8_t Yaw;
    uint8_t AX;
    uint8_t AY;
    uint8_t AZ;
    uint8_t GX;
    uint8_t GY;
    uint8_t GZ;
    uint8_t HX;   /* JY61P 无磁力计，此寄存器无效 */
    uint8_t HY;   /* JY61P 无磁力计，此寄存器无效 */
    uint8_t HZ;   /* JY61P 无磁力计，此寄存器无效 */
    uint8_t LEDOFF;
};
/*对外函数*/
struct JY61PFUN
{
    void (*Init)(JY61P_Driver *self);
    void (*ROLL_GET)(JY61P_Driver *self);
    void (*PITCH_GET)(JY61P_Driver *self);
    void (*YAW_GET)(JY61P_Driver *self);
    void (*GX_GET)(JY61P_Driver *self);
    void (*GY_GET)(JY61P_Driver *self);
    void (*GZ_GET)(JY61P_Driver *self);
    void (*YAW_ZERO)(JY61P_Driver *self);
    void (*YAW_PID_OUT)(JY61P_Driver *self);
    void (*YAW_PID_SET)(JY61P_Driver *self, float target, float kp,float ki,float kd);
};
struct JY61P_Driver
{
    JY61PVAR var;
    JY61PDATA data;
    JY61PFUN *fun;
    JY61PPERE *pere;
};
JY61P_Driver* JY61P_Create(uint8_t JY61P_Address, GPIO_TypeDef* SCL_port, uint16_t SCL_pin, GPIO_TypeDef* SDA_port, uint16_t SDA_pin);
uint8_t JY61P_Check(JY61P_Driver *self);
#endif /* __JY61P_H */
