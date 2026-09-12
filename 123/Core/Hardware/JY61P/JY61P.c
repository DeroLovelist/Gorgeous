#include "JY61P.h"
#include <stdint.h>

/* 平台头文件：
 * - 换 STM32F407VET6 时，在工程编译宏里定义 STM32F407xx，本文件自动切到 F4 库
 * - 当前 G431 工程（定义 STM32G431xx）自动走 G4 库
 */
#if defined(STM32F407xx)
    #include "stm32f4xx.h"
    #include "stm32f4xx_hal.h"
#else
    #include "stm32g4xx.h"
    #include "stm32g4xx_hal.h"
#endif

void JY61P_Init(JY61P_Driver *self)
{
    // Initialize I2C driver
    self->pere->i2c_driver->fun->MyI2C_Init(self->pere->i2c_driver);

    self->var.yaw = 0.0f;
    self->var.yaw_last = 0.0f;
    self->var.roll = 0.0f;
    self->var.pitch = 0.0f;
    self->var.gx = 0.0f;
    self->var.gy = 0.0f;
    self->var.gz = 0.0f;
    self->var.pid.p = 0.0f;
    self->var.pid.i = 0.0f;
    self->var.pid.d = 0.0f;
    self->var.pid.err = 0.0f;
    self->var.pid.err_last = 0.0f;
    self->var.pid.out = 0.0f;
    self->var.pid.target = 0.0f;
    self->var.pid.err_sum = 0.0f;

    self->data.Roll = 0x3d;
    self->data.Pitch = 0x3e;
    self->data.Yaw = 0x3f;
    self->data.AX = 0x34;
    self->data.AY = 0x35;
    self->data.AZ = 0x36;
    self->data.GX = 0x37;
    self->data.GY = 0x38;
    self->data.GZ = 0x39;
    self->data.HX = 0x3a;
    self->data.HY = 0x3b;
    self->data.HZ = 0x3c;
    self->data.LEDOFF = 0x1b;
}

void JY61P_Write(JY61P_Driver *self, uint8_t RegAddress, uint8_t WriteData)
{
    uint8_t ack;
    // 起始
    self->pere->i2c_driver->fun->MyI2C_Start(self->pere->i2c_driver);

    // 发送地址(写)
    self->pere->i2c_driver->fun->MyI2C_Send_Byte(self->pere->i2c_driver, self->pere->Address << 1);
    self->pere->i2c_driver->fun->MyI2C_ReceiveAck(self->pere->i2c_driver, &ack);

    // 发送寄存器地址
    self->pere->i2c_driver->fun->MyI2C_Send_Byte(self->pere->i2c_driver, RegAddress);
    self->pere->i2c_driver->fun->MyI2C_ReceiveAck(self->pere->i2c_driver, &ack);

    // 发送数据
    self->pere->i2c_driver->fun->MyI2C_Send_Byte(self->pere->i2c_driver, WriteData);
    self->pere->i2c_driver->fun->MyI2C_ReceiveAck(self->pere->i2c_driver, &ack);

    // 停止
    self->pere->i2c_driver->fun->MyI2C_Stop(self->pere->i2c_driver);
}

/* 官方 I2C 写时序：Start -> 地址(写) -> 寄存器 -> DataL -> DataH -> Stop
 * 注意：与串口帧不同，I2C 写入【不含】 0xFF 0xAA 前缀（JY61P 官方要求）*/
void JY61P_Write2(JY61P_Driver *self, uint8_t RegAddress, uint8_t *WriteData)
{
    uint8_t ack;
    // 起始
    self->pere->i2c_driver->fun->MyI2C_Start(self->pere->i2c_driver);

    // 发送地址(写)
    self->pere->i2c_driver->fun->MyI2C_Send_Byte(self->pere->i2c_driver, self->pere->Address << 1);
    self->pere->i2c_driver->fun->MyI2C_ReceiveAck(self->pere->i2c_driver, &ack);

    // 寄存器地址
    self->pere->i2c_driver->fun->MyI2C_Send_Byte(self->pere->i2c_driver, RegAddress);
    self->pere->i2c_driver->fun->MyI2C_ReceiveAck(self->pere->i2c_driver, &ack);

    // 发送数据低字节、数据高字节
    self->pere->i2c_driver->fun->MyI2C_Send_Byte(self->pere->i2c_driver, WriteData[0]);
    self->pere->i2c_driver->fun->MyI2C_ReceiveAck(self->pere->i2c_driver, &ack);

    self->pere->i2c_driver->fun->MyI2C_Send_Byte(self->pere->i2c_driver, WriteData[1]);
    self->pere->i2c_driver->fun->MyI2C_ReceiveAck(self->pere->i2c_driver, &ack);

    // 停止
    self->pere->i2c_driver->fun->MyI2C_Stop(self->pere->i2c_driver);
}

int16_t JY61P_Read(JY61P_Driver *self, uint8_t RegAddress)
{
    uint8_t ack;
    uint8_t DataL = 0, DataH = 0;  /* 初始化为0, 避免I2C读取失败时返回未定义乱码 */
    int16_t ReadData;

    // 起始
    self->pere->i2c_driver->fun->MyI2C_Start(self->pere->i2c_driver);

    // 发送地址(写)
    self->pere->i2c_driver->fun->MyI2C_Send_Byte(self->pere->i2c_driver, self->pere->Address << 1);
    self->pere->i2c_driver->fun->MyI2C_ReceiveAck(self->pere->i2c_driver, &ack);

    // 发送寄存器
    self->pere->i2c_driver->fun->MyI2C_Send_Byte(self->pere->i2c_driver, RegAddress);
    self->pere->i2c_driver->fun->MyI2C_ReceiveAck(self->pere->i2c_driver, &ack);

    // 重复起始
    self->pere->i2c_driver->fun->MyI2C_Start(self->pere->i2c_driver);

    // 发送地址(读)
    self->pere->i2c_driver->fun->MyI2C_Send_Byte(self->pere->i2c_driver, (self->pere->Address << 1) | 1);
    self->pere->i2c_driver->fun->MyI2C_ReceiveAck(self->pere->i2c_driver, &ack);

    // 读低8位 -> ACK
    self->pere->i2c_driver->fun->MyI2C_Receive_Byte(self->pere->i2c_driver, &DataL);
    self->pere->i2c_driver->fun->MyI2C_SendAck(self->pere->i2c_driver, 0);

    // 读高8位 -> NACK
    self->pere->i2c_driver->fun->MyI2C_Receive_Byte(self->pere->i2c_driver, &DataH);
    self->pere->i2c_driver->fun->MyI2C_SendAck(self->pere->i2c_driver, 1);

    // 停止
    self->pere->i2c_driver->fun->MyI2C_Stop(self->pere->i2c_driver);

    // 组合16位数据
    ReadData = (DataH << 8) | DataL;
    return ReadData;
}

void JY61P_ROLL_GET(JY61P_Driver *self)
{
    float roll;
    roll = self->pere->Read(self, self->data.Roll);
    self->var.roll = roll/32768.0f * 180.0f;
}
void JY61P_PITCH_GET(JY61P_Driver *self)
{
    float pitch;
    pitch = self->pere->Read(self, self->data.Pitch);
    self->var.pitch = pitch/32768.0f * 180.0f;
}
void JY61P_YAW_GET(JY61P_Driver *self)
{
    double yaw;
    yaw = self->pere->Read(self, self->data.Yaw);
    self->var.yaw_last = self->var.yaw; // 更新上一次的yaw值
    self->var.yaw = yaw/32768.0f * 180.0f;
}

void JY61P_GX_GET(JY61P_Driver *self)
{
    float gx;
    gx = self->pere->Read(self, self->data.GX);
    self->var.gx = gx/32768.0f * 2000.0f;
}
void JY61P_GY_GET(JY61P_Driver *self)
{
    float gy;
    gy = self->pere->Read(self, self->data.GY);
    self->var.gy = gy/32768.0f * 2000.0f;
}
void JY61P_GZ_GET(JY61P_Driver *self)
{
    float gz;
    gz = self->pere->Read(self, self->data.GZ);
    self->var.gz = gz/32768.0f * 2000.0f;
}

void JY61P_YAW_ZERO(JY61P_Driver *self)
{
    uint8_t unlock_reg1[2] = {0x88, 0xB5};
    self->pere->Write2(self,0x69, unlock_reg1);
    for(uint16_t i=0;i<10000;i++); // 延时等待陀螺仪处理归零指令

    uint8_t unlock_reg2[2] = {0x04, 0x00};
    self->pere->Write2(self,0x01, unlock_reg2); // 寄存器应该是 0x01
    for(uint16_t i=0;i<10000;i++); // 延时等待陀螺仪处理归零指令

    uint8_t unlock_reg3[2] = {0x00, 0x00};
    self->pere->Write2(self,0x00, unlock_reg3); // 寄存器应该是 0x00
    for(uint16_t i=0;i<10000;i++); // 延时等待陀螺仪处理归零指令
}

// 偏航环 位置式 PID（完整可用版）
void JY61P_YAW_PID_OUT(JY61P_Driver *self)
{
    // 1. 获取当前角度
    self->fun->YAW_GET(self);
    // 2. 计算误差（位置式核心）
    self->var.pid.err = self->var.yaw - self->var.pid.target;
    if(self->var.pid.err > 180) self->var.pid.err -= 360; // 误差范围调整为 [-180, 180]
    if(self->var.pid.err < -180) self->var.pid.err += 360; // 误差范围调整为 [-180, 180]
    if(self->var.pid.err<10 && self->var.pid.err>-10) self->var.pid.err_sum += self->var.pid.err; // 积分累加（给I用）
    // 3. 位置式 PID 公式（P + I + D）
    self->var.pid.out  =  0.1f * self->var.pid.p * self->var.pid.err                  // P
                        + 0.1f * self->var.pid.i * self->var.pid.err_sum              // I（积分）
                        + 0.1f * self->var.pid.d * (self->var.pid.err - self->var.pid.err_last); // D（微分）
    if(self->var.pid.out > 100) self->var.pid.out = 100; // 输出限幅
    else if(self->var.pid.out < -100) self->var.pid.out = -100; // 输出限幅
    // 4. 保存上一次误差（给D用）
    self->var.pid.err_last = self->var.pid.err;

    // 积分限幅（防止即使在小范围内也累加过大）
    if(self->var.pid.err_sum > 1000) self->var.pid.err_sum = 1000;
    if(self->var.pid.err_sum < -1000) self->var.pid.err_sum = -1000;
}

void JY61P_YAW_PID_SET(JY61P_Driver *self, float target, float kp, float ki, float kd)
{
    self->var.pid.target = target;
    self->var.pid.err = self->var.pid.target - self->var.yaw;
    self->var.pid.p = kp;
    self->var.pid.i = ki;
    self->var.pid.d = kd;
}

/* 探测 I2C 设备是否存在：发送地址后检查 ACK。返回 1=有设备(ACK)，0=无设备(NACK) */
uint8_t JY61P_Check(JY61P_Driver *self)
{
    uint8_t ack = 1;
    self->pere->i2c_driver->fun->MyI2C_Start(self->pere->i2c_driver);
    self->pere->i2c_driver->fun->MyI2C_Send_Byte(self->pere->i2c_driver, self->pere->Address << 1);
    self->pere->i2c_driver->fun->MyI2C_ReceiveAck(self->pere->i2c_driver, &ack);
    self->pere->i2c_driver->fun->MyI2C_Stop(self->pere->i2c_driver);
    return (ack == 0) ? 1 : 0;   /* ACK=0(低电平) = 设备应答 */
}

/* 静态实例（避免依赖 malloc：工程使用 nosys.specs，无堆可用） */
static JY61P_Driver s_jy61p;
static JY61PPERE s_jy61p_pere;
static JY61PFUN s_jy61p_fun;

JY61P_Driver* JY61P_Create(uint8_t JY61P_Address, GPIO_TypeDef* SCL_port, uint16_t SCL_pin, GPIO_TypeDef* SDA_port, uint16_t SDA_pin)
{
    s_jy61p.pere = &s_jy61p_pere;
    s_jy61p.fun = &s_jy61p_fun;

    s_jy61p_pere.i2c_driver = MyI2C_Create(SCL_port, SCL_pin, SDA_port, SDA_pin);
    s_jy61p_pere.Read = JY61P_Read;
    s_jy61p_pere.Write = JY61P_Write;
    s_jy61p_pere.Write2 = JY61P_Write2;
    s_jy61p_pere.Address = JY61P_Address;

    s_jy61p_fun.Init = JY61P_Init;
    s_jy61p_fun.ROLL_GET = JY61P_ROLL_GET;
    s_jy61p_fun.PITCH_GET = JY61P_PITCH_GET;
    s_jy61p_fun.YAW_GET = JY61P_YAW_GET;
    s_jy61p_fun.GX_GET = JY61P_GX_GET;
    s_jy61p_fun.GY_GET = JY61P_GY_GET;
    s_jy61p_fun.GZ_GET = JY61P_GZ_GET;
    s_jy61p_fun.YAW_ZERO = JY61P_YAW_ZERO;
    s_jy61p_fun.YAW_PID_OUT = JY61P_YAW_PID_OUT;
    s_jy61p_fun.YAW_PID_SET = JY61P_YAW_PID_SET;

    return &s_jy61p;
}
