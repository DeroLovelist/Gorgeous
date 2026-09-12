#include "stm32f4xx.h"
#include "stm32f407xx.h"
#include "MyI2C.h"
#include "stm32f4xx_hal.h"

/* 位翻转半周期延时计数：168MHz 下约 100kHz（JY61P 上限 400kHz）。
 * 若模块仍不响应可继续调大；若想更快可调小。 */
#define MYI2C_DELAY_CNT  200

void MyI2C_W_SCL(MyI2C_Driver_t* self, uint8_t BitValue)
{
    HAL_GPIO_WritePin(self->pere->SCL_port, self->pere->SCL_pin, (GPIO_PinState)BitValue);
    for(volatile int i=0;i<MYI2C_DELAY_CNT;i++); // 简单延时，调整时序
}

void MyI2C_W_SDA(MyI2C_Driver_t* self, uint8_t BitValue)
{
    HAL_GPIO_WritePin(self->pere->SDA_port, self->pere->SDA_pin, (GPIO_PinState)BitValue);
    for(volatile int i=0;i<MYI2C_DELAY_CNT;i++); // 简单延时，调整时序
}

uint8_t MyI2C_R_SDA(MyI2C_Driver_t* self)
{
    return HAL_GPIO_ReadPin(self->pere->SDA_port, self->pere->SDA_pin);
}

void MyI2C_Start(MyI2C_Driver_t* self)
{
    MyI2C_W_SDA(self, 1);
    MyI2C_W_SCL(self, 1);
    MyI2C_W_SDA(self, 0);
    MyI2C_W_SCL(self, 0);
}

void MyI2C_Stop(MyI2C_Driver_t* self)
{
    MyI2C_W_SDA(self, 0);
    MyI2C_W_SCL(self, 1);
    MyI2C_W_SDA(self, 1);
}

void MyI2C_Send_Byte(MyI2C_Driver_t* self, uint8_t data)
{
    for (int i = 0; i < 8; i++)
    {
        MyI2C_W_SDA(self, (data & 0x80) >> 7);
        MyI2C_W_SCL(self, 1);
        MyI2C_W_SCL(self, 0);
        data <<= 1;
    }
}

void MyI2C_ReceiveByte(MyI2C_Driver_t* self, uint8_t* data)
{
    *data = 0;
    MyI2C_W_SDA(self, 1);
    for (int i = 0; i < 8; i++)
    {
        *data <<= 1;
        MyI2C_W_SCL(self, 1);
        if(MyI2C_R_SDA(self))
        {
            *data |= 0x01;
        }
        MyI2C_W_SCL(self, 0);
    }
}

void MyI2C_SendAck(MyI2C_Driver_t* self, uint8_t ack)
{
    MyI2C_W_SDA(self, ack);
    MyI2C_W_SCL(self, 1);
    MyI2C_W_SCL(self, 0);
}

void MyI2C_ReceiveAck(MyI2C_Driver_t* self, uint8_t* ack)
{
    MyI2C_W_SDA(self, 1);
    MyI2C_W_SCL(self, 1);
    *ack = MyI2C_R_SDA(self);
    MyI2C_W_SCL(self, 0);
}

void MyI2C_Init(MyI2C_Driver_t* self)
{
    //时钟选择
    if (self->pere->SCL_port == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
    else if (self->pere->SCL_port == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
    else if (self->pere->SCL_port == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
    else if (self->pere->SCL_port == GPIOD) __HAL_RCC_GPIOD_CLK_ENABLE();
    else if (self->pere->SCL_port == GPIOE) __HAL_RCC_GPIOE_CLK_ENABLE();
    if (self->pere->SDA_port == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
    else if (self->pere->SDA_port == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
    else if (self->pere->SDA_port == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
    else if (self->pere->SDA_port == GPIOD) __HAL_RCC_GPIOD_CLK_ENABLE();
    else if (self->pere->SDA_port == GPIOE) __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configure SCL pin
    GPIO_InitStruct.Pin = self->pere->SCL_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(self->pere->SCL_port, &GPIO_InitStruct);

    // Configure SDA pin
    GPIO_InitStruct.Pin = self->pere->SDA_pin;
    HAL_GPIO_Init(self->pere->SDA_port, &GPIO_InitStruct);

    // Set both pins high
    MyI2C_W_SCL(self, 1);
    MyI2C_W_SDA(self, 1);
}

/* 静态实例（避免依赖 malloc：工程使用 nosys.specs，无堆可用） */
static MyI2C_Driver_t s_myi2c;
static I2CPERE s_myi2c_pere;
static I2CFUN s_myi2c_fun;

MyI2C_Driver_t* MyI2C_Create(GPIO_TypeDef* SCL_port,uint16_t SCL_pin,GPIO_TypeDef* SDA_port,uint16_t SDA_pin)
{
    s_myi2c.pere = &s_myi2c_pere;
    s_myi2c.fun = &s_myi2c_fun;

    //引脚分配
    s_myi2c_pere.SCL_port = SCL_port;
    s_myi2c_pere.SCL_pin = SCL_pin;
    s_myi2c_pere.SDA_port = SDA_port;
    s_myi2c_pere.SDA_pin = SDA_pin;

    //函数映射
    s_myi2c_fun.MyI2C_Init = MyI2C_Init;
    s_myi2c_fun.MyI2C_Send_Byte = MyI2C_Send_Byte;
    s_myi2c_fun.MyI2C_Receive_Byte = MyI2C_ReceiveByte;
    s_myi2c_fun.MyI2C_SendAck = MyI2C_SendAck;
    s_myi2c_fun.MyI2C_ReceiveAck = MyI2C_ReceiveAck;
    s_myi2c_fun.MyI2C_Start = MyI2C_Start;
    s_myi2c_fun.MyI2C_Stop = MyI2C_Stop;

    return &s_myi2c;
}
