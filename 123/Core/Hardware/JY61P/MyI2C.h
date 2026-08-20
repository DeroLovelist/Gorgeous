#ifndef _MY_I2C_H_
#define _MY_I2C_H_
#include "stm32f4xx.h"
#include <stdint.h>
typedef struct MyI2C_Driver_t MyI2C_Driver_t;
typedef struct I2CPERE I2CPERE;
typedef struct I2CFUN I2CFUN;
struct I2CPERE
{
    GPIO_TypeDef* SCL_port;
    uint16_t SCL_pin;
    GPIO_TypeDef* SDA_port;
    uint16_t SDA_pin;    
};

struct I2CFUN
{
    void (*MyI2C_Init)(MyI2C_Driver_t* self);
    void (*MyI2C_Send_Byte)(MyI2C_Driver_t* self, uint8_t data);
    void (*MyI2C_Receive_Byte)(MyI2C_Driver_t* self, uint8_t* data);
    void (*MyI2C_SendAck)(MyI2C_Driver_t* self,uint8_t ack);
    void (*MyI2C_ReceiveAck)(MyI2C_Driver_t* self, uint8_t* ack);
    void (*MyI2C_Start)(MyI2C_Driver_t* self);
    void (*MyI2C_Stop)(MyI2C_Driver_t* self);
};

struct MyI2C_Driver_t
{
    I2CPERE *pere;
    I2CFUN *fun;
};
MyI2C_Driver_t* MyI2C_Create(GPIO_TypeDef* SCL_port,uint16_t SCL_pin,GPIO_TypeDef* SDA_port,uint16_t SDA_pin);

#endif
