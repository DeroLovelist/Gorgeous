/*
恒速模式例子
*/

#include "stm32f4xx.h"
#include "SCServo.h"
#include "uart.h"

void setup()
{
  Uart_Init(115200);
  HAL_Delay(1000);
  WheelMode(1);//恒速模式
}

void loop()
{
  WriteSpe(1, 2000, 50);//舵机(ID1)以最高速度V=2000步/秒，加速度A=50(50*100步/秒^2)，旋转
  HAL_Delay(4000);
  WriteSpe(1, 0, 50);//舵机(ID1)以加速度A=50(50*100步/秒^2)，停止旋转(V=0)
  HAL_Delay(2000);
  WriteSpe(1, -2000, 50);//舵机(ID1)以最高速度V=-2000步/秒，加速度A=50(50*100步/秒^2)，反向旋转
  HAL_Delay(4000);
  WriteSpe(1, 0, 50);//舵机(ID1)以加速度A=50(50*100步/秒^2)，停止旋转(V=0)
  HAL_Delay(2000);
}
