#ifndef	_UART_H
#define	_UART_H

#include <stdint.h>

/* RX ring buffer size */
#define UART_BUF_SIZE 256

extern volatile uint8_t uartBuf[UART_BUF_SIZE];
extern volatile int head;
extern volatile int tail;

extern void Uart_Init(uint32_t baudRate);
extern void Uart_Flush(void);
extern int16_t Uart_Read(uint8_t *buf , uint16_t len, uint32_t timeout);
extern void Uart_Send(uint8_t *buf , uint16_t len);
extern void Uart_RxISR(void);

#endif
