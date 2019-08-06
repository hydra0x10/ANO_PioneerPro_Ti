#ifndef _DRV_UART_H_
#define _DRV_UART_H_
#include "sysconfig.h"

void Drv_Uart1Init(uint32_t baudrate);
void Drv_Uart1SendBuf(u8 *data, u8 len);
void Drv_Uart1TxCheck(void);
void Drv_Uart2Init(uint32_t baudrate);
void Drv_Uart2SendBuf(u8 *data, u8 len);
void Drv_Uart2TxCheck(void);
void Drv_Uart3Init(uint32_t baudrate);
void Drv_Uart3SendBuf(u8 *data, u8 len);
void Drv_Uart3TxCheck(void);
void Drv_Uart4Init(uint32_t baudrate);
void Drv_Uart4SendBuf(u8 *data, u8 len);
void Drv_Uart4TxCheck(void);
void Drv_Uart5Init(uint32_t baudrate);
void Drv_Uart5SendBuf(u8 *data, u8 len);
void Drv_Uart5TxCheck(void);
#endif
