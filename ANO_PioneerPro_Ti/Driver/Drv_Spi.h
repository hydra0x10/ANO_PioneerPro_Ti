#ifndef _DRV_SPI_H_
#define _DRV_SPI_H_

#include "sysconfig.h"

void Drv_Spi0Init(void);
uint8_t Drv_Spi0SingleWirteAndRead(uint8_t SendData);
void Drv_Spi0Transmit(uint8_t *ucp_Data, uint16_t us_Size);
void Drv_Spi0Receive(uint8_t *ucp_Data, uint16_t us_Size);
#endif
