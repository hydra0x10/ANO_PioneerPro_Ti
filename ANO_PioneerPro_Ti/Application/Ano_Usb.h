#ifndef __ANO_LED_H
#define __ANO_LED_H
//==ÒýÓÃ
#include "sysconfig.h"

void AnoUsbCdcInit(void);
void AnoUsbCdcSend( const uint8_t* data , uint16_t length );
uint16_t AnoUsbCdcRead( uint8_t* data , uint16_t length );

#endif
