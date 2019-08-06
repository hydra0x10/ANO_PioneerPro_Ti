/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
 * 作者    ：匿名科创
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 * 描述    ：电子罗盘驱动
**********************************************************************************/
#include "drv_ak8975.h"
#include "Drv_spi.h"
#include "Drv_Bsp.h"

void Drv_AK8975CSPinInit(void)
{
	ROM_SysCtlPeripheralEnable(AK_CSPIN_SYSCTL);
	ROM_GPIOPinTypeGPIOOutput(AK8975_CS_PORT,AK8975_CS_PIN);
	ROM_GPIOPinWrite(AK8975_CS_PORT, AK8975_CS_PIN,AK8975_CS_PIN);
}

static void ak8975_enable(u8 ena)
{
	if(ena)
		ROM_GPIOPinWrite(AK8975_CS_PORT, AK8975_CS_PIN,0);
	else
		ROM_GPIOPinWrite(AK8975_CS_PORT, AK8975_CS_PIN,AK8975_CS_PIN);
}

static void ak8975_Trig(void)
{
	ak8975_enable(1);
	Drv_Spi0SingleWirteAndRead(AK8975_CNTL_REG);
	Drv_Spi0SingleWirteAndRead(0x01);
	ak8975_enable(0);
}

static u8 ak8975_buf[6];
void Drv_AK8975_Read(void)
{	
	
	ak8975_enable(1);
	Drv_Spi0SingleWirteAndRead(AK8975_HXL_REG|0x80);
	for(u8 i=0; i<6; i++)
		ak8975_buf[i] = Drv_Spi0SingleWirteAndRead(0xff);
	ak8975_enable(0);
	
	ak8975_Trig();
	
}


void Mag_Get(s16 mag_val[3])
{
	s16 t[3];
	
	t[0] = ((((int16_t)ak8975_buf[1]) << 8) | ak8975_buf[0]) ;
	t[1] = ((((int16_t)ak8975_buf[3]) << 8) | ak8975_buf[2]) ;
	t[2] = ((((int16_t)ak8975_buf[5]) << 8) | ak8975_buf[4]) ;
	
	/*转换坐标轴为ANO坐标*/
	mag_val[0] = +t[0];
	mag_val[1] = -t[1];
	mag_val[2] = -t[2];
}


/******************* (C) COPYRIGHT 2017 ANO TECH *****END OF FILE************/

