/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
 * 作者    ：匿名科创
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 * 描述    ：ICM20602驱动
**********************************************************************************/
#include "Drv_icm20602.h"
#include "Drv_spi.h"
#include "Drv_Bsp.h"
#include "hw_ints.h"
//
#include "Ano_Scheduler.h"

void Drv_Icm20602IrqHandler(void)
{
	//清除中断标记
	GPIOIntClear(ICM20602_READY_PORT, ICM20602_READY_PIN);
	//执行中断函数
	//Drv_Icm20602_Read();
	//利用icm的1ms中断做1ms任务
	INT_1ms_Task();
}
void Drv_Icm20602CSPinInit(void)
{
	ROM_SysCtlPeripheralEnable(ICM_CSPIN_SYSCTL);
	ROM_GPIOPinTypeGPIOOutput(ICM20602_CS_PORT,ICM20602_CS_PIN);
	ROM_GPIOPinWrite(ICM20602_CS_PORT, ICM20602_CS_PIN,ICM20602_CS_PIN);
}
void Drv_Icm20602ReadyPinInit(void)
{
	ROM_SysCtlPeripheralEnable(ICM_READYPIN_SYSCTL);
	ROM_GPIODirModeSet(ICM20602_READY_PORT, ICM20602_READY_PIN, GPIO_DIR_MODE_IN);
	ROM_GPIOPinTypeGPIOInput(ICM20602_READY_PORT, ICM20602_READY_PIN);
	ROM_GPIOPadConfigSet(ICM20602_READY_PORT,ICM20602_READY_PIN,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPD);
	ROM_GPIOIntTypeSet(ICM20602_READY_PORT, ICM20602_READY_PIN , GPIO_RISING_EDGE);
	//GPIO注册中断
	GPIOIntRegister(ICM20602_READY_PORT, Drv_Icm20602IrqHandler);
	//使能中断
	GPIOIntEnable(ICM20602_READY_PORT, ICM20602_READY_PIN);
	//设置中断优先级
	ROM_IntPrioritySet(ICM20602_READY_INT_PORT, USER_INT7);
}

static void icm20602_enable(u8 ena)
{
	if(ena)
		ROM_GPIOPinWrite(ICM20602_CS_PORT, ICM20602_CS_PIN,0);
	else
		ROM_GPIOPinWrite(ICM20602_CS_PORT, ICM20602_CS_PIN,ICM20602_CS_PIN);
}

static void icm20602_readbuf(u8 reg, u8 length, u8 *data)
{
	icm20602_enable(1);
	Drv_Spi0SingleWirteAndRead(reg|0x80);
	Drv_Spi0Receive(data,length);
	icm20602_enable(0);
}

static u8 icm20602_writebyte(u8 reg, u8 data)
{
	u8 status;
	
	icm20602_enable(1);
	status = Drv_Spi0SingleWirteAndRead(reg);
	Drv_Spi0SingleWirteAndRead(data);
	icm20602_enable(0);
	return status;
}
/**************************实现函数********************************************
*功　　能:	  读 修改 写 指定设备 指定寄存器一个字节 中的1个位
reg	   寄存器地址
bitNum  要修改目标字节的bitNum位
data  为0 时，目标位将被清0 否则将被置位
*******************************************************************************/
static void icm20602_writeBit(u8 reg, u8 bitNum, u8 data) 
{
    u8 b;
    icm20602_readbuf(reg, 1, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
	icm20602_writebyte(reg, b);
}
/**************************实现函数********************************************
*功　　能:	    读 修改 写 指定设备 指定寄存器一个字节 中的多个位
reg	   寄存器地址
bitStart  目标字节的起始位
length   位长度
data    存放改变目标字节位的值
******************************************************************************
static void icm20602_writeBits(u8 reg,u8 bitStart,u8 length,u8 data)
{
    u8 b,mask;
    icm20602_readbuf(reg, 1, &b);
    mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
    data <<= (8 - length);
    data >>= (7 - bitStart);
    b &= mask;
    b |= data;
    icm20602_writebyte(reg, b);
}*/

static void icm20602_setIntEnabled ( void )
{
	icm20602_writeBit ( MPUREG_INT_PIN_CFG, ICM_INTCFG_INT_LEVEL_BIT, ICM_INTMODE_ACTIVEHIGH );
	icm20602_writeBit ( MPUREG_INT_PIN_CFG, ICM_INTCFG_INT_OPEN_BIT, ICM_INTDRV_PUSHPULL );
	icm20602_writeBit ( MPUREG_INT_PIN_CFG, ICM_INTCFG_LATCH_INT_EN_BIT, ICM_INTLATCH_50USPULSE);//MPU6050_INTLATCH_WAITCLEAR );
	icm20602_writeBit ( MPUREG_INT_PIN_CFG, ICM_INTCFG_INT_RD_CLEAR_BIT, ICM_INTCLEAR_ANYREAD );

	icm20602_writeBit ( MPUREG_INT_ENABLE, ICM_INTERRUPT_DATA_RDY_BIT, 1 );
	Drv_Icm20602ReadyPinInit();
}
static void icm20602_INT_Config(void)
{
	icm20602_setIntEnabled();
}
/**************************实现函数********************************************
*功　　能:	    初始化icm进入可用状态。
*******************************************************************************/
static u8 ICM_ID;
u8 Drv_Icm20602Init(void)
{
	icm20602_writebyte(MPU_RA_PWR_MGMT_1,0x80);
	MyDelayMs(10);
	icm20602_writebyte(MPU_RA_PWR_MGMT_1,0x01);
	MyDelayMs(10);
	
	u8 tmp;
	icm20602_readbuf(MPUREG_WHOAMI, 1, &tmp);
	if(tmp != MPU_WHOAMI_20602)
	return 0;

	/*复位reg*/
	icm20602_writebyte(MPU_RA_SIGNAL_PATH_RESET,0x03);
	MyDelayMs(10);
  /*复位reg*/
	icm20602_writebyte(MPU_RA_USER_CTRL,0x01);	
	MyDelayMs(10);

	icm20602_writebyte(0x70,0x40);//dmp 
	MyDelayMs(10);
	icm20602_writebyte(MPU_RA_PWR_MGMT_2,0x00);
	MyDelayMs(10);
	//不分频，配置内部lpf以后，最高1000hz采样，同时对应产生1ms中断
	icm20602_writebyte(MPU_RA_SMPLRT_DIV,0);
	MyDelayMs(10);
	
	/*陀螺仪LPF 20HZ*/
	icm20602_writebyte(MPU_RA_CONFIG,ICM20602_LPF_20HZ);
	MyDelayMs(10);
	/*陀螺仪量程 +-2000dps*/
	icm20602_writebyte(MPU_RA_GYRO_CONFIG,(3 << 3));
	MyDelayMs(10);
	/*加速度计量程 +-16G*/
	icm20602_writebyte(MPU_RA_ACCEL_CONFIG,(3 << 3));
	MyDelayMs(10);
	/*加速度计LPF 20HZ*/
	icm20602_writebyte(0X1D,0x04);
	MyDelayMs(10);
	/*关闭低功耗*/
	icm20602_writebyte(0X1E,0x00);
	MyDelayMs(10);
	/*关闭FIFO*/
	icm20602_writebyte(0X23,0x00);
	MyDelayMs(10);
	
	icm20602_INT_Config();
	//读取ID
	icm20602_readbuf(MPU_RA_WHO_AM_I, 1, &ICM_ID);
	//
	if(ICM_ID == 0X12)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}



u8 mpu_buffer[14];

void Drv_Icm20602_Read( void )
{
	//读取传感器寄存器，连续读14个字节
	icm20602_readbuf(MPUREG_ACCEL_XOUT_H,14,mpu_buffer);
	//数据赋值
	ICM_Get_Data();
}

#include "Ano_Sensor_Basic.h"
void ICM_Get_Data()
{
	s16 temp[2][3];
	//	/*读取buffer原始数据*/
	temp[0][X] = (s16)((((u16)mpu_buffer[0]) << 8) | mpu_buffer[1]);//>>1;// + 2 *sensor.Tempreature_C;// + 5 *sensor.Tempreature_C;
	temp[0][Y] = (s16)((((u16)mpu_buffer[2]) << 8) | mpu_buffer[3]);//>>1;// + 2 *sensor.Tempreature_C;// + 5 *sensor.Tempreature_C;
	temp[0][Z] = (s16)((((u16)mpu_buffer[4]) << 8) | mpu_buffer[5]);//>>1;// + 4 *sensor.Tempreature_C;// + 7 *sensor.Tempreature_C;
 
	temp[1][X] = (s16)((((u16)mpu_buffer[ 8]) << 8) | mpu_buffer[ 9]) ;
	temp[1][Y] = (s16)((((u16)mpu_buffer[10]) << 8) | mpu_buffer[11]) ;
	temp[1][Z] = (s16)((((u16)mpu_buffer[12]) << 8) | mpu_buffer[13]) ;

	sensor.Tempreature = ((((int16_t)mpu_buffer[6]) << 8) | mpu_buffer[7]); //tempreature
	/*icm20602温度*/
	sensor.Tempreature_C = sensor.Tempreature/326.8f + 25 ;//sensor.Tempreature/340.0f + 36.5f;
	
	//调整物理坐标轴与软件坐标轴方向定义一致
	sensor.Acc_Original[X] = temp[0][X];
	sensor.Acc_Original[Y] = temp[0][Y];
	sensor.Acc_Original[Z] = temp[0][Z];
	
	sensor.Gyro_Original[X] = temp[1][X];
	sensor.Gyro_Original[Y] = temp[1][Y];
	sensor.Gyro_Original[Z] = temp[1][Z];
}




