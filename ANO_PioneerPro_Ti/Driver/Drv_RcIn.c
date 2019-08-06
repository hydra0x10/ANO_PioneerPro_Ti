#include "Drv_RcIn.h"
#include "Timer.h"
#include "hw_ints.h"
#include "uart.h"
//
#include "Ano_RC.h"

union PPM  RC_PPM;
//RCData_t   SbusData;
/**********************************************************************************************************
*函 数 名: PPM_Cal
*功能说明: PPM通道数据计算
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void PPM_Cal(uint32_t  PulseHigh)
{
    static uint8_t Chan = 0;
    /*脉宽高于一定值说明一帧数据已经结束*/
    if(PulseHigh > 5000)
    {
        /*一帧数据解析完成*/
        
        Chan = 0;

    }
    else
    {
		/*脉冲高度正常*/
        if (PulseHigh > PULSE_MIN && PulseHigh < PULSE_MAX)
        {
            if(Chan < 16)
            {
							ch_watch_dog_feed(Chan);
              RC_PPM.Captures[Chan++] = PulseHigh;
            }
        }
    }
}
static void PPM_Decode(void)
{
	static uint32_t	PeriodVal1,PeriodVal2 = 0;
	static uint32_t PulseHigh;
	/*清除中断标志*/
	ROM_TimerIntClear( WTIMER1_BASE , TIMER_CAPB_EVENT );
	/*获取捕获值*/	
	PeriodVal1 = ROM_TimerValueGet( WTIMER1_BASE , TIMER_B );
	if( PeriodVal1 > PeriodVal2 )
		PulseHigh =  (PeriodVal1 - PeriodVal2) /80;
	else
		PulseHigh =  (PeriodVal1  - PeriodVal2 + 0xffffff)/80;
		PeriodVal2 = PeriodVal1;
		PPM_Cal(PulseHigh);
}
void Drv_PpmInit(void)
{
	ROM_SysCtlPeripheralEnable(PPM_SYSCTL);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER1);
	/*GPIOC配置为定时器捕获模式*/
	ROM_GPIOPinTypeTimer(PPM_PORTS, PPM_PIN);
	ROM_GPIOPinConfigure(PPM_FUNCTION);
	/*配置定时器5B为捕获上升沿*/
	ROM_TimerConfigure( WTIMER1_BASE ,TIMER_CFG_SPLIT_PAIR | TIMER_CFG_B_CAP_TIME_UP ); 
	ROM_TimerControlEvent(WTIMER1_BASE,TIMER_B,TIMER_EVENT_POS_EDGE);	
	ROM_TimerLoadSet( WTIMER1_BASE , TIMER_B , 0xffff );
	ROM_TimerPrescaleSet( WTIMER1_BASE , TIMER_B , 0xff );
	/*开启定时器中断*/
	TimerIntRegister(WTIMER1_BASE,  TIMER_B , PPM_Decode);	
	ROM_IntPrioritySet( INT_WTIMER1B , USER_INT6);
	ROM_TimerIntEnable( WTIMER1_BASE , TIMER_CAPB_EVENT);
	ROM_TimerEnable( WTIMER1_BASE, TIMER_B );
	ROM_IntEnable( INT_WTIMER1B );
}


u16 Rc_Sbus_In[16];
u8 sbus_flag;
/*
sbus flags的结构如下所示：
flags：
bit7 = ch17 = digital channel (0x80)
bit6 = ch18 = digital channel (0x40)
bit5 = Frame lost, equivalent red LED on receiver (0x20)
bit4 = failsafe activated (0x10) b: 0001 0000
bit3 = n/a
bit2 = n/a
bit1 = n/a
bit0 = n/a

*/
static void Sbus_Decode(uint8_t data)
{
	static uint8_t i;
    static uint8_t DataCnt  = 0;
	static uint8_t SUBS_RawData[25];
	/*接收数据*/
	 SUBS_RawData[DataCnt++]=data;  
    /*每帧数据长度为25*/
    if(DataCnt >= 25)
    {
        /*判断帧头帧尾是否正确 只要有一个不正确就退出函数*/
        if(SUBS_RawData[0] == 0x0F && SUBS_RawData[24] == 0)
		{
			DataCnt = 0;
			Rc_Sbus_In[0] = (s16)(SUBS_RawData[2] & 0x07) << 8 | SUBS_RawData[1];
			Rc_Sbus_In[1] = (s16)(SUBS_RawData[3] & 0x3f) << 5 | (SUBS_RawData[2] >> 3);
			Rc_Sbus_In[2] = (s16)(SUBS_RawData[5] & 0x01) << 10 | ((s16)SUBS_RawData[4] << 2) | (SUBS_RawData[3] >> 6);
			Rc_Sbus_In[3] = (s16)(SUBS_RawData[6] & 0x0F) << 7 | (SUBS_RawData[5] >> 1);
			Rc_Sbus_In[4] = (s16)(SUBS_RawData[7] & 0x7F) << 4 | (SUBS_RawData[6] >> 4);
			Rc_Sbus_In[5] = (s16)(SUBS_RawData[9] & 0x03) << 9 | ((s16)SUBS_RawData[8] << 1) | (SUBS_RawData[7] >> 7);
			Rc_Sbus_In[6] = (s16)(SUBS_RawData[10] & 0x1F) << 6 | (SUBS_RawData[9] >> 2);
			Rc_Sbus_In[7] = (s16)SUBS_RawData[11] << 3 | (SUBS_RawData[10] >> 5);
			
			Rc_Sbus_In[8] = (s16)(SUBS_RawData[13] & 0x07) << 8 | SUBS_RawData[12];
			Rc_Sbus_In[9] = (s16)(SUBS_RawData[14] & 0x3f) << 5 | (SUBS_RawData[13] >> 3);
			Rc_Sbus_In[10] = (s16)(SUBS_RawData[16] & 0x01) << 10 | ((s16)SUBS_RawData[15] << 2) | (SUBS_RawData[14] >> 6);
			Rc_Sbus_In[11] = (s16)(SUBS_RawData[17] & 0x0F) << 7 | (SUBS_RawData[16] >> 1);
			Rc_Sbus_In[12] = (s16)(SUBS_RawData[18] & 0x7F) << 4 | (SUBS_RawData[17] >> 4);
			Rc_Sbus_In[13] = (s16)(SUBS_RawData[20] & 0x03) << 9 | ((s16)SUBS_RawData[19] << 1) | (SUBS_RawData[18] >> 7);
			Rc_Sbus_In[14] = (s16)(SUBS_RawData[21] & 0x1F) << 6 | (SUBS_RawData[20] >> 2);
			Rc_Sbus_In[15] = (s16)SUBS_RawData[22] << 3 | (SUBS_RawData[21] >> 5);
			sbus_flag = SUBS_RawData[23];
			/*一帧数据解析完成*/
			
			//user
			//
			if(sbus_flag & 0x10)
			{
				//如果有数据且能接收到有失控标记，则不处理，转嫁成无数据失控。
			}
			else
			{
				//否则有数据就喂狗
				for(u8 i = 0;i < 8;i++)//原RC接收程序只设计了8个通道
				{
					ch_watch_dog_feed(i);
				}
			}			
		}
		else
		{
			for( i=0; i<24;i++)
				SUBS_RawData[i] = SUBS_RawData[i+1];
			DataCnt = 24;
		}
	}

}
static void Sbus_IRQHandler(void)
{
	uint8_t com_data;	
	/*获取中断标志 原始中断状态 屏蔽中断标志*/		
	uint32_t flag = ROM_UARTIntStatus(SBUS_UART,1);
	/*清除中断标志*/
	ROM_UARTIntClear(SBUS_UART,flag);
	ROM_UARTRxErrorClear( SBUS_UART );
	/*判断FIFO是否还有数据*/
	while(ROM_UARTCharsAvail(SBUS_UART))			
	{		
		com_data=UART3->DR;
		Sbus_Decode(com_data);	
	}
}
void Drv_SbusInit(void)
{
	ROM_SysCtlPeripheralEnable(SBUS_SYSCTL);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);
	/*GPIO的UART模式配置*/
	ROM_GPIOPinConfigure(UART3_RX);
	ROM_GPIOPinTypeUART( UART3_PORT ,UART3_PIN_RX );
	/*配置串口的波特率和时钟源*/
	ROM_UARTConfigSetExpClk( SBUS_UART ,SysCtlClockGet(), SBUS_BAUDRATE ,UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_TWO | UART_CONFIG_PAR_EVEN );
	/*FIFO设置*/	
	ROM_UARTFIFOLevelSet( SBUS_UART , UART_FIFO_TX1_8 , UART_FIFO_RX1_8 );
	ROM_UARTFIFOEnable(SBUS_UART);
	/*使能串口*/
	ROM_UARTEnable( SBUS_UART );
	/*串口中断配置与使能*/		
	UARTIntRegister( SBUS_UART , Sbus_IRQHandler );
	ROM_IntPrioritySet( INT_UART3 , USER_INT6 );
	ROM_UARTIntEnable( SBUS_UART , UART_INT_RX | UART_INT_OE );
}
