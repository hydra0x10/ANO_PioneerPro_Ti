/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
 * 作者    ：匿名科创
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 * 描述    ：GPS驱动
**********************************************************************************/
#include "drv_spl06.h"
#include "Drv_Uart.h"
#include "Drv_gps.h"
#include "Ano_Imu.h"


#define GPS_UART	USART1

GPS_INF Gps_information;
unsigned short len;

unsigned char GPS_ubx_check_sum(unsigned char *Buffer)
{
	unsigned char CK_A = 0, CK_B = 0;
	unsigned short  i;
	
	len = Buffer[4] + (Buffer[5]<<8);
	if (len > 100)
	{
		return 0;
	}
	for (i = 2; i < len+6; i++)
	{
		CK_A = CK_A + Buffer[i];
		CK_B = CK_B + CK_A;
	}
	if (CK_A == Buffer[len+6] && CK_B == Buffer[len+7])
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

static void UART_Write_D(const unsigned char *send_buff, unsigned char len)
{
//	unsigned char i;
//	
//	for (i = 0; i < len; i++)
//	{
//		while(USART_GetFlagStatus(GPS_UART, USART_FLAG_TC)== RESET);
//		USART_SendData(GPS_UART,send_buff[i]);
//	}
	//
	Drv_Uart1SendBuf((u8 *)send_buff, len);
}



const unsigned char gps_petrol_out_config[28]=
{
//	0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x80,0x25,0x00,0x00,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0xA0,0xA9
	0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0xC2,0x01,0x00,0x01,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0xB8,0x42
};

const unsigned char gps_pvt_out_config[90]=
{
	0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x18,0xE1,
//	0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x12,0x01,0x01,0x01,0x01,0x01,0x00,0x27,0x3D
	0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x12,0xB9,
	0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x13,0xC0,
	0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x16,0xD5,
	0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x12,0x00,0x00,0x00,0x00,0x00,0x00,0x22,0x29
};

const unsigned char gps_rate_out_config[16]=
{
	0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12  //len 14
};

const unsigned char Enter_Send[]={0xB5,0x62,0x06,0x00,0x01,0x00,0x01,0x08,0x22};

void gps_baudrate_config(void)
{
	Delay_ms(200);
//		UART_Write_D(gps_rate_out_config,14);
	UART_Write_D(gps_petrol_out_config,28);

	UART_Write_D(Enter_Send,sizeof(Enter_Send));
		Delay_ms(20);
}

void gps_config(void)
{
	Delay_ms(100);

	UART_Write_D(gps_pvt_out_config,90);
		Delay_ms(20);

	UART_Write_D(gps_rate_out_config,14);
		Delay_ms(20);

//	UART_Write_D(gps_petrol_out_config,28);
//		Delay_ms(20);
	
	UART_Write_D(Enter_Send,sizeof(Enter_Send));
		Delay_ms(20);
}

#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )
	
extern u8 data_to_send[50];
extern void ANO_DT_Send_Data(u8 *dataToSend , u8 length);
short Gps_send_Temp[10];
float wcx_acc_use;		
float wcy_acc_use;

//void ANO_DT_Send_Gps_data(void)
//{
//	u8 _cnt=0;
//	vs16 _temp;
//	
//	data_to_send[_cnt++]=0xAA;
//	data_to_send[_cnt++]=0xAA;
//	data_to_send[_cnt++]=0xF1;
//	data_to_send[_cnt++]=8;
//	
//	_temp = (short)(Gps_send_Temp[0]);
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = (short)(Gps_send_Temp[1]);
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = (short)(Gps_send_Temp[2]);
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = (short)(Gps_send_Temp[3]);
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	
//	u8 sum = 0;
//	for(u8 i=0;i<_cnt;i++)
//		sum += data_to_send[i];
//	data_to_send[_cnt++]=sum;
//	
//	ANO_DT_Send_Data(data_to_send, _cnt);
//}

void Drv_GpsPin_Init(void)
{
	Delay_ms(200);
	Drv_Uart1Init(9600);
	gps_baudrate_config();
	Drv_Uart1Init(38400);
	gps_baudrate_config();
	Drv_Uart1Init(115200);
	gps_config();
}

unsigned char GPS_data_buff[100];
unsigned char GPS_get_cnt = 0;

void GPS_data_analysis(void)
{
//	Gps_information.last_N_vel = (float)Gps_information.N_vel;							//记录上次南北向速度
//	Gps_information.last_E_vel = (float)Gps_information.E_vel;							//记录上次东西向速度
	
	Gps_information.satellite_num = GPS_data_buff[29];									//卫星数量
	Gps_information.longitude = GPS_data_buff[30] + (GPS_data_buff[31]<<8) + (GPS_data_buff[32]<<16) + (GPS_data_buff[33]<<24);		//经度
	Gps_information.latitude  = GPS_data_buff[34] + (GPS_data_buff[35]<<8) + (GPS_data_buff[36]<<16) + (GPS_data_buff[37]<<24);		//纬度
	Gps_information.N_vel	  = GPS_data_buff[54] + (GPS_data_buff[55]<<8) + (GPS_data_buff[56]<<16) + (GPS_data_buff[57]<<24);		//南北向速度
	Gps_information.E_vel	  = GPS_data_buff[58] + (GPS_data_buff[59]<<8) + (GPS_data_buff[60]<<16) + (GPS_data_buff[61]<<24);		//东西向速度
	
	Gps_information.N_vel /=10;								//单位换算 cm/s
	Gps_information.E_vel /=10;								//单位换算 cm/s	

	if (Gps_information.satellite_num >= 6 && Gps_information.new_pos_get == 0)			//卫星数量到达6颗,且第一次获取经纬度点
	{
		Gps_information.new_pos_get = 1;
		Gps_information.start_longitude = Gps_information.longitude;
		Gps_information.start_latitude  = Gps_information.latitude;
		Gps_information.hope_latitude  = 0;
		Gps_information.hope_longitude = 0;

	}
	
	if (Gps_information.new_pos_get)
	{
		Gps_information.latitude_offset  = Gps_information.latitude  - Gps_information.start_latitude;
		Gps_information.longitude_offset = Gps_information.longitude - Gps_information.start_longitude;
	}

	Gps_information.run_heart++;	
}

void Drv_GpsGetOneByte(u8 data)
{
	if (GPS_get_cnt == 0)
	{
		if (data == 0xB5)									//帧头1
		{
			GPS_data_buff[GPS_get_cnt] = data;
			GPS_get_cnt = 1;
		}
	}
	else if (GPS_get_cnt == 1)
	{
		if (data == 0x62)									//帧头2
		{
			GPS_data_buff[GPS_get_cnt] = data;
			GPS_get_cnt = 2;
		}
		else
		{
			GPS_get_cnt = 0;
		}
	}
	else
	{
		GPS_data_buff[GPS_get_cnt] = data;
		GPS_get_cnt++;
		if (GPS_get_cnt >= 100)
		{
			GPS_get_cnt = 0;
			
			if (GPS_ubx_check_sum(GPS_data_buff))			//GPS数据校验
			{
				GPS_data_analysis();						//GPS数据解析
			}
		}
	}			
}

//void Uart1_GPS_IRQ(void)  
//{  
////	unsigned char RX_dat;  

////	if ( GPS_UART->SR & USART_SR_ORE ) //ORE??
////    {
////        RX_dat = GPS_UART->DR;
////    }
////	
////	if(USART_GetITStatus(GPS_UART,USART_IT_RXNE)==SET)//USART_IT_RXNE:????  
////	{   
////		USART_ClearITPendingBit(GPS_UART,USART_IT_RXNE);  
////		RX_dat=USART_ReceiveData(GPS_UART); 
////		if (GPS_get_cnt == 0)
////		{
////			if (RX_dat == 0xB5)									//帧头1
////			{
////				GPS_data_buff[GPS_get_cnt] = RX_dat;
////				GPS_get_cnt = 1;
////			}
////		}
////		else if (GPS_get_cnt == 1)
////		{
////			if (RX_dat == 0x62)									//帧头2
////			{
////				GPS_data_buff[GPS_get_cnt] = RX_dat;
////				GPS_get_cnt = 2;
////			}
////			else
////			{
////				GPS_get_cnt = 0;
////			}
////		}
////		else
////		{
////			GPS_data_buff[GPS_get_cnt] = RX_dat;
////			GPS_get_cnt++;
////			if (GPS_get_cnt >= 100)
////			{
////				GPS_get_cnt = 0;
////				
////				if (GPS_ubx_check_sum(GPS_data_buff))			//GPS数据校验
////				{
////					GPS_data_analysis();						//GPS数据解析
////				}
////			}
////		}			
////	}	
//} 


void WCXY_Acc_Get_Task(void)//最小周期
{
	wcx_acc_use += 0.015f *(imu_data.w_acc[X] - wcx_acc_use);
	wcy_acc_use += 0.015f *(imu_data.w_acc[Y] - wcy_acc_use);
}

static u8 home_locked;
static u8 update_cnt = 10;
void GPS_Data_Processing_Task(u8 dT_ms)
{
	static float err_N_step, err_E_step;
	static unsigned char last_gps_heart = 0; 
	//
	if(update_cnt<100)
	{
		update_cnt ++;
	}
	//
	if (Gps_information.run_heart != last_gps_heart)
	{
		last_gps_heart = Gps_information.run_heart;
		err_N_step = (float)(Gps_information.N_vel - Gps_information.last_N_vel)/update_cnt;		//计算两次GPS速度误差作为插值
		err_E_step = (float)(Gps_information.E_vel - Gps_information.last_E_vel)/update_cnt;
		update_cnt = 0;
	}
	//
	Gps_information.last_N_vel += err_N_step;									//对速度插值
	Gps_information.last_E_vel += err_E_step;
	
	if(
		Gps_information.satellite_num >= 6 
		&& flag.flight_mode >= LOC_HOLD
	&& update_cnt < 100 //等于100代表断线
	)//CH_N[AUX1] > -200 )
	{
		if(1) //(!flag.taking_off)
		{
			switchs.gps_on = 1;							//开启GPS模式
		}
	}
	else
	{
		switchs.gps_on = 0;							//关闭GPS模式
		//
		Gps_information.last_N_vel = 0;
		Gps_information.last_E_vel = 0;
	}
	//没有起飞的时候才能记录返航点
	if (!flag.taking_off)
	{
		if (switchs.gps_on)
		{
			Gps_information.hope_latitude = Gps_information.latitude_offset;		//记录期望位置为当前位置
			Gps_information.hope_longitude = Gps_information.longitude_offset;
			Gps_information.home_latitude = Gps_information.latitude_offset;		//记录返航点位置
			Gps_information.home_longitude = Gps_information.longitude_offset;
			home_locked = 1;
		}
	}
	else
	{
		if (switchs.gps_on && home_locked != 0)
		{	
			if (flag.flight_mode == RETURN_HOME || flag.rc_loss_back_home )//(CH_N[AUX1] > 200)
			{
				if (!Gps_information.Back_home_f)			
				{
					Gps_information.Back_home_f = TRUE;		//打开返航
					Gps_information.hope_latitude = Gps_information.home_latitude;			//改变期望位置为起飞点位置
					Gps_information.hope_longitude = Gps_information.home_longitude;
				}
			}	
			else
			{
				if (Gps_information.Back_home_f)			
				{
					Gps_information.Back_home_f = FALSE;	//关闭返航
					Gps_information.hope_latitude = Gps_information.latitude_offset;		//改变期望位置为当前位置
					Gps_information.hope_longitude = Gps_information.longitude_offset;
				}
			}
		}			
	}
}

