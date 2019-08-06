//==引用
#include "Ano_UWB.h"
//
#include "Ano_Imu.h"
#include "Ano_FcData.h"
//==定义
#define fc_sta flag

//==数据声明
_uwb_data_st uwb_data;


//*********************************************************************************************************
/**********************************************************************************************************
*函 数 名: Ano_UWB_Get_Byte
*功能说明: UWB获取字节
*参    数: 数据（1字节）
*返 回 值: 无
**********************************************************************************************************/
static u8 UWB_RxBuffer[256],UWB_data_len = 0,UWB_Data_OK;
void Ano_UWB_Get_Byte(u8 data)
{
	static u8 _data_len = 0;
	static u8 _sta = 0;
	static u8 _rx_buf[256];
	static u8 _rx_buf_len = 0;
	
	if(_sta==0&&data==0xAA)	//帧头0xAA
	{
		_sta=1;
		_rx_buf[0]=data;
	}
	else if(_sta==1&&data==0x30)	//数据源，0x30表示数据来自UWB
	{
		_sta=2;
		_rx_buf[1]=data;
	}
	else if(_sta==2)		//数据目的地
	{
		_sta=3;
		_rx_buf[2]=data;
	}
	else if(_sta==3)		//功能字
	{
		_sta=4;
		_rx_buf[3]=data;
	}
	else if(_sta==4)		//数据长度
	{
		_sta = 5;
		_rx_buf[4]=data;
		_data_len = data;
		_rx_buf_len = 5;
	}
	else if(_sta==5&&_data_len>0)
	{
		_data_len--;
		_rx_buf[_rx_buf_len++]=data;
		if(_data_len==0)
			_sta = 6;
	}
	else if(_sta==6)
	{
		_sta = 0;
		_rx_buf[_rx_buf_len]=data;
		if(!UWB_Data_OK)
		{
			for(u8 i=0; i<=_rx_buf_len; i++)
				UWB_RxBuffer[i] = _rx_buf[i];
			UWB_data_len = _rx_buf_len+1;
			UWB_Data_OK = 1;

		}
	}
	else
		_sta = 0;
}

/**********************************************************************************************************
*函 数 名: Ano_UWB_Get_Data_Task
*功能说明: UWB数据获取任务
*参    数: 周期（毫秒）
*返 回 值: 无
**********************************************************************************************************/
static u16 uwb_check_time;
void Ano_UWB_Get_Data_Task(u8 dT_ms)
{
	if(UWB_Data_OK)
	{
		UWB_Data_OK = 0;
		u8 sum = 0;
		for(u8 i=0;i<(UWB_data_len-1);i++)
			sum += *(UWB_RxBuffer+i);
		if(!(sum==*(UWB_RxBuffer+UWB_data_len-1)))		return;		//判断sum
		
		if(*(UWB_RxBuffer+3)==0X31)			//距离信息
		{
			
		}
		else if(*(UWB_RxBuffer+3)==0X32)			//位置信息
		{
			uwb_data.raw_data_loc[1] = -(float)(s16)((*(UWB_RxBuffer+6)<<8)|*(UWB_RxBuffer+7)) / 100;
			uwb_data.raw_data_loc[0] =  (float)(s16)((*(UWB_RxBuffer+8)<<8)|*(UWB_RxBuffer+9)) / 100;
			uwb_data.raw_data_loc[2] =  (float)(s16)((*(UWB_RxBuffer+10)<<8)|*(UWB_RxBuffer+11)) / 100;
			uwb_data.raw_data_vel[1] = -(float)(s16)((*(UWB_RxBuffer+12)<<8)|*(UWB_RxBuffer+13)) / 100;
			uwb_data.raw_data_vel[0] =  (float)(s16)((*(UWB_RxBuffer+14)<<8)|*(UWB_RxBuffer+15)) / 100;
			uwb_data.raw_data_vel[2] =  (float)(s16)((*(UWB_RxBuffer+16)<<8)|*(UWB_RxBuffer+17)) / 100;
		}
		//
		uwb_check_time = 0;
	}
	//
	if(uwb_check_time <1000)
	{
		uwb_check_time += dT_ms;
		uwb_data.online = 1;
	}
	else
	{
		uwb_data.online = 0;
	}
	
}

/**********************************************************************************************************
*函 数 名: Ano_UWB_Data_Calcu_Task
*功能说明: UWB数据计算任务
*参    数: 周期（毫秒）
*返 回 值: 无
**********************************************************************************************************/
void Ano_UWB_Data_Calcu_Task(u8 dT_ms)
{
	//解锁前，记录当前机体X轴正方向在水平面投影为UWB坐标X轴正方向
	//要求解锁前，飞机放正，机体X轴正方向对准UWB坐标轴X轴正方向
	//记录参考方向
	if(!fc_sta.unlock_sta)
	{
		//
		uwb_data.init_ok = 0;
		//
		uwb_data.ref_dir[X] = imu_data.hx_vec[X];
		uwb_data.ref_dir[Y] = imu_data.hx_vec[Y];

	}
	//
	else
	{
		//
		uwb_data.init_ok = 1;
	}
	//参考方向转世界坐标（这里等同于地理坐标）
	h2w_2d_trans(uwb_data.raw_data_loc,uwb_data.ref_dir,uwb_data.w_dis_cm);
	//计算速度
	h2w_2d_trans(uwb_data.raw_data_vel,uwb_data.ref_dir,uwb_data.w_vel_cmps);
	
}
