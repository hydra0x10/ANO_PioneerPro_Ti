#ifndef _DRV_GPS_H_
#define _DRV_GPS_H_

#include "sysconfig.h"

typedef struct{
	unsigned char satellite_num;
	unsigned char Location_stu;			//定位状态
	double latitude;						//纬度		
	double longitude;						//经度		
	int N_vel;							//南北向速度
	int E_vel;							//东西向速度
	
	float last_N_vel;							//上次南北向速度
	float last_E_vel;							//上次东西向速度
	
	float real_N_vel;
	float real_E_vel;
	
	unsigned char run_heart;			//运行心跳
	double start_latitude;					//起始纬度		
	double start_longitude;					//起始经度		
	int latitude_offset;
	int longitude_offset;
	float hope_latitude;
	float hope_longitude;
	float hope_latitude_err;
	float hope_longitude_err;
	unsigned char new_pos_get;
	unsigned char Back_home_f;
	float home_latitude;
	float home_longitude;
	
}GPS_INF;

extern GPS_INF Gps_information;
extern short Gps_send_Temp[10];
extern float wcx_acc_use;		
extern float wcy_acc_use;


void Drv_GpsGetOneByte(u8 data);
void Drv_GpsPin_Init(void);
void ANO_DT_Send_Gps_data(void);
void WCXY_Acc_Get_Task(void);
void GPS_Data_Processing_Task(u8 dT_ms);

#endif
