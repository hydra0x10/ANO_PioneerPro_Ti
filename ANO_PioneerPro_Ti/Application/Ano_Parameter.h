#ifndef _PARAMETER_H
#define	_PARAMETER_H

#include "config.h"
#include "Ano_FcData.h"

__packed struct Parameter_s
{
	u16 frist_init;	//飞控第一次初始化，需要做一些特殊工作，比如清空flash
	
	
	u8		pwmInMode;				//接收机模式，分别为PWM型PPM型
	u8		heatSwitch;				//

	float 	acc_offset[VEC_XYZ];  	//加速度计零偏
	float 	gyro_offset[VEC_XYZ]; 	//陀螺仪零偏
	
	float 	surface_vec[VEC_XYZ]; 	//水平面向量
	float 	center_pos_cm[VEC_XYZ]; //重心相对传感器位置偏移量
	
	float 	mag_offset[VEC_XYZ];  	//磁力计零偏
	float 	mag_gain[VEC_XYZ];    	//磁力计校正比例
	
	float 	pid_att_1level[VEC_RPY][PID]; //姿态控制角速度环PID参数
	float 	pid_att_2level[VEC_RPY][PID]; //姿态控制角度环PID参数
	float 	pid_alt_1level[PID];          //高度控制高度速度环PID参数
	float 	pid_alt_2level[PID];           //高度控制高度环PID参数
	float 	pid_loc_1level[PID];          //位置控制位置速度环PID参数
	float 	pid_loc_2level[PID];           //位置控制位置环PID参数

	float 	pid_gps_loc_1level[PID];          //位置控制位置速度环PID参数
	float 	pid_gps_loc_2level[PID];           //位置控制位置环PID参数

	float   warn_power_voltage;
	float	return_home_power_voltage;
	float   lowest_power_voltage;
	
	float	auto_take_off_height;
	float auto_take_off_speed;
	float auto_landing_speed;
	float idle_speed_pwm;
};

union Parameter
{
	//这里使用联合体，长度是4KByte，联合体内部是一个结构体，该结构体内是需要保存的参数
	struct Parameter_s set;
	u8 byte[2048];
};
extern union Parameter Ano_Parame;

typedef struct
{
	u8 save_en;
	u8 save_trig;
	u16 time_delay;
}_parameter_state_st ;
extern _parameter_state_st para_sta;

void Ano_Parame_Read(void);
void Ano_Parame_Write_task(u16 dT_ms);
void PID_Rest(void);
void Parame_Reset(void);

#endif 

