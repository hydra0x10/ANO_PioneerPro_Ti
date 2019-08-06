#include "Ano_AttCtrl.h"
#include "Ano_Imu.h"
#include "Drv_icm20602.h"
#include "Ano_MagProcess.h"
#include "Drv_spl06.h"
#include "Ano_MotionCal.h"
#include "Ano_FlightCtrl.h"
#include "Ano_LocCtrl.h"
#include "Ano_MotorCtrl.h"
#include "Ano_FlyCtrl.h"
#include "Ano_Parameter.h"
#include "Ano_Sensor_Basic.h"
#include "Ano_ProgramCtrl_User.h"


//角度环控制参数
_PID_arg_st arg_2[VEC_RPY] ; 

//角速度环控制参数
_PID_arg_st arg_1[VEC_RPY] ;


//角度环控制数据
_PID_val_st val_2[VEC_RPY];

//角速度环控制数据
_PID_val_st val_1[VEC_RPY];

/*角度环PID参数初始化*/
void Att_2level_PID_Init()
{
	arg_2[ROL].kp = Ano_Parame.set.pid_att_2level[ROL][KP];
	arg_2[ROL].ki = Ano_Parame.set.pid_att_2level[ROL][KI];
	arg_2[ROL].kd_ex = Ano_Parame.set.pid_att_2level[ROL][KD];
	arg_2[ROL].kd_fb = Ano_Parame.set.pid_att_2level[ROL][KD];
	arg_2[ROL].k_ff = 0.0f;
	
	arg_2[PIT].kp = Ano_Parame.set.pid_att_2level[PIT][KP];
	arg_2[PIT].ki = Ano_Parame.set.pid_att_2level[PIT][KI];
	arg_2[PIT].kd_ex = Ano_Parame.set.pid_att_2level[PIT][KD];
	arg_2[PIT].kd_fb = Ano_Parame.set.pid_att_2level[PIT][KD];
	arg_2[PIT].k_ff = 0.0f;

	arg_2[YAW].kp = Ano_Parame.set.pid_att_2level[YAW][KP];
	arg_2[YAW].ki = Ano_Parame.set.pid_att_2level[YAW][KI];
	arg_2[YAW].kd_ex = Ano_Parame.set.pid_att_2level[YAW][KD];
	arg_2[YAW].kd_fb = Ano_Parame.set.pid_att_2level[YAW][KD];
	arg_2[YAW].k_ff = 0.0f;		
}


/*
姿态角速率部分控制参数

arg_1_kp：调整角速度响应速度，不震荡的前提下，尽量越高越好。

震荡试，可以降低arg_1_kp，增大arg_1_kd。

若增大arg_1_kd已经不能抑制震荡，需要将kp和kd同时减小。
*/
#define CTRL_1_KI_START 0.f

/*角速度环PID参数初始化*/
void Att_1level_PID_Init()
{
	arg_1[ROL].kp = Ano_Parame.set.pid_att_1level[ROL][KP];
	arg_1[ROL].ki = Ano_Parame.set.pid_att_1level[ROL][KI];
	arg_1[ROL].kd_ex = 0;//0.000f   ;
	arg_1[ROL].kd_fb = Ano_Parame.set.pid_att_1level[ROL][KD];
	arg_1[ROL].k_ff = 0.0f;
	
	arg_1[PIT].kp = Ano_Parame.set.pid_att_1level[PIT][KP];
	arg_1[PIT].ki = Ano_Parame.set.pid_att_1level[PIT][KI];
	arg_1[PIT].kd_ex = 0;//0.000f   ;
	arg_1[PIT].kd_fb = Ano_Parame.set.pid_att_1level[PIT][KD];
	arg_1[PIT].k_ff = 0.0f;

	arg_1[YAW].kp = Ano_Parame.set.pid_att_1level[YAW][KP];
	arg_1[YAW].ki = Ano_Parame.set.pid_att_1level[YAW][KI];
	arg_1[YAW].kd_ex = 0;//0.00f   ;
	arg_1[YAW].kd_fb = Ano_Parame.set.pid_att_1level[YAW][KD];
	arg_1[YAW].k_ff = 0.00f;	
	
#if (MOTOR_ESC_TYPE == 2)
	#define DIFF_GAIN 0.3f
//	arg_1[ROL].kd_ex = arg_1[ROL].kd_ex *DIFF_GAIN;
//	arg_1[PIT].kd_ex = arg_1[PIT].kd_ex *DIFF_GAIN;
	arg_1[ROL].kd_fb = arg_1[ROL].kd_fb *DIFF_GAIN;
	arg_1[PIT].kd_fb = arg_1[PIT].kd_fb *DIFF_GAIN;
#elif (MOTOR_ESC_TYPE == 1)
	#define DIFF_GAIN 1.0f
//	arg_1[ROL].kd_ex = arg_1[ROL].kd_ex *DIFF_GAIN;
//	arg_1[PIT].kd_ex = arg_1[PIT].kd_ex *DIFF_GAIN;
	arg_1[ROL].kd_fb = arg_1[ROL].kd_fb *DIFF_GAIN;
	arg_1[PIT].kd_fb = arg_1[PIT].kd_fb *DIFF_GAIN;
#endif
}

void Set_Att_1level_Ki(u8 mode)
{
	if(mode == 0)
	{
		arg_1[ROL].ki = arg_1[PIT].ki = 0;
	}
	else if(mode == 1)
	{
		arg_1[ROL].ki = Ano_Parame.set.pid_att_1level[ROL][KI];
		arg_1[PIT].ki = Ano_Parame.set.pid_att_1level[PIT][KI];
	}
	else 
	{
		arg_1[ROL].ki = arg_1[PIT].ki = CTRL_1_KI_START;
	}
}

void Set_Att_2level_Ki(u8 mode)
{
	if(mode == 0)
	{
		arg_2[ROL].ki = arg_2[PIT].ki = 0;
	}
	else
	{
		arg_2[ROL].ki = Ano_Parame.set.pid_att_2level[ROL][KI];
		arg_2[PIT].ki = Ano_Parame.set.pid_att_2level[PIT][KI];
	}
}


_att_2l_ct_st att_2l_ct;

static s32 max_yaw_speed,set_yaw_av_tmp;

#define POS_V_DAMPING 0.02f
static float exp_rol_tmp,exp_pit_tmp;
	
/*角度环控制*/
void Att_2level_Ctrl(float dT_s,s16 *CH_N)
{
	/*积分微调*/
	exp_rol_tmp = - loc_ctrl_1.out[Y];
	exp_pit_tmp = - loc_ctrl_1.out[X];
	
	if(flag.flight_mode == ATT_STAB)
	{
		if(ABS(exp_rol_tmp + att_2l_ct.exp_rol_adj) < 5)
		{
			att_2l_ct.exp_rol_adj += 0.2f *exp_rol_tmp *dT_s;
			att_2l_ct.exp_rol_adj = LIMIT(att_2l_ct.exp_rol_adj,-1,1);
		}
		
		if(ABS(exp_pit_tmp + att_2l_ct.exp_pit_adj) < 5)
		{
			att_2l_ct.exp_pit_adj += 0.2f *exp_pit_tmp *dT_s;
			att_2l_ct.exp_pit_adj = LIMIT(att_2l_ct.exp_pit_adj,-1,1);
		}
	}
	else
	{
		att_2l_ct.exp_rol_adj = 
		att_2l_ct.exp_pit_adj = 0;
	}
	
	/*正负参考ANO坐标参考方向*/
	att_2l_ct.exp_rol = exp_rol_tmp + att_2l_ct.exp_rol_adj;// + POS_V_DAMPING *imu_data.h_acc[Y];
	att_2l_ct.exp_pit = exp_pit_tmp + att_2l_ct.exp_pit_adj;// + POS_V_DAMPING *imu_data.h_acc[X];
	
	/*期望角度限幅*/
	att_2l_ct.exp_rol = LIMIT(att_2l_ct.exp_rol,-MAX_ANGLE,MAX_ANGLE);
	att_2l_ct.exp_pit = LIMIT(att_2l_ct.exp_pit,-MAX_ANGLE,MAX_ANGLE);
	

	//////////////////////////////////////////////////////////////
	if(flag.speed_mode == 3)
	{
		max_yaw_speed = MAX_SPEED_YAW;
	}
	else if(flag.speed_mode == 2 )
	{
		max_yaw_speed = 220;
	}
	else 
	{
		max_yaw_speed = 200;
	}
	//
	fc_stv.yaw_pal_limit = max_yaw_speed;
	/*摇杆量转换为YAW期望角速度 + 程控期望角速度*/
	set_yaw_av_tmp = (s32)(0.0023f *my_deadzone(CH_N[CH_YAW],0,65) *max_yaw_speed) + (-program_ctrl.yaw_pal_dps) + pc_user.pal_dps_set;

	/*最大YAW角速度限幅*/
	set_yaw_av_tmp = LIMIT(set_yaw_av_tmp ,-max_yaw_speed,max_yaw_speed);
	
	/*没有起飞，复位*/
	if(flag.taking_off==0)//if(flag.locking)
	{
		att_2l_ct.exp_rol = att_2l_ct.exp_pit = set_yaw_av_tmp = 0;
		att_2l_ct.exp_yaw = att_2l_ct.fb_yaw;
	}
	/*限制误差增大*/
	if(att_2l_ct.yaw_err>90)
	{
		if(set_yaw_av_tmp>0)
		{
			set_yaw_av_tmp = 0;
		}
	}
	else if(att_2l_ct.yaw_err<-90)
	{
		if(set_yaw_av_tmp<0)
		{
			set_yaw_av_tmp = 0;
		}
	}	

	//增量限幅
	att_1l_ct.set_yaw_speed += LIMIT((set_yaw_av_tmp - att_1l_ct.set_yaw_speed),-30,30);
	/*设置期望YAW角度*/
	att_2l_ct.exp_yaw += att_1l_ct.set_yaw_speed *dT_s;
	/*限制为+-180度*/
	if(att_2l_ct.exp_yaw<-180) att_2l_ct.exp_yaw += 360;
	else if(att_2l_ct.exp_yaw>180) att_2l_ct.exp_yaw -= 360;	
	
	/*计算YAW角度误差*/
	att_2l_ct.yaw_err = (att_2l_ct.exp_yaw - att_2l_ct.fb_yaw);
	/*限制为+-180度*/
	if(att_2l_ct.yaw_err<-180) att_2l_ct.yaw_err += 360;
	else if(att_2l_ct.yaw_err>180) att_2l_ct.yaw_err -= 360;
	


		/*赋值反馈角度值*/
		att_2l_ct.fb_yaw = imu_data.yaw ;
			
		att_2l_ct.fb_rol = (imu_data.rol ) ;
		att_2l_ct.fb_pit = (imu_data.pit ) ;
			
	
	PID_calculate( dT_s,            //周期（单位：秒）
										0 ,				//前馈值
										att_2l_ct.exp_rol ,				//期望值（设定值）
										att_2l_ct.fb_rol ,			//反馈值（）
										&arg_2[ROL], //PID参数结构体
										&val_2[ROL],	//PID数据结构体
	                  5,//积分误差限幅
										5 *flag.taking_off			//integration limit，积分限幅
										 )	;
										
	PID_calculate( dT_s,            //周期（单位：秒）
										0 ,				//前馈值
										att_2l_ct.exp_pit ,				//期望值（设定值）
										att_2l_ct.fb_pit ,			//反馈值（）
										&arg_2[PIT], //PID参数结构体
										&val_2[PIT],	//PID数据结构体
	                  5,//积分误差限幅
										5 *flag.taking_off		//integration limit，积分限幅
										 )	;
	
	PID_calculate( dT_s,            //周期（单位：秒）
										0 ,				//前馈值
										att_2l_ct.yaw_err ,				//期望值（设定值）
										0 ,			//反馈值（）
										&arg_2[YAW], //PID参数结构体
										&val_2[YAW],	//PID数据结构体
	                  5,//积分误差限幅
										5 *flag.taking_off			//integration limit，积分限幅
										 )	;

}

_att_1l_ct_st att_1l_ct;
static float ct_val[4];
/*角速度环控制*/
void Att_1level_Ctrl(float dT_s)
{
	////////////////改变控制参数任务（最小控制周期内）////////////////////////
	ctrl_parameter_change_task();
	

		/*目标角速度赋值*/
		 for(u8 i = 0;i<3;i++)
		{
			att_1l_ct.exp_angular_velocity[i] = val_2[i].out;// val_2[i].out;//
		}
	
		/*目标角速度限幅*/
		att_1l_ct.exp_angular_velocity[ROL] = LIMIT(att_1l_ct.exp_angular_velocity[ROL],-MAX_ROLLING_SPEED,MAX_ROLLING_SPEED);
		att_1l_ct.exp_angular_velocity[PIT] = LIMIT(att_1l_ct.exp_angular_velocity[PIT],-MAX_ROLLING_SPEED,MAX_ROLLING_SPEED);


	/*反馈角速度赋值*/
	att_1l_ct.fb_angular_velocity[ROL] = ( sensor.Gyro_deg[X] );
	att_1l_ct.fb_angular_velocity[PIT] = (-sensor.Gyro_deg[Y] );
	att_1l_ct.fb_angular_velocity[YAW] = (-sensor.Gyro_deg[Z] );
	

	/*PID计算*/									 
 for(u8 i = 0;i<3;i++)
 {
		PID_calculate( dT_s,            //周期（单位：秒）
										0,				//前馈值
										att_1l_ct.exp_angular_velocity[i],				//期望值（设定值）
										att_1l_ct.fb_angular_velocity[i],			//反馈值（）
										&arg_1[i], //PID参数结构体
										&val_1[i],	//PID数据结构体
                    200,//积分误差限幅
										CTRL_1_INTE_LIM *flag.taking_off			//integration limit，积分幅度限幅
										 )	; 
 
	
	 ct_val[i] = (val_1[i].out);
 }
										 

	/*赋值，最终比例调节*/
	mc.ct_val_rol =                   FINAL_P *ct_val[ROL];
	mc.ct_val_pit = X_PROPORTION_X_Y *FINAL_P *ct_val[PIT];
	mc.ct_val_yaw =                   FINAL_P *ct_val[YAW];
	/*输出量限幅*/
	mc.ct_val_rol = LIMIT(mc.ct_val_rol,-1000,1000);
	mc.ct_val_pit = LIMIT(mc.ct_val_pit,-1000,1000);
	mc.ct_val_yaw = LIMIT(mc.ct_val_yaw,-400,400);	
}

_rolling_flag_st rolling_flag;

