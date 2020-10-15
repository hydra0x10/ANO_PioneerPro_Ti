#include "Ano_FlightCtrl.h"
#include "Ano_Imu.h"
#include "Drv_icm20602.h"
#include "Ano_MagProcess.h"
#include "Drv_spl06.h"
#include "Ano_MotionCal.h"
#include "Ano_AttCtrl.h"
#include "Ano_LocCtrl.h"
#include "Ano_AltCtrl.h"
#include "Ano_MotorCtrl.h"
#include "Drv_led.h"
#include "Ano_RC.h"
#include "Drv_laser.h"
#include "Ano_OF.h"
#include "Ano_OF_DecoFusion.h"
#include "Ano_FlyCtrl.h"
#include "Ano_UWB.h"
#include "Ano_Sensor_Basic.h"
#include "Ano_DT.h"
#include "Ano_LED.h"
#include "Ano_ProgramCtrl_User.h"
#include "Drv_OpenMV.h"


/*============================================================================
更新：
201908012059-Jyoun：修正因高度失效判定光流失效的条件bug，以更好兼容超声波。



===========================================================================*/
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////

/*PID参数初始化*/
void All_PID_Init(void)
{

	/*姿态控制，角速度PID初始化*/
	Att_1level_PID_Init();
	
	/*姿态控制，角度PID初始化*/
	Att_2level_PID_Init();
	
	/*高度控制，高度速度PID初始化*/
	Alt_1level_PID_Init();	
	
	/*高度控制，高度PID初始化*/
	Alt_2level_PID_Init();
	
	
	/*位置速度控制PID初始化*/
	Loc_1level_PID_Init();
	
}

/*控制参数改变任务*/
void ctrl_parameter_change_task()
{

	
	if(0)
	{
		Set_Att_2level_Ki(0);
		
	}
	else
	{
		if(flag.auto_take_off_land ==AUTO_TAKE_OFF)
		{

			Set_Att_1level_Ki(2);
		}
		else
		{

			Set_Att_1level_Ki(1);
		}
		
		Set_Att_2level_Ki(1);
	}
}


/*一键翻滚（暂无）*/
void one_key_roll()
{

			if(flag.flying && flag.auto_take_off_land == AUTO_TAKE_OFF_FINISH)
			{	
				if(rolling_flag.roll_mode==0)
				{
					rolling_flag.roll_mode = 1;
					
				}
			}

}

static u16 one_key_taof_start;
/*一键起飞任务（主要功能为延迟）*/
void one_key_take_off_task(u16 dt_ms)
{
	if(one_key_taof_start != 0)
	{
		one_key_taof_start += dt_ms;
		
		
		if(one_key_taof_start > 1400 && flag.motor_preparation == 1)
		{
			one_key_taof_start = 0;
				if(flag.auto_take_off_land == AUTO_TAKE_OFF_NULL)
				{
					flag.auto_take_off_land = AUTO_TAKE_OFF;
					//解锁、起飞

					flag.taking_off = 1;
				}
			
		}
	}
	//reset
	if(flag.unlock_sta == 0)
	{
		one_key_taof_start = 0;
	}

}
/*一键起飞*/
void one_key_take_off()
{
	if(flag.unlock_err == 0)
	{	
		if(flag.auto_take_off_land == AUTO_TAKE_OFF_NULL && one_key_taof_start == 0)
		{
			one_key_taof_start = 1;
			flag.unlock_cmd = 1;
		}
	}
}
/*一键降落*/
void one_key_land()
{
	flag.auto_take_off_land = AUTO_LAND;
}

//////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////
_flight_state_st fs;

s16 flying_cnt,landing_cnt;

extern s32 ref_height_get;

float stop_baro_hpf;

/*降落检测*/

static s16 ld_delay_cnt ;
void land_discriminat(s16 dT_ms)
{
//	static s16 acc_delta,acc_old;
	
//	acc_delta = imu_data.w_acc[Z]- acc_old;
//	acc_old = imu_data.w_acc[Z];
	
	/*油门归一值小于0.1  或者启动自动降落*/
	if((fs.speed_set_h_norm[Z] < 0.1f) || flag.auto_take_off_land == AUTO_LAND)
	{
		if(ld_delay_cnt>0)
		{
			ld_delay_cnt -= dT_ms;
		}
	}
	else
	{
		ld_delay_cnt = 200;
	}
	
	/*意义是：如果向上推了油门，就需要等垂直方向加速度小于200cm/s2 保持200ms才开始检测*/	
	if(ld_delay_cnt <= 0 && (flag.thr_low || flag.auto_take_off_land == AUTO_LAND) )
	{
		/*油门最终输出量小于250并且没有在手动解锁上锁过程中，持续1秒，认为着陆，然后上锁*/
		if(mc.ct_val_thr<250 && flag.unlock_sta == 1 && flag.locking != 2)//ABS(wz_spe_f1.out <20 ) //还应当 与上速度条件，速度小于正20厘米每秒。
		{
			if(landing_cnt<1500)
			{
				landing_cnt += dT_ms;
			}
			else
			{

				flying_cnt = 0;
				flag.taking_off = 0;
				///////
					landing_cnt =0;	
					flag.unlock_cmd =0;				

				flag.flying = 0;

			}
		}
		else
		{
			landing_cnt = 0;
		}
			
		
	}
	else
	{
		landing_cnt  = 0;
	}

}


/*飞行状态任务*/

void Flight_State_Task(u8 dT_ms,s16 *CH_N)
{
	s16 thr_deadzone;
	static float max_speed_lim,vel_z_tmp[2];
	/*设置油门摇杆量*/
	thr_deadzone = (flag.wifi_ch_en != 0) ? 0 : 50;
	fs.speed_set_h_norm[Z] = my_deadzone(CH_N[CH_THR],0,thr_deadzone) *0.0023f;
	fs.speed_set_h_norm_lpf[Z] += 0.5f *(fs.speed_set_h_norm[Z] - fs.speed_set_h_norm_lpf[Z]);
	
	/*推油门起飞*/
	if(flag.unlock_sta)
	{	
		if(fs.speed_set_h_norm[Z]>0.01f && flag.motor_preparation == 1) // 0-1
		{
			flag.taking_off = 1;
		}	
	}		
	//
	fc_stv.vel_limit_z_p = MAX_Z_SPEED_UP;
	fc_stv.vel_limit_z_n = -MAX_Z_SPEED_DW;	
	//
	if( flag.taking_off )
	{
			
		if(flying_cnt<1000)//800ms
		{
			flying_cnt += dT_ms;
		}
		else
		{
			/*起飞后1秒，认为已经在飞行*/
			flag.flying = 1;  
		}
		
		if(fs.speed_set_h_norm[Z]>0)
		{
			/*设置上升速度*/
			vel_z_tmp[0] = (fs.speed_set_h_norm_lpf[Z] *MAX_Z_SPEED_UP);
		}
		else
		{
			/*设置下降速度*/
			vel_z_tmp[0] = (fs.speed_set_h_norm_lpf[Z] *MAX_Z_SPEED_DW);
		}

		//飞控系统Z速度目标量综合设定
		vel_z_tmp[1] = vel_z_tmp[0] + program_ctrl.vel_cmps_h[Z] + pc_user.vel_cmps_set_z;
		//
		vel_z_tmp[1] = LIMIT(vel_z_tmp[1],fc_stv.vel_limit_z_n,fc_stv.vel_limit_z_p);
		//
		fs.speed_set_h[Z] += LIMIT((vel_z_tmp[1] - fs.speed_set_h[Z]),-0.8f,0.8f);//限制增量幅度
	}
	else
	{
		fs.speed_set_h[Z] = 0 ;
	}
	float speed_set_tmp[2];
	/*速度设定量，正负参考ANO坐标参考方向*/
	fs.speed_set_h_norm[X] = (my_deadzone(+CH_N[CH_PIT],0,50) *0.0022f);
	fs.speed_set_h_norm[Y] = (my_deadzone(-CH_N[CH_ROL],0,50) *0.0022f);
		
	LPF_1_(3.0f,dT_ms*1e-3f,fs.speed_set_h_norm[X],fs.speed_set_h_norm_lpf[X]);
	LPF_1_(3.0f,dT_ms*1e-3f,fs.speed_set_h_norm[Y],fs.speed_set_h_norm_lpf[Y]);
	
	max_speed_lim = MAX_SPEED;
	
	if(switchs.of_flow_on && !switchs.gps_on )
	{
		max_speed_lim = 1.5f *wcz_hei_fus.out;
		max_speed_lim = LIMIT(max_speed_lim,50,150);
	}	
	
	fc_stv.vel_limit_xy = max_speed_lim;
	
	//飞控系统XY速度目标量综合设定
	speed_set_tmp[X] = fc_stv.vel_limit_xy *fs.speed_set_h_norm_lpf[X] + program_ctrl.vel_cmps_h[X] + pc_user.vel_cmps_set_h[X];
	speed_set_tmp[Y] = fc_stv.vel_limit_xy *fs.speed_set_h_norm_lpf[Y] + program_ctrl.vel_cmps_h[Y] + pc_user.vel_cmps_set_h[Y];
	
	length_limit(&speed_set_tmp[X],&speed_set_tmp[Y],fc_stv.vel_limit_xy,fs.speed_set_h_cms);

	fs.speed_set_h[X] = fs.speed_set_h_cms[X];
	fs.speed_set_h[Y] = fs.speed_set_h_cms[Y];	
	
	/*调用检测着陆的函数*/
	land_discriminat(dT_ms);
	
	/*倾斜过大上锁*/
	if(rolling_flag.rolling_step == ROLL_END)
	{
		if(imu_data.z_vec[Z]<0.25f)//75度  ////////////////////////////////////////*************************** 倾斜过大上锁，慎用。
		{
			//
			if(mag.mag_CALIBRATE==0)
			{
				imu_state.G_reset = 1;
			}
			flag.unlock_cmd = 0;
		}

	}	
		//////////////////////////////////////////////////////////
	/*校准中，复位重力方向*/
	if(sensor.gyr_CALIBRATE != 0 || sensor.acc_CALIBRATE != 0 ||sensor.acc_z_auto_CALIBRATE)
	{
		imu_state.G_reset = 1;
	}
	
	/*复位重力方向时，认为传感器失效*/
	if(imu_state.G_reset == 1)
	{
		flag.sensor_imu_ok = 0;
		LED_STA.rst_imu = 1;
		WCZ_Data_Reset(); //复位高度数据融合
	}
	else if(imu_state.G_reset == 0)
	{	
		if(flag.sensor_imu_ok == 0)
		{
			flag.sensor_imu_ok = 1;
			LED_STA.rst_imu = 0;
			ANO_DT_SendString("IMU OK!");
		}
	}
	
	/*飞行状态复位*/
	if(flag.unlock_sta == 0)
	{
		flag.flying = 0;
		landing_cnt = 0;
		flag.taking_off = 0;
		flying_cnt = 0;
		
		
		flag.rc_loss_back_home = 0;
		
		//复位融合
		if(flag.taking_off == 0)
		{
//			wxyz_fusion_reset();
		}
	}
	

}

//
static u8 of_quality_ok;
static u16 of_quality_delay;
//
static u8 of_alt_ok;
static s16 of_alt_delay;
//
static u8 of_tof_on_tmp;
//

_judge_sync_data_st jsdata;
void Swtich_State_Task(u8 dT_ms)
{
	switchs.baro_on = 1;

	//光流模块
	if(sens_hd_check.of_ok || sens_hd_check.of_df_ok)
	{
		//
		if(sens_hd_check.of_ok)
		{
			jsdata.of_qua = OF_QUALITY;
			jsdata.of_alt = (u16)OF_ALT;
		}
		else if(sens_hd_check.of_df_ok)
		{
			jsdata.of_qua = of_rdf.quality;
			jsdata.of_alt = Laser_height_cm;
		}
		
		//
		if(jsdata.of_qua>50 )//|| flag.flying == 0) //光流质量大于50 /*或者在飞行之前*/，认为光流可用，判定可用延迟时间为1秒
		{
			if(of_quality_delay<500)
			{
				of_quality_delay += dT_ms;
			}
			else
			{
				of_quality_ok = 1;
			}
		}
		else
		{
			of_quality_delay =0;
			of_quality_ok = 0;
		}
		
		//光流高度600cm内有效
		if(jsdata.of_alt<600)
		{
			//		
			jsdata.valid_of_alt_cm = jsdata.of_alt;
			//延时1.5秒判断激光高度是否有效
			if(of_alt_delay<1000)
			{
				of_alt_delay += dT_ms;			
			}
			else
			{
				//判定高度有效
				of_alt_ok = 1;
				of_tof_on_tmp = 1;
			}
		}
		else
		{
			//
			if(of_alt_delay>0)
			{
				of_alt_delay -= dT_ms;	
			}
			else
			{
				//判定高度无效
				of_alt_ok = 0;
				of_tof_on_tmp = 0;
			}				
		}
		//
		
		
		//
		if(flag.flight_mode == LOC_HOLD)
		{		
			if(of_alt_ok && of_quality_ok)
			{
				switchs.of_flow_on = 1;
			}
			else
			{
				switchs.of_flow_on = 0;
			}

		}
		else
		{
			of_tof_on_tmp = 0;
			switchs.of_flow_on = 0;
		}	
		//
		switchs.of_tof_on = of_tof_on_tmp;
	}
	else
	{
		switchs.of_flow_on = switchs.of_tof_on = 0;
	}
	
	//激光模块
	if(sens_hd_check.tof_ok)
	{
		if(0)//(Laser_height_mm<1900)
		{
			switchs.tof_on = 1;
		}
		else
		{
			switchs.tof_on = 0;
		}
	}
	else
	{
		switchs.tof_on = 0;
	}
	
	//GPS	
	
	
	//UWB
	if(uwb_data.online && flag.flight_mode == LOC_HOLD)
	{
		switchs.uwb_on = 1;
	}
	else
	{
		switchs.uwb_on = 0;
	}
	
	
	//OPMV
	if(opmv.offline==0 && flag.flight_mode == LOC_HOLD)
	{
		switchs.opmv_on = 1;
	}
	else
	{
		switchs.opmv_on = 0;
	}
}


static void Speed_Mode_Switch()
{
//	if( ubx_user_data.s_acc_cms > 60)// || ubx_user_data.svs_used < 6)
//	{
//		flag.speed_mode = 0;
//	}
//	else
//	{
//		flag.speed_mode = 1;
//	}

}

u8 speed_mode_old = 255;
u8 flight_mode_old = 255;

void Flight_Mode_Set(u8 dT_ms)
{
	Speed_Mode_Switch();

	
	if(speed_mode_old != flag.speed_mode) //状态改变
	{
		speed_mode_old = flag.speed_mode;
		//xy_speed_pid_init(flag.speed_mode);////////////
	}

///////////////////////////////////////////////////////
	//CH_N[]+1500为上位机显示通道值
	if(CH_N[AUX1] <-100 && CH_N[AUX1]>-200)//接收机失控值，需要手工设置遥控器
	{
		//遥控设置的接收机输出的失控保护的信号。
		flag.chn_failsafe = 1;
	}
	else
	{
		flag.chn_failsafe = 0;
		////
		if(CH_N[AUX1]<-300)
		{
			flag.flight_mode = ATT_STAB;
		}
		else if(CH_N[AUX1]<200)
		{
			flag.flight_mode = LOC_HOLD;
		}
		else
		{
			flag.flight_mode = RETURN_HOME;
		}
	}
	//
	if(flight_mode_old != flag.flight_mode) //摇杆对应模式状态改变
	{
		flight_mode_old = flag.flight_mode;
		
		flag.rc_loss_back_home = 0;

	}
	//
	if(flag.rc_loss ==0)//接收机有信号
	{
		//CH_N[]+1500为上位机显示通道值
		if(CH_N[AUX2]<-300)//<1200
		{
			flag.flight_mode2 = 0;
		}
		else if(CH_N[AUX2]<200)//1200-1700
		{
			flag.flight_mode2 = 1;
		}
		else//>=1700
		{
			flag.flight_mode2 = 2;
		}
	}
	else
	{
		flag.flight_mode2 = 0;
	}
		
}

