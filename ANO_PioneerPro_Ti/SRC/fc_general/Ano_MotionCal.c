#include "Ano_MotionCal.h"
#include "Ano_Math.h"
#include "Drv_icm20602.h"
#include "Ano_Imu.h"
#include "Ano_MotorCtrl.h"

#define BARO_SPEED_ERR_LIM   50               //对比融合速度

_inte_fix_filter_st wcz_acc_fus;
_fix_inte_filter_st wcz_spe_fus,wcz_hei_fus;

s32 ref_height_old,ref_speed_old;

s32 wcz_ref_height,wcz_ref_speed,wcz_ref_acc;

//static float wcz_acc_deadzone;	


static s32 wcz_acc;
#define N_TIMES 5

void WCZ_Data_Calc(u8 dT_ms,u8 wcz_f_pause,s32 wcz_acc_get,s32 ref_height)
{
	static u8 cyc_xn;
	float hz,ntimes_hz;	
	hz = safe_div(1000,dT_ms,0);
	ntimes_hz = hz/N_TIMES;
	
	wcz_ref_height = ref_height;
	wcz_acc = wcz_acc_get;
/////////////////////////////////////////////////////////////	
	//wcz_acc_deadzone = 0;//LIMIT(5 *(0.996f - imu_data.z_vec[Z] *imu_data.z_vec[Z]),0,1) *10;
	
//	roll_acc_fix = (ABS(sensor.Gyro_deg[X]) + ABS(sensor.Gyro_deg[Y]) - 20) *0.1f;
//	roll_acc_fix = LIMIT(roll_acc_fix,0,5);
//	
//	wz_acc_comp = roll_acc_fix ;//- LIMIT(5 *(0.996f - imu_data.z_vec[Z] *imu_data.z_vec[Z]),0,1) *5 ;
	
	
	cyc_xn ++;
	cyc_xn %= N_TIMES;
	
	if(cyc_xn == 0)
	{
		wcz_ref_speed = (wcz_ref_height - ref_height_old) *ntimes_hz;
		
		wcz_ref_acc = (wcz_ref_speed - ref_speed_old) *ntimes_hz;
		
		ref_height_old = wcz_ref_height;	
		ref_speed_old = wcz_ref_speed;
	
	}
	////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////
	
	wcz_acc_fus.fix_ki = 0.1f;
	wcz_acc_fus.in_est = wcz_acc;
	wcz_acc_fus.in_obs = wcz_ref_acc;
	wcz_acc_fus.ei_limit = 100;
	inte_fix_filter(dT_ms*1e-3f,&wcz_acc_fus);

	
	wcz_spe_fus.fix_kp = 0.6f;
	wcz_spe_fus.in_est_d = wcz_acc_fus.out;
	wcz_spe_fus.in_obs = wcz_ref_speed;
	wcz_spe_fus.e_limit = 100;
	fix_inte_filter(dT_ms*1e-3f,&wcz_spe_fus);
	
	
	wcz_hei_fus.fix_kp = 0.3f;
	wcz_hei_fus.in_est_d = wcz_spe_fus.out;
	wcz_hei_fus.in_obs = ref_height;
	//wcz_hei_fus.e_limit = 200;
	fix_inte_filter(dT_ms*1e-3f,&wcz_hei_fus);
	
	

///////////////////////////////////////////////////////////////
	

	
}


void WCZ_Data_Reset()
{
	wcz_acc_fus.out = 0;
	wcz_acc_fus.ei = -wcz_acc;
	
	wcz_spe_fus.out = 0;
	wcz_spe_fus.e = 0;

	wcz_hei_fus.out = 0;
	wcz_hei_fus.e = 0;	

}
