/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ANO_FCDATA_H
#define __ANO_FCDATA_H
/* Includes ------------------------------------------------------------------*/
#include "config.h"
/* Exported types ------------------------------------------------------------*/
#define TRUE 1
#define FALSE 0 

enum
{
	AUTO_TAKE_OFF_NULL = 0,
	AUTO_TAKE_OFF = 1,
	AUTO_TAKE_OFF_FINISH,
	AUTO_LAND,
};

enum pwminmode_e
{
	PWM = 0,
	PPM,
	SBUS,
};

enum
{
 A_X = 0,
 A_Y ,
 A_Z ,
 G_X ,
 G_Y ,
 G_Z ,
 TEM ,
 MPU_ITEMS ,
};

	
enum
{
 CH_ROL = 0,
 CH_PIT ,
 CH_THR ,
 CH_YAW ,
 AUX1 ,
 AUX2 ,
 AUX3 ,
 AUX4 ,
 CH_NUM,//8
};

enum
{
	m1=0,
	m2,
	m3,
	m4,
	m5,
	m6,
	m7,
	m8,

};

enum
{
	MPU_6050_0 = 0,
	MPU_6050_1,
	
};

enum
{
	X = 0,
	Y = 1,
	Z = 2,
	VEC_XYZ,
};

enum
{
	ROL = 0,
	PIT = 1,
	YAW = 2,
	VEC_RPY,
};

enum
{
	KP = 0,
	KI = 1,
	KD = 2,
	PID,
};

enum _power_alarm
{

	HIGH_POWER = 0,
	HALF_POWER,
	LOW_POWER ,
	LOWEST_POWER, 
	

};


enum _flight_mode
{
	ATT_STAB = 0,//Attitude stabilization
	LOC_HOLD,
	RETURN_HOME,
	
};

//thr_mode
enum
{
  THR_MANUAL = 0,
	THR_AUTO,
	
};

typedef struct
{
	u8 first_f;
	float acc_offset[VEC_XYZ];
	float gyro_offset[VEC_XYZ];
	
	float surface_vec[VEC_XYZ];
	
	float mag_offset[VEC_XYZ];
	float mag_gain[VEC_XYZ];

} _save_st ;
extern _save_st save;

typedef struct
{
	//基本状态/传感器
	u8 start_ok;
	u8 sensor_imu_ok;
	u8 mems_temperature_ok;
	
	u8 motionless;
	u8 power_state;
	u8 wifi_ch_en;
	u8 chn_failsafe;
	u8 rc_loss;	
	u8 rc_loss_back_home;
	u8 gps_ok;	

	
	//控制状态
	u8 manual_locked;
	u8 unlock_err;
	u8 unlock_cmd;
	u8 unlock_sta;//unlocked
	u8 thr_low;
	u8 locking;
	u8 taking_off; //起飞
	u8 set_yaw;
	u8 ct_loc_hold;
	u8 ct_alt_hold;

	
	//飞行状态
	u8 flying;
	u8 auto_take_off_land;
	u8 home_location_ok;	
	u8 speed_mode;
	u8 thr_mode;	
	u8 flight_mode;
	u8 flight_mode2;
	u8 gps_mode_en;
	u8 motor_preparation;
	u8 locked_rotor;
	
	
}_flag;
extern _flag flag;

typedef struct
{
	float vel_limit_xy;
	float vel_limit_z_p;
	float vel_limit_z_n;
	float yaw_pal_limit;
}_fc_sta_var_st; //state variable
extern _fc_sta_var_st fc_stv;
	
typedef struct
{
	u8 sonar_on;
	u8 tof_on;
	u8 of_flow_on;
	u8 of_tof_on;
	u8 baro_on;
	u8 gps_on;
	u8 uwb_on;
	u8 opmv_on;
	
}_switch_st;
extern _switch_st switchs;

typedef struct
{
	u8 gyro_ok;
	u8 acc_ok;
	u8 mag_ok;
	u8 baro_ok;
	u8 gps_ok;
	u8 sonar_ok;
	u8 tof_ok;
	u8 of_ok;
	u8 of_df_ok;
	
} _sensor_hd_check_st; //Hardware
extern _sensor_hd_check_st sens_hd_check;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void data_save(void);
void Para_Data_Init(void);


#endif

