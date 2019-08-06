/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FLIGHT_CTRL_H
#define __FLIGHT_CTRL_H
/* Includes ------------------------------------------------------------------*/
#include "Ano_FcData.h"
#include "Ano_Filter.h"
#include "Ano_Math.h"

/* Exported types ------------------------------------------------------------*/

enum
{
   null=0,
   takeoff,
   landing,
	
    s_up_down_2, //小幅度上升下降2次
	
		s_yaw_pn_2,  //小幅度左右转2次
		b_yaw_pn_1,  //大幅度左右转1次
	
		s_rol_pn_2,  //小幅度左右2次
		b_rol_pn_1,  //大幅度左右1次
		
		s_pit_pn_2,  //小幅度前后2次
		b_pit_pn_1,  //大幅度前后1次
	
		yaw_n360,   //左转1圈
		yaw_p360,   //右转1圈
	
		roll_1,    //翻滚
	
		pit_jump_pn_2,  //前后跳2次
		rol_jump_pn_2,  //左右跳2次
		
		rol_up_down_2,  //蛇形上升下降	  
		yaw_up_dowm_1,  //旋转上升下降		2
		pit_rol_pn_2,   //蛇形前进后退		3
		
};

typedef struct
{
	s16 alt_ctrl_speed_set;
	float speed_set_h[VEC_XYZ];	
	float speed_set_h_cms[VEC_XYZ];
	
	float speed_set_h_norm[VEC_XYZ];
	float speed_set_h_norm_lpf[VEC_XYZ];
	
}_flight_state_st;
extern _flight_state_st fs;

typedef struct
{
	u8 of_qua;
	u16 of_alt;
	u16 valid_of_alt_cm;
	
}_judge_sync_data_st;
extern _judge_sync_data_st jsdata;

/* Exported constants --------------------------------------------------------*/
extern float wifi_selfie_mode_yaw_vlue;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void user_fun(float dT,u8 action_num);

void All_PID_Init(void);

void one_key_take_off(void);
void one_key_land(void);

void one_key_roll(void);
void app_one_key_roll(void);
void app_one_key_roll_reset(void);
void one_key_take_off_task(u16 dt_ms);

void ctrl_parameter_change_task(void);
	
void Flight_State_Task(u8,s16 *CH_N);

void Flight_Mode_Set(u8 dT_ms);

void Swtich_State_Task(u8 dT_ms);
#endif

