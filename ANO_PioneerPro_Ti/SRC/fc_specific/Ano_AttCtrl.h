/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ATT_CTRL_H
#define __ATT_CTRL_H
/* Includes ------------------------------------------------------------------*/
#include "Ano_FcData.h"
#include "Ano_Filter.h"
#include "Ano_Math.h"
#include "Ano_Pid.h"
/* Exported types ------------------------------------------------------------*/

enum
{
	ROLL_END=0,
	ROLL_UP,
	ROLL_ROLLING,
	ROLL_KEEP,
};

typedef struct
{
	u8 roll_mode;
	u8 rolling_step;
	u8 roll_thr_step;
	
	float rol_angle[2];
	
	s16 ref_height;
	s16 up_height;
	
	u16 keep_cnt;
	s16 roll_up_speed;
	u8 roll_height_ok;
//	s16 roll_acc_fix;
}_rolling_flag_st;
extern _rolling_flag_st rolling_flag;

typedef struct
{
	float set_yaw_speed;
	
	float exp_angular_velocity[VEC_RPY];

	float fb_angular_velocity[VEC_RPY];
}_att_1l_ct_st;
extern _att_1l_ct_st att_1l_ct;

typedef struct
{
	float yaw_err;
  float exp_rol_adj;
	float exp_pit_adj;
	
	float exp_rol,exp_pit,exp_yaw;
	float fb_rol,fb_pit,fb_yaw;

}_att_2l_ct_st;
extern _att_2l_ct_st att_2l_ct;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void Att_2level_PID_Init(void);
void Att_1level_PID_Init(void);
void Set_Att_1level_Ki(u8 mode);
void Set_Att_2level_Ki(u8 mode);

void Att_2level_Ctrl(float dT,s16 *CH_N);
void Att_1level_Ctrl(float dT);


extern _PID_arg_st arg_2[VEC_RPY] ; 

extern _PID_arg_st arg_1[VEC_RPY] ;

extern _PID_val_st val_2[VEC_RPY];

extern _PID_val_st val_1[VEC_RPY];

#endif
