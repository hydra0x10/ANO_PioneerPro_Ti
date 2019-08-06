
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTOR_CTRL_H
#define __MOTOR_CTRL_H

/* Includes ------------------------------------------------------------------*/
#include "Ano_FcData.h"
#include "Ano_Pid.h"
/* Exported types ------------------------------------------------------------*/

typedef struct
{
	s32 ct_val_rol;
	s32 ct_val_pit;
	s32 ct_val_yaw; 
	s32 ct_val_thr;
} _mc_st;
extern _mc_st mc;





/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void Motor_Ctrl_Task(u8 dT_ms);

extern s16 motor[MOTORSNUM];
#endif

