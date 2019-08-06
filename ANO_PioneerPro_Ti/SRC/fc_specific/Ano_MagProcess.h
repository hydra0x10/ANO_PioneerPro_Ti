/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ECOMPASS_H
#define __ECOMPASS_H
/* Includes ------------------------------------------------------------------*/
#include "Ano_FcData.h"
#include "Ano_Filter.h"
#include "Ano_Math.h"
/* Exported types ------------------------------------------------------------*/

typedef struct
{
	u8 mag_CALIBRATE;
//	s16 offset[VEC_XYZ];
//	float gain[VEC_XYZ];
	s16 val[VEC_XYZ];

}_mag_cal_st;
extern _mag_cal_st mag;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void Mag_Data_Deal_Task(u8 dT_ms,s16 mag_in[],float z_vec_z,float gyro_deg_x,float gyro_deg_z);
#endif

