/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __NAVIGATE_H
#define __NAVIGATE_H
/* Includes ------------------------------------------------------------------*/
#include "Ano_FcData.h"
#include "Ano_Filter.h"
/* Exported types ------------------------------------------------------------*/

typedef struct
{
	float ref_acc_w[VEC_XYZ];
	float ref_speed_w_pre[VEC_XYZ];
	float ref_speed_w_pre_old[VEC_XYZ];
	
  float ref_speed_w[VEC_XYZ];
	float ref_speed_w_old[VEC_XYZ];
	float ref_speed_h[VEC_XYZ];
	
	_inte_fix_filter_st acc_if[VEC_XYZ];
	
	_fix_inte_filter_st speed_fi[VEC_XYZ];
	_inte_fix_filter_st speed_if[VEC_XYZ];
	
	float acc_out_w[VEC_XYZ];
	float speed_out_w_pre[VEC_XYZ];
	float speed_out_w[VEC_XYZ];
	float speed_out_h[VEC_XYZ];
} _w_speed_fus_st;
extern _w_speed_fus_st w_speed_fus;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void wxy_speed_fusing(float dT,u8 est_en,u8 fix_en);
void wxy_fus_task(float dT);
#endif

