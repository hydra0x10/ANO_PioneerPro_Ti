#ifndef __ANO_FILTER_H
#define __ANO_FILTER_H
#include "Ano_FcData.h"

typedef struct
{
	float out;
	float last_out;
	float a;
	float b;
	float a_c2;
	float b_c2;
	float c_c2;

	float k;
} _ano_filter_1_st;

//#define LPF_1_(hz,t,in,out) ((out) += ( 1 / ( 1 + 1 / ( (hz) *3.14f *(t) ) ) ) *( (in) - (out) ))
typedef struct
{
	float in_est;    //Estimator
	float in_obs;    //Observation
	
	float fix_ki;
	float ei_limit;     //


/////	
	float e;
	float ei;

	float out;
}_inte_fix_filter_st;

typedef struct
{
	float in_est_d;   //Estimator
	float in_obs;    //Observation
	
	float fix_kp;
	float e_limit;

/////	
	float e;

	float out;
}_fix_inte_filter_st;





typedef struct
{
	float lpf_1;

	float out;
}_lf_t;

typedef struct
{
	float lpf_1;
	float lpf_2;
	float in_old;
	float out;
}_jldf_t;

typedef struct
{
	float a;//q
	float b;

  s16 limit;
	float e_nr;

	float err_v;
	float out;
	float ei;
	float out_f;
} _filter_1_st;

typedef struct
{
	u8 cnt;

	s32 lst_pow_sum;
	
	s32 now_out;
	s32 lst_out;
	s32 now_velocity_xdt;
} _steepest_st;

 typedef struct Filter
{
	float           _cutoff_freq; 
	float           _a1;
	float           _a2;
	float           _b0;
	float           _b1;
	float           _b2;
	float           _delay_element_1;        // buffered sample -1
	float           _delay_element_2;        // buffered sample -2

}filter_s;



void inte_fix_filter(float dT,_inte_fix_filter_st *data);
void fix_inte_filter(float dT,_fix_inte_filter_st *data);

void filter_1(float ,float ,float ,float ,_filter_1_st *);

void limit_filter(float T,float hz,_lf_t *data,float in);
void limit_filter_2(float T,float hz,_lf_t *data,float in);
void limit_filter_3(float T,float hz,_lf_t *data,float in);


void init_low_pass2_fliter(float sample_freq, float cutoff_freq,filter_s* imu_filter);
float low_pass2_filter(float sample,filter_s* imu_filter);


void steepest_descend(s32 arr[],u8 len,_steepest_st *steepest,u8 step_num,s32 in);
	
void filter_1(float ,float ,float ,float ,_filter_1_st *);

void jyoun_limit_deadzone_filter(float T,float hz1,float hz2,_jldf_t *data,float in);//白噪声滤波

void jyoun_filter(float dT,float hz,float ref_value,float exp,float fb,float *out);
//float Moving_Average(u8 item,u8 width_num,float in);
void Moving_Average(float moavarray[],//滤波数组 数组长度：len+1
										u16 len ,//滤波数据长度
										u16 *fil_cnt,//滤波元素号数标记（静态，用作存储）
										float in,//输入
										float *out //输出
										);



void step_filter(float step,float in,float *out);

void fir_arrange_filter(float *arr,u16 len,u8 *fil_cnt,float in,float *arr_out);  //len<=255 len >= 3

#define LPF_1_(hz,t,in,out) ((out) += ( 1 / ( 1 + 1 / ( (hz) *6.28f *(t) ) ) ) *( (in) - (out) ))	
#define S_LPF_1(a,in,out) ((out) += (a) *( (in) - (out) ))	//a == 1 / ( 1 + 1 / ( (hz) *6.28f *(t) ) ) 
										
//void LPF_1(float hz,//截止频率
//					float time,//周期
//					float in,//输入
//					float *out//输出
//					);

					
void LPF_1_db(float hz,float time,double in,double *out); //低通滤波，2hz代表0.5秒上升至目标值0.7倍，大约1秒上升到90%

void LPF_I(float raw_a,float raw_b,float time,float in,float *out,float *intera);



float my_deadzone_3(float T,float hz,float x,float ,float zoom,float range_x,float *zoom_adj); //range_x   0 ----- 1  *****


/*============ 坐标转换 ===============
适用坐标系
					x
					|
			y---z
			
对应世界坐标中，x为地磁方向，z为重力方向。

======================================*/
void vec_3dh_transition(float ref[VEC_XYZ], float in[VEC_XYZ], float out[VEC_XYZ]);
void vec_3dh_transition_matrix(float ref[VEC_XYZ],float wh_matrix[VEC_XYZ][VEC_XYZ]);

#endif
