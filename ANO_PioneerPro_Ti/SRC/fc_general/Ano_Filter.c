/******************** (C) COPYRIGHT 2016 ANO Tech ***************************
 * 作者		 ：匿名科创
 * 文件名  ：ANO_filter.c
 * 描述    ：滤波函数
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
*****************************************************************************/
#include "Ano_Filter.h"
#include "Ano_Math.h"



void inte_fix_filter(float dT,_inte_fix_filter_st *data)
{
	float ei_lim_val;
	
	if(data->ei_limit>0)
	{		
		ei_lim_val = LIMIT(data->ei,-data->ei_limit,data->ei_limit);
	}
	else
	{
		ei_lim_val = data->ei;
	}	
	
	data->out = (data->in_est + ei_lim_val);
	
	data->e = data->fix_ki *(data->in_obs - data->out);

	data->ei += data->e *dT;


}

void fix_inte_filter(float dT,_fix_inte_filter_st *data)
{
	
	data->out += (data->in_est_d + data->e ) *dT;
	
	data->e = data->fix_kp *(data->in_obs - data->out);

	if(data->e_limit>0)
	{		
		data->e = LIMIT(data->e,-data->e_limit,data->e_limit);
	}
	
	
}



void limit_filter(float T,float hz,_lf_t *data,float in) //增量滤波，适合大噪声低滞后（无缝），收敛最快
{
	float abs_t;
// 	LPF_1(hz,T,	 in,&(data->lpf_1)); 
// 	abs_t = ABS(data->lpf_1);
// 	data->out = LIMIT(in,-abs_t,abs_t);
	LPF_1_(hz,T,	 in,(data->lpf_1)); 
 	abs_t = ABS(in);
	data->out = LIMIT((data->lpf_1),-abs_t,abs_t);
}
void limit_filter_2(float T,float hz,_lf_t *data,float in) //增量滤波，均衡，但滞后稍大(数据反向等待低通反向)
{
	float abs_t;
	LPF_1_(hz,T,	 in,(data->lpf_1)); 
	abs_t = ABS(data->lpf_1);
	data->out = LIMIT(in,-abs_t,abs_t);
// 	LPF_1(hz,T,	 in,&(data->lpf_1)); 
//  	abs_t = ABS(in);
// 	data->out = LIMIT((data->lpf_1),-abs_t,abs_t);
}

void limit_filter_3(float T,float hz,_lf_t *data,float in) //增量滤波，适合低噪声滞后较小(等幅大噪声平行不收敛),噪声大收敛过慢
{
	float abs_t;
	LPF_1_(hz,T,	 in,(data->lpf_1)); 
	abs_t = ABS(in);
	data->out = data->lpf_1 = LIMIT((data->lpf_1),-abs_t,abs_t);

}

// #define STEEPEST_ARR_NUM 10
// #define STEEPEST_STEP 10  //次


void steepest_descend(s32 arr[],u8 len,_steepest_st *steepest,u8 step_num,s32 in)
{	
	u8 updw = 1;//0 dw,1up
	s16 i;
	u8 step_cnt=0;
	u8 step_slope_factor=1;
	u8 on = 1;
	s8 pn = 1;
	//float last = 0;
	float step = 0;
	s32 start_point = 0;
	s32 pow_sum = 0;
	
	steepest->lst_out = steepest->now_out;
	
	if( ++(steepest->cnt) >= len )	
	{
		(steepest->cnt) = 0; //now
	}
	
	//last = arr[ (steepest->cnt) ];
	
	arr[ (steepest->cnt) ] = in;
	
	step = (float)(in - steepest->lst_out)/step_num ;//梯度
	
	if(ABS(step)<1)//整形数据<1的有效判定
	{
		if(ABS(step)*step_num<2)
		{
			step = 0;
		}
		else
		{
		  step = (step > 0) ? 1 : -1;
		}
	}
	
	start_point = steepest->lst_out;
	do
	{
		//start_point = steepest->lst_out;
		for(i=0;i<len;i++)
		{
// 			j = steepest->cnt + i + 1;
// 			if( j >= len )	
// 			{
// 				j = j - len; //顺序排列
// 			}
			pow_sum += my_pow(arr[i] - start_point );// /step_num;//除法减小比例**
			
			//start_point += pn *(step_slope_factor *step/len);
		}
			
		if(pow_sum - steepest->lst_pow_sum > 0)
		{		
			if(updw==0)
			{
				on = 0;
			}
			updw = 1;//上升了
			pn = (pn == 1 )? -1:1;

		}
		else
		{
			updw = 0; //正在下降
 			if(step_slope_factor<step_num)
 			{
 				step_slope_factor++;
 			}
		}
			
		steepest->lst_pow_sum = pow_sum;		
		pow_sum = 0;
		start_point += pn *step;//调整
		
		if(++step_cnt > step_num)//限制计算次数
		{
			on = 0;
		}
			//////
			if(step_slope_factor>=2)//限制下降次数1次，节省时间，但会增大滞后，若cpu时间充裕可不用。
			{
				on = 0;

			}
			//////
		
	}
	while(on==1);
	
	steepest->now_out = start_point ;//0.5f *(start_point + steepest->lst_out);//
	
	steepest->now_velocity_xdt = steepest->now_out - steepest->lst_out;
}





void fir_arrange_filter(float *arr,u16 len,u8 *fil_cnt,float in,float *arr_out) //len<=255 len >= 3
{
	//float arrange[len];
	float tmp;
	u8 i,j;
/*
窗口数据处理
*/		
	if( ++*fil_cnt >= len )	
	{
		*fil_cnt = 0; //now
	}
	
	arr[ *fil_cnt ] = in;
/*
窗口数据处理
*/	
	
/*
赋值、排列
*/	
	for(i=0;i<len;i++)
	{
		arr_out[i] = arr[i];
	}
	
	for(i=0;i<len-1;i++)
	{
		for(j=0;j<len-1-i;j++)
		{
			if(arr_out[j]>arr_out[j+1])
			{
				tmp = arr_out[j+1];
				arr_out[j+1] = arr_out[j];
				arr_out[j] = tmp;
			}
		}
	}
/*
赋值、排列
*/		
	

}

// #define WIDTH_NUM 101
// #define FIL_ITEM  10

 void Moving_Average(float moavarray[],u16 len ,u16 *fil_cnt,float in,float *out)
{
	u16 width_num;
	float last;

	width_num = len ;
	
	if( ++*fil_cnt >= width_num )	
	{
		*fil_cnt = 0; //now
	}
	
	last = moavarray[ *fil_cnt ];
	
	moavarray[ *fil_cnt ] = in;
	
	*out += ( in - ( last  ) )/(float)( width_num ) ;
	//*out += 0.00001f *(in - *out);
	*out += 0.00001f *LIMIT((in - *out),-1,1);  //数据精度误差修正
	
}

void LPF_1(float hz,float time,float in,float *out)  
{
	*out += ( 1 / ( 1 + 1 / ( hz *6.28f *time ) ) ) *( in - *out );

}

void LPF_1_db(float hz,float time,double in,double *out)
{
	*out += ( 1 / ( 1 + 1 / ( hz *6.28f *time ) ) ) *( in - *out );

}


void step_filter(float step,float in,float *out) 
{
	if(in - *out > step)
	{
		*out += step;
	}
	else if(in - *out < -step)
	{
		*out -= step;
	}
	else
	{
		*out = in;
	}

}

float my_hpf_limited(float T,float hz,float x,float zoom,float *zoom_adj)
{

	
	*zoom_adj += ( 1 / ( 1 + 1 / ( hz *6.28f *T ) ) ) *(x - *zoom_adj);
	*zoom_adj = LIMIT(*zoom_adj,-zoom,zoom);
	return (x - *zoom_adj);

}


void simple_3d_trans(float ref[VEC_XYZ], float in[VEC_XYZ], float out[VEC_XYZ]) //该函数只有在水平面附近一个有限的范围内正确。
{
	static s8 pn;
	static float h_tmp_x,h_tmp_y;
	
	h_tmp_x = my_sqrt(my_pow(ref[Z]) + my_pow(ref[Y]));
	h_tmp_y = my_sqrt(my_pow(ref[Z]) + my_pow(ref[X]));
	
	pn = ref[Z] < 0? -1 : 1;
	
	  out[X] = ( h_tmp_x *in[X] - pn *ref[X] *in[Z] ) ;
		out[Y] = ( pn *h_tmp_y *in[Y] - ref[Y] *in[Z] ) ;
	
// 	 out[X] = h_tmp_x *in[X] - ref[X] *in[Z];
// 	 out[Y] = ref[Z] *in[Y] - ref[Y] *in[Z];
	
	out[Z] = ref[X] *in[X] + ref[Y] *in[Y] + ref[Z] *in[Z] ;

}

void vec_3dh_transition(float ref[VEC_XYZ], float in[VEC_XYZ], float out[VEC_XYZ])
{
	simple_3d_trans(ref,in,out); //
}

void vec_3dh_transition_matrix(float ref[VEC_XYZ],float wh_matrix[VEC_XYZ][VEC_XYZ])
{

}


//#define M_PI_F 3.14159f
//#include "math.h"
//void init_low_pass2_fliter(float sample_freq, float cutoff_freq,filter_s* imu_filter)
//{
//	imu_filter->_cutoff_freq = cutoff_freq;
//	
//	float fr = sample_freq/imu_filter->_cutoff_freq;
//	float ohm = tanf(M_PI_F/fr);
//	float c = 1.0f+2.0f*cosf(M_PI_F/4.0f)*ohm + ohm*ohm;
//	imu_filter->_b0 = ohm*ohm/c;
//	imu_filter->_b1 = 2.0f*imu_filter->_b0;
//	imu_filter->_b2 = imu_filter->_b0;
//	imu_filter->_a1 = 2.0f*(ohm*ohm-1.0f)/c;
//	imu_filter->_a2 = (1.0f-2.0f*cosf(M_PI_F/4.0f)*ohm+ohm*ohm)/c;
//	imu_filter->_delay_element_1 = 0.0;
//	imu_filter->_delay_element_2 = 0.0;
//}


//float low_pass2_filter(float sample,filter_s* imu_filter)//低通二阶滤波器
//{
//	float delay_element_0 = sample - imu_filter->_delay_element_1 * imu_filter->_a1 - imu_filter->_delay_element_2 * imu_filter->_a2;
//	float output = delay_element_0 * imu_filter->_b0 + imu_filter->_delay_element_1 * imu_filter->_b1 + imu_filter->_delay_element_2 * imu_filter->_b2;

//	imu_filter->_delay_element_2 = imu_filter->_delay_element_1;
//	imu_filter->_delay_element_1 = delay_element_0;

//	// return the value.  Should be no need to check limits
//	return output;
//}




/******************* (C) COPYRIGHT 2016 ANO TECH *****END OF FILE************/
