#include "Ano_MagProcess.h"
#include "Ano_LED.h"

static s16 max_t[VEC_XYZ];
static s16 min_t[VEC_XYZ];


_mag_cal_st mag;

static void Mag_Cal_Reset(u8 mode)
{
	if(mode == 2)
	{
		for(u8 i = 0;i<2;i++)
		{
			max_t[i] = -30000;
			min_t[i] = 30000;

		}
	}
	else if(mode == 1)
	{
			max_t[Z] = -30000;
			min_t[Z] = 30000;		
	}
	else
	{
		for(u8 i = 0;i<3;i++)
		{
			max_t[i] = -30000;
			min_t[i] = 30000;

		}	
	}
}

static void Mag_Cal_XY(s16 mag_in[])
{
	for(u8 i = 0;i<2;i++)
	{
		max_t[i] = _MAX(max_t[i],mag_in[i]);
		min_t[i] = _MIN(min_t[i],mag_in[i]);
	}
	

	
}

static void Mag_Cal_Z(s16 mag_in[])
{
	max_t[Z] = _MAX(max_t[Z],mag_in[Z]);
	min_t[Z] = _MIN(min_t[Z],mag_in[Z]);
}

static u8 mag_cal_step;

void Mag_Data_Deal_Task(u8 dT_ms,s16 mag_in[],float z_vec_z,float gyro_deg_x,float gyro_deg_z)
{	
	static u16 cali_cnt;
	static float mag_cal_angle[2];
	float t_length;
	
	for(u8 i = 0;i<3;i++)
	{
		save.mag_gain[i] = LIMIT(save.mag_gain[i],0.05f,100);
		mag.val[i] = (mag_in[i] - save.mag_offset[i]) *save.mag_gain[i];
	}

///////////////////cali//////////////////////////////////////////////////////	
	if(mag.mag_CALIBRATE!= 0 && flag.unlock_sta == 0)
	{	
		switch(mag_cal_step)
		{
			case 0://第一步，水平旋转

				Mag_Cal_XY(mag_in);			
			
				if(z_vec_z<0.985f)//+-10deg	
				{
					LED_STA.calMag = 100;
					mag_cal_step = 1;
					mag_cal_angle[0] = 0;
				}
				else
				{	
					LED_STA.calMag = 1;
					mag_cal_angle[0] += dT_ms *1e-3f *(gyro_deg_z); //角度积分，旋转360度
					if(ABS(mag_cal_angle[0])>360)
					{
						mag_cal_angle[0] = 0;
						mag_cal_step = 2;
					}
				}
			break;
			
			case 1://error
				
				Mag_Cal_Reset(2);
				mag_cal_angle[0] = 0;
				mag_cal_step = 0;
			break;
			
			case 2://第二步，竖直旋转，机头朝下
				LED_STA.calMag = 2;
				if(z_vec_z<0.1f)//5.7deg
				{
					mag_cal_step = 3;
				}
			break;
			
			case 3:
				mag.mag_CALIBRATE = 2;																					
				
				Mag_Cal_Z(mag_in);

				if(z_vec_z>0.17f)//10deg
				{
					LED_STA.calMag = 2;
					mag_cal_step = 4;
					mag_cal_angle[1] = 0;
				}
				else
				{
					LED_STA.calMag = 3;
					mag_cal_angle[1] += dT_ms *1e-3f *(gyro_deg_x);	//角度积分，旋转360度
					if(ABS(mag_cal_angle[1])>360)
					{
						mag_cal_angle[1] = 0;
						mag_cal_step = 5;
					}
				}			
			break;
			
			case 4://error_2，重新开始竖直旋转
				Mag_Cal_Reset(1);
				mag_cal_angle[1] = 0;
				mag_cal_step = 2;				
			break;
			
			case 5:
				for(u8 i = 0;i<3;i++)
				{
					save.mag_offset[i] = 0.5f *(max_t[i] + min_t[i]);		//中值校准
					save.mag_gain[i] = safe_div(200.0f ,(0.5f *(max_t[i] - min_t[i])),0);		//幅值校准
				}
				
				Mag_Cal_Reset(3);		
				mag_cal_angle[0] = mag_cal_angle[1] = 0;		
				mag_cal_step = 0;
				mag.mag_CALIBRATE = 0;			
				LED_STA.calMag = 0;
				
				data_save();//保存数据
			break;
			
			default:break;	
		}
		
		
		if(mag_cal_step == 0 || mag_cal_step == 3)
		{
			//长时间出错，退出校准逻辑
			if(cali_cnt<15000)
			{
				cali_cnt+= dT_ms;
				
			}
			else////校准错误
			{
				LED_STA.errOneTime = 1;
				cali_cnt = 0;
				LED_STA.calMag = 0;				
				mag.mag_CALIBRATE = 0;

			}
		}
		else
		{
			cali_cnt = 0;
		}
	}
	else
	{
														//if(LED_state ==4 || LED_state ==5) LED_state = 0;
		
		mag_cal_step = 0;
		
//////////////////////////////////////////////	
		t_length = my_3_norm(mag.val[X],mag.val[Y],mag.val[Z]);
		
		if(t_length<150||t_length>350)
		{
			//state[3] |= (1<<3);//罗盘严重干扰
																											//LED_state = 6;
		}
		else
		{
			//state[3] &= ~(1<<3);//罗盘无严重干扰
			
//			if(LED_state == 6)
//			{
//				LED_state = 0;
//			}
		}
///////////////////////////////////////////////	
	}
}
