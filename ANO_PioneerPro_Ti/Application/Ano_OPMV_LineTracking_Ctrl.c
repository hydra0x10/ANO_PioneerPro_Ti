/*==========================================================================
 * ����    ����OPMV���ص����ݽ��д�����������������ת�����Ѱ��Ŀ��
             λ�ñ仯����ϣ�Ҳ��������ת�������ת������������ϸ߶ȵõ�
						 ����ƫ��ý���ĵ���ƫ����������ٶȣ�������ƫ�
						 
 
 * ����ʱ�䣺2019-07-20 
 * ����		 �������ƴ�-Jyoun
 * ����    ��www.anotc.com
 * �Ա�    ��anotc.taobao.com
 * ����QȺ ��190169595
 * ��Ŀ������18084888982��18061373080
============================================================================
 * �����ƴ��ŶӸ�л��ҵ�֧�֣���ӭ��ҽ�Ⱥ���ཻ�������ۡ�ѧϰ��
 * �������������в��õĵط�����ӭ����ש�������
 * �������������ã�����������Ƽ���֧�����ǡ�
 * ������Դ������뻶ӭ�������á��������չ��������ϣ������ʹ��ʱ��ע��������
 * ����̹������С�˳����ݣ��������������ˮ���������ӣ�Ҳ��δ�й�Ĩ��ͬ�е���Ϊ��  
 * ��Դ���ף�����������ף�ϣ����һ������ء����ﻥ������ͬ������
 * ֻ������֧�֣������������ø��á�  
===========================================================================*/

//Ĭ������
#include "Ano_OPMV_LineTracking_Ctrl.h"
#include "Drv_OpenMV.h"
#include "Ano_OPMV_Ctrl.h"
#include "Ano_Math.h"
#include "Ano_Filter.h"

//
//���ݽӿڶ��壺
//=========mapping===============
//��Ҫ���õ��ļ���
#include "ANO_IMU.h"
#include "Ano_FlightCtrl.h"

//��Ҫ�������õ��ⲿ������
#define IMU_ROL                 (imu_data.rol)     //�����
#define IMU_PIT                 (imu_data.pit)     //������
#define RELATIVE_HEIGHT_CM           (jsdata.valid_of_alt_cm)  //��Ը߶�


#define LT_KP                  (0.80f)  //������
#define LT_KD                  (0.05f)  //΢����


//��Ҫ������ֵ���ⲿ������


//===============================
//ȫ�ֱ�����
_ano_opmv_lt_ctrl_st ano_opmv_lt_ctrl;
static u16 line_loss_hold_time;
static float lt_decou_pos_pixel_lpf[2];
static u8 step_pro_sta;
//�����趨��
#define LT_PIXELPDEG    2.4f  //ÿ1�Ƕȶ�Ӧ�����ظ�������ֱ��ʺͽ����йأ���Ҫ���Ա궨��//3.2f->(160*120,3.6mm)  2.4f->(160*120,2.8mm)
#define LT_CMPPIXEL     0.01f     //ÿ���ض�Ӧ�ĵ�����룬�뽹��͸߶��йأ���Ҫ���Ա궨��//Ŀǰ���Ա궨
#define LLH_TIME        1000   //�ж�Ŀ�궪ʧ�ı���ʱ�䡣
#define CONFIRM_TIMES     10     //��ȷ�������жϵĴ�����ȷ�ϴ���Խ�࣬Խ�ܱ������⣬����Ҳ���ܵ��¼�ⲻ����
#define YAW_PAL_DPS       90    //������ٶȣ���ÿ��
#define FORWARD_VEL       50    //ǰ���ٶ�

/**********************************************************************************************************
*�� �� ��: ANO_LTracking_Task
*����˵��: �����ƴ�Ѱ������
*��    ��: ����ʱ��(ms)
*�� �� ֵ: ��
**********************************************************************************************************/
void ANO_LTracking_Task(u8 dT_ms)
{
	//�������Ƶ������������Լ��޸�
	if(opmv.mode_sta==2)// && opmv_ct_sta.en)
	{
		//Ѱ��������ת�����
		ANO_LTracking_Decoupling(&dT_ms,IMU_ROL,IMU_PIT);
		//Ѱ�����ݼ���
		ANO_LTracking_Calcu(&dT_ms,(s32)RELATIVE_HEIGHT_CM);
	}
	else
	{
		//reset
		ano_opmv_lt_ctrl.target_loss = 1;
	}
}

/**********************************************************************************************************
*�� �� ��: ANO_LTracking_Decoupling
*����˵��: �����ƴ�Ѱ�߽����
*��    ��: ����ʱ��(�β�ms)������Ƕȣ������Ƕ�
*�� �� ֵ: ��
**********************************************************************************************************/
static void ANO_LTracking_Decoupling(u8 *dT_ms,float rol_degs,float pit_degs)
{
	float dT_s = (*dT_ms) *1e-3f;

	//��ʶ��Ŀ��
	if(opmv.lt.sta != 0)//(opmv.cb.color_flag!=0)
	{
		//
		ano_opmv_lt_ctrl.target_loss = 0;
		line_loss_hold_time = 0;
	}
	else
	{
		//�ӳ�һ��ʱ��
		if(line_loss_hold_time<LLH_TIME)
		{
			line_loss_hold_time += *dT_ms;
		}
		else
		{
			//Ŀ�궪ʧ�����λ
			ano_opmv_lt_ctrl.target_loss = 1;
		}
	}
	//
	ano_opmv_lt_ctrl.opmv_pos = -opmv.lt.deviation;

	//
	if(opmv.lt.sta != 0)
	{
		//������̬����Ӧ��ƫ����
		ano_opmv_lt_ctrl.r2pixel_val = -LT_PIXELPDEG *rol_degs;
		ano_opmv_lt_ctrl.r2pixel_val = LIMIT(ano_opmv_lt_ctrl.r2pixel_val,-80,80);//���160pixel

	}
	else
	{
		//��̬����Ӧƫ�������ֲ���

	}

	//
	if(ano_opmv_lt_ctrl.target_loss==0) //��Ч��û��ʧ
	{
		//�õ�ƽ��ƫ����������ͨ�˲�
		lt_decou_pos_pixel_lpf[0] += 0.2f *((ano_opmv_lt_ctrl.opmv_pos - ano_opmv_lt_ctrl.r2pixel_val) - lt_decou_pos_pixel_lpf[0]);
		//����һ��
		lt_decou_pos_pixel_lpf[1] += 0.2f *(lt_decou_pos_pixel_lpf[0] - lt_decou_pos_pixel_lpf[1]);

		//��ֵ
		ano_opmv_lt_ctrl.decou_pos_pixel = lt_decou_pos_pixel_lpf[1];

	}
	else //��ʧĿ��
	{

		//��ͨ��λ��0
		LPF_1_(0.2f,dT_s,0,ano_opmv_lt_ctrl.decou_pos_pixel);


	}
	//

}

/**********************************************************************************************************
*�� �� ��: ANO_LTracking_Calcu
*����˵��: �����ƴ�Ѱ�߼��㴦��
*��    ��: ����ʱ��(�β�ms)����Ը߶�
*�� �� ֵ: ��
**********************************************************************************************************/
static void ANO_LTracking_Calcu(u8 *dT_ms,s32 relative_height_cm)
{
	static float relative_height_cm_valid;
	static float lt_gnd_pos_err_old;
	//��Ը߶ȸ�ֵ
	if(relative_height_cm<500)
	{
		relative_height_cm_valid = relative_height_cm;
	}
	else
	{
		//null
	}
	//��¼��ʷֵ
	lt_gnd_pos_err_old = ano_opmv_lt_ctrl.ground_pos_err_h_cm;

	//�õ�����ƫ���λ����
	ano_opmv_lt_ctrl.ground_pos_err_h_cm = LT_CMPPIXEL *relative_height_cm_valid *ano_opmv_lt_ctrl.decou_pos_pixel;

	//����΢��ƫ���λ����ÿ��
	ano_opmv_lt_ctrl.ground_pos_err_d_h_cmps = (ano_opmv_lt_ctrl.ground_pos_err_h_cm - lt_gnd_pos_err_old)*(1000/(*dT_ms));


}

/**********************************************************************************************************
*�� �� ��: ANO_LT_StepProcedure
*����˵��: �����ƴ�Ѱ��ѭ���ֲ���������
*��    ��: ����ʱ��(ms,�β�)
*�� �� ֵ: ��
**********************************************************************************************************/

void ANO_LT_StepProcedure(u8 *dT_ms)
{
	static u8 confirm_cnt;
	static u16 elapsed_time_ms;
	switch(step_pro_sta)
	{
		case 0:
		{
			//reset
			ano_opmv_lt_ctrl.exp_yaw_pal_dps = 0;
			ano_opmv_lt_ctrl.exp_velocity_h_cmps[0]=0;
			ano_opmv_lt_ctrl.exp_velocity_h_cmps[1]=0;
			elapsed_time_ms = 0;
			step_pro_sta = 1;
		}
		break;
		case 1:
		{
			//��λȷ�ϴ���
			confirm_cnt = 0;
			//ʶ��ֱ��
			if(opmv.lt.sta == 1)
			{
				//
				ano_opmv_lt_ctrl.exp_velocity_h_cmps[0] = FORWARD_VEL;
			}
			//ʶ��ֱ����ת
			else if(opmv.lt.sta == 2)
			{
				step_pro_sta = 2;
			}
			//ʶ��ֱ����ת
			else if(opmv.lt.sta == 3)
			{
				step_pro_sta = 3;
			}
			else if(opmv.lt.sta == 0)
			{
				step_pro_sta = 0;
			}
				
		}
		break;
		case 2:
		{
			//ȷ���Ƿ�ʶ�𵽣����򷵻�
			if(opmv.lt.sta != 2)
			{
				step_pro_sta = 1;

			}
			else
			{
				//��ȷ��n��
				if(confirm_cnt<CONFIRM_TIMES)
				{
					confirm_cnt++;
				}
				else
				{
					confirm_cnt = 0;
					//��һ��
					step_pro_sta = 4;
				}		
			}
		}
		break;
		case 3:
		{
			//ȷ���Ƿ�ʶ�𵽣����򷵻�
			if(opmv.lt.sta != 3)
			{
				step_pro_sta = 1;

			}
			else
			{
				//��ȷ��n��
				if(confirm_cnt<CONFIRM_TIMES)
				{
					confirm_cnt++;
				}
				else
				{
					confirm_cnt = 0;
					//��һ��
					step_pro_sta = 5;
				}		
			}		
		}
		break;
		case 4:
		{
			//ɲ������
			ano_opmv_lt_ctrl.exp_velocity_h_cmps[0] = 10;//ʶ��ֱ�Ǻ����ǰ�����ٶ�cm/s
			ano_opmv_lt_ctrl.exp_velocity_h_cmps[1] = 0;//���Ҿ����ٶȸ�λ
			//����ʱ�䣬ת���ƫ�ڲ⣬��ӳ�ʱ��;��֮����ʱ��
			if(elapsed_time_ms<1500)
			{
				elapsed_time_ms+= *dT_ms;
			}
			else
			{
				elapsed_time_ms = 0;
//				if(opmv.lt.sta == 2)
				step_pro_sta = 12;
			}		
		}
		break;
		case 5:
		{
			//ɲ������
			ano_opmv_lt_ctrl.exp_velocity_h_cmps[0] = 10;//ʶ��ֱ�Ǻ����ǰ�����ٶ�cm/s
			ano_opmv_lt_ctrl.exp_velocity_h_cmps[1] = 0;//���Ҿ����ٶȸ�λ
			//����ʱ�䣬ת���ƫ�ڲ⣬��ӳ�ʱ��;��֮����ʱ��
			if(elapsed_time_ms<1500)
			{
				elapsed_time_ms+= *dT_ms;
			}
			else
			{
				elapsed_time_ms = 0;
//				if(opmv.lt.sta == 3)
				step_pro_sta = 13;
			}			
		}
		break;
		case 12://��ת90��
		{
			//
			ano_opmv_lt_ctrl.exp_velocity_h_cmps[0] = 30;//����ת�߸�ǰ���ٶȣ��߳�һ��Բ��
			ano_opmv_lt_ctrl.exp_velocity_h_cmps[1] = 0;//���Ҿ����ٶȸ�λ
			ano_opmv_lt_ctrl.exp_yaw_pal_dps = -YAW_PAL_DPS;
			//
			if(elapsed_time_ms<90000/YAW_PAL_DPS)
			{
				elapsed_time_ms += *dT_ms;
			}
			else
			{
				ano_opmv_lt_ctrl.exp_yaw_pal_dps = 0;
				elapsed_time_ms = 0;
				step_pro_sta = 20;
			}
		}
		break;
		case 13://��ת90��
		{
			//
			ano_opmv_lt_ctrl.exp_velocity_h_cmps[0] = 30;//����ת�߸�ǰ���ٶȣ��߳�һ��Բ��
			ano_opmv_lt_ctrl.exp_velocity_h_cmps[1] = 0;//���Ҿ����ٶȸ�λ
			ano_opmv_lt_ctrl.exp_yaw_pal_dps = YAW_PAL_DPS;
			//
			if(elapsed_time_ms<90000/YAW_PAL_DPS)
			{
				elapsed_time_ms += *dT_ms;
			}
			else
			{
				ano_opmv_lt_ctrl.exp_yaw_pal_dps = 0;
				elapsed_time_ms = 0;
				step_pro_sta = 20;
			}			
		}
		break;		
		case 20://�νӣ�����ת���п���û�п���ֱ�ߣ��������趨ǰ��500ms
		{
			//
			elapsed_time_ms+= *dT_ms;
			//
			if(elapsed_time_ms<100)
			{
				elapsed_time_ms+= *dT_ms;
			}
			else if(elapsed_time_ms<600)
			{
				elapsed_time_ms+= *dT_ms;
				ano_opmv_lt_ctrl.exp_velocity_h_cmps[0] = FORWARD_VEL;
				ano_opmv_lt_ctrl.exp_velocity_h_cmps[1] = 0;
			}
			else
			{
				elapsed_time_ms = 0;
				step_pro_sta = 1;			
			}
		}
		break;
		default:
		{
			
		}
		break;
	}
	


}

/**********************************************************************************************************
*�� �� ��: ANO_LTracking_Ctrl
*����˵��: �����ƴ�ɫ����ٿ�������
*��    ��: ����ʱ��(ms)
*�� �� ֵ: ��
**********************************************************************************************************/
void ANO_LTracking_Ctrl(u8 *dT_ms,u8 en)
{
	//��������
	if(en)
	{
		//����ƫ��PD����
		ano_opmv_lt_ctrl.exp_velocity_h_cmps[1]\
		= LT_KP *ano_opmv_lt_ctrl.ground_pos_err_h_cm\
		+ LT_KD *ano_opmv_lt_ctrl.ground_pos_err_d_h_cmps;

		//ת����������
		if(opmv.lt.sta == 1)
		{
			//
			ano_opmv_lt_ctrl.exp_yaw_pal_dps = -opmv.lt.angle *6;
			ano_opmv_lt_ctrl.exp_yaw_pal_dps = LIMIT(ano_opmv_lt_ctrl.exp_yaw_pal_dps,-100,100);
		}
		else
		{
			ano_opmv_lt_ctrl.exp_yaw_pal_dps = 0;
		}
		//ǰ����ת�����
		ANO_LT_StepProcedure(dT_ms);
	}
	else
	{
		//
		ano_opmv_lt_ctrl.exp_velocity_h_cmps[0] = 0;		
		ano_opmv_lt_ctrl.exp_velocity_h_cmps[1] = 0;
		ano_opmv_lt_ctrl.exp_yaw_pal_dps = 0;
		//
		step_pro_sta = 0;
	}
	//
	
}	
