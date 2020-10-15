/*==========================================================================
 * 描述    ：对OPMV传回的数据进行处理，并解除因机体横滚旋转而造成寻线目标
             位置变化的耦合，也称作“旋转解耦”或“旋转补偿”。并耦合高度得到
						 地面偏差，用解算的地面偏差给到期望速度，以修正偏差。
						 
 
 * 更新时间：2019-07-20 
 * 作者		 ：匿名科创-Jyoun
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 * 项目合作：18084888982，18061373080
============================================================================
 * 匿名科创团队感谢大家的支持，欢迎大家进群互相交流、讨论、学习。
 * 若您觉得匿名有不好的地方，欢迎您拍砖提意见。
 * 若您觉得匿名好，请多多帮我们推荐，支持我们。
 * 匿名开源程序代码欢迎您的引用、延伸和拓展，不过在希望您在使用时能注明出处。
 * 君子坦荡荡，小人常戚戚，匿名坚决不会请水军、请喷子，也从未有过抹黑同行的行为。  
 * 开源不易，生活更不容易，希望大家互相尊重、互帮互助，共同进步。
 * 只有您的支持，匿名才能做得更好。  
===========================================================================*/

//默认引用
#include "Ano_OPMV_LineTracking_Ctrl.h"
#include "Drv_OpenMV.h"
#include "Ano_OPMV_Ctrl.h"
#include "Ano_Math.h"
#include "Ano_Filter.h"

//
//数据接口定义：
//=========mapping===============
//需要引用的文件：
#include "ANO_IMU.h"
#include "Ano_FlightCtrl.h"

//需要调用引用的外部变量：
#define IMU_ROL                 (imu_data.rol)     //横滚角
#define IMU_PIT                 (imu_data.pit)     //俯仰角
#define RELATIVE_HEIGHT_CM           (jsdata.valid_of_alt_cm)  //相对高度


#define LT_KP                  (0.80f)  //比例项
#define LT_KD                  (0.05f)  //微分项


//需要操作赋值的外部变量：


//===============================
//全局变量：
_ano_opmv_lt_ctrl_st ano_opmv_lt_ctrl;
static u16 line_loss_hold_time;
static float lt_decou_pos_pixel_lpf[2];
static u8 step_pro_sta;
//参数设定：
#define LT_PIXELPDEG    2.4f  //每1角度对应的像素个数，与分辨率和焦距有关，需要调试标定。//3.2f->(160*120,3.6mm)  2.4f->(160*120,2.8mm)
#define LT_CMPPIXEL     0.01f     //每像素对应的地面距离，与焦距和高度有关，需要调试标定。//目前粗略标定
#define LLH_TIME        1000   //判断目标丢失的保持时间。
#define CONFIRM_TIMES     10     //重确认特征判断的次数，确认次数越多，越能避免误检测，但是也可能导致检测不到。
#define YAW_PAL_DPS       90    //航向角速度，度每秒
#define FORWARD_VEL       50    //前进速度

/**********************************************************************************************************
*函 数 名: ANO_LTracking_Task
*功能说明: 匿名科创寻线任务
*参    数: 周期时间(ms)
*返 回 值: 无
**********************************************************************************************************/
void ANO_LTracking_Task(u8 dT_ms)
{
	//开启控制的条件，可以自己修改
	if(opmv.mode_sta==2)// && opmv_ct_sta.en)
	{
		//寻线数据旋转解耦合
		ANO_LTracking_Decoupling(&dT_ms,IMU_ROL,IMU_PIT);
		//寻线数据计算
		ANO_LTracking_Calcu(&dT_ms,(s32)RELATIVE_HEIGHT_CM);
	}
	else
	{
		//reset
		ano_opmv_lt_ctrl.target_loss = 1;
	}
}

/**********************************************************************************************************
*函 数 名: ANO_LTracking_Decoupling
*功能说明: 匿名科创寻线解耦合
*参    数: 周期时间(形参ms)，横滚角度，俯仰角度
*返 回 值: 无
**********************************************************************************************************/
static void ANO_LTracking_Decoupling(u8 *dT_ms,float rol_degs,float pit_degs)
{
	float dT_s = (*dT_ms) *1e-3f;

	//有识别到目标
	if(opmv.lt.sta != 0)//(opmv.cb.color_flag!=0)
	{
		//
		ano_opmv_lt_ctrl.target_loss = 0;
		line_loss_hold_time = 0;
	}
	else
	{
		//延迟一定时间
		if(line_loss_hold_time<LLH_TIME)
		{
			line_loss_hold_time += *dT_ms;
		}
		else
		{
			//目标丢失标记置位
			ano_opmv_lt_ctrl.target_loss = 1;
		}
	}
	//
	ano_opmv_lt_ctrl.opmv_pos = -opmv.lt.deviation;

	//
	if(opmv.lt.sta != 0)
	{
		//更新姿态量对应的偏移量
		ano_opmv_lt_ctrl.r2pixel_val = -LT_PIXELPDEG *rol_degs;
		ano_opmv_lt_ctrl.r2pixel_val = LIMIT(ano_opmv_lt_ctrl.r2pixel_val,-80,80);//宽度160pixel

	}
	else
	{
		//姿态量对应偏移量保持不变

	}

	//
	if(ano_opmv_lt_ctrl.target_loss==0) //有效，没丢失
	{
		//得到平移偏移量，并低通滤波
		lt_decou_pos_pixel_lpf[0] += 0.2f *((ano_opmv_lt_ctrl.opmv_pos - ano_opmv_lt_ctrl.r2pixel_val) - lt_decou_pos_pixel_lpf[0]);
		//再滤一次
		lt_decou_pos_pixel_lpf[1] += 0.2f *(lt_decou_pos_pixel_lpf[0] - lt_decou_pos_pixel_lpf[1]);

		//赋值
		ano_opmv_lt_ctrl.decou_pos_pixel = lt_decou_pos_pixel_lpf[1];

	}
	else //丢失目标
	{

		//低通复位到0
		LPF_1_(0.2f,dT_s,0,ano_opmv_lt_ctrl.decou_pos_pixel);


	}
	//

}

/**********************************************************************************************************
*函 数 名: ANO_LTracking_Calcu
*功能说明: 匿名科创寻线计算处理
*参    数: 周期时间(形参ms)，相对高度
*返 回 值: 无
**********************************************************************************************************/
static void ANO_LTracking_Calcu(u8 *dT_ms,s32 relative_height_cm)
{
	static float relative_height_cm_valid;
	static float lt_gnd_pos_err_old;
	//相对高度赋值
	if(relative_height_cm<500)
	{
		relative_height_cm_valid = relative_height_cm;
	}
	else
	{
		//null
	}
	//记录历史值
	lt_gnd_pos_err_old = ano_opmv_lt_ctrl.ground_pos_err_h_cm;

	//得到地面偏差，单位厘米
	ano_opmv_lt_ctrl.ground_pos_err_h_cm = LT_CMPPIXEL *relative_height_cm_valid *ano_opmv_lt_ctrl.decou_pos_pixel;

	//计算微分偏差，单位厘米每秒
	ano_opmv_lt_ctrl.ground_pos_err_d_h_cmps = (ano_opmv_lt_ctrl.ground_pos_err_h_cm - lt_gnd_pos_err_old)*(1000/(*dT_ms));


}

/**********************************************************************************************************
*函 数 名: ANO_LT_StepProcedure
*功能说明: 匿名科创寻线循迹分步控制任务
*参    数: 周期时间(ms,形参)
*返 回 值: 无
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
			//复位确认次数
			confirm_cnt = 0;
			//识别直线
			if(opmv.lt.sta == 1)
			{
				//
				ano_opmv_lt_ctrl.exp_velocity_h_cmps[0] = FORWARD_VEL;
			}
			//识别到直角左转
			else if(opmv.lt.sta == 2)
			{
				step_pro_sta = 2;
			}
			//识别到直角右转
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
			//确认是否识别到，否则返回
			if(opmv.lt.sta != 2)
			{
				step_pro_sta = 1;

			}
			else
			{
				//再确认n次
				if(confirm_cnt<CONFIRM_TIMES)
				{
					confirm_cnt++;
				}
				else
				{
					confirm_cnt = 0;
					//下一步
					step_pro_sta = 4;
				}		
			}
		}
		break;
		case 3:
		{
			//确认是否识别到，否则返回
			if(opmv.lt.sta != 3)
			{
				step_pro_sta = 1;

			}
			else
			{
				//再确认n次
				if(confirm_cnt<CONFIRM_TIMES)
				{
					confirm_cnt++;
				}
				else
				{
					confirm_cnt = 0;
					//下一步
					step_pro_sta = 5;
				}		
			}		
		}
		break;
		case 4:
		{
			//刹车减速
			ano_opmv_lt_ctrl.exp_velocity_h_cmps[0] = 10;//识别到直角后减速前进的速度cm/s
			ano_opmv_lt_ctrl.exp_velocity_h_cmps[1] = 0;//左右纠正速度复位
			//持续时间，转弯后偏内测，则加长时间;反之减少时间
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
			//刹车减速
			ano_opmv_lt_ctrl.exp_velocity_h_cmps[0] = 10;//识别到直角后减速前进的速度cm/s
			ano_opmv_lt_ctrl.exp_velocity_h_cmps[1] = 0;//左右纠正速度复位
			//持续时间，转弯后偏内测，则加长时间;反之减少时间
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
		case 12://左转90度
		{
			//
			ano_opmv_lt_ctrl.exp_velocity_h_cmps[0] = 30;//边旋转边给前进速度，走出一定圆弧
			ano_opmv_lt_ctrl.exp_velocity_h_cmps[1] = 0;//左右纠正速度复位
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
		case 13://右转90度
		{
			//
			ano_opmv_lt_ctrl.exp_velocity_h_cmps[0] = 30;//边旋转边给前进速度，走出一定圆弧
			ano_opmv_lt_ctrl.exp_velocity_h_cmps[1] = 0;//左右纠正速度复位
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
		case 20://衔接，因旋转后有可能没有看到直线，所以先设定前进500ms
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
*函 数 名: ANO_LTracking_Ctrl
*功能说明: 匿名科创色块跟踪控制任务
*参    数: 周期时间(ms)
*返 回 值: 无
**********************************************************************************************************/
void ANO_LTracking_Ctrl(u8 *dT_ms,u8 en)
{
	//开启控制
	if(en)
	{
		//距离偏差PD控制
		ano_opmv_lt_ctrl.exp_velocity_h_cmps[1]\
		= LT_KP *ano_opmv_lt_ctrl.ground_pos_err_h_cm\
		+ LT_KD *ano_opmv_lt_ctrl.ground_pos_err_d_h_cmps;

		//转向修正控制
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
		//前进、转向控制
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
