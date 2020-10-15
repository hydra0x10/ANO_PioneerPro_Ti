/*==========================================================================
 * 描述    ：对OPMV传回的数据进行处理，并解除因机体俯仰、横滚旋转而造成追踪
             目标坐标变化的耦合，也称作“旋转解耦”或“旋转补偿”。并用解算出的
						 像素偏差耦合高度计算到地面偏差，用地面偏差控制期望速度，以减小
						 偏差，实现追踪。
 
 * 更新时间：2019-07-03 
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
============================================================================
更新：
201908022321-Jyoun：修复对优像光流的兼容。
201908032123-Jyoun：修改角度解耦数据类型，整形改为float
201908032253-Jyoun：增加对微分的滤波，并重新整定参数，效果提升。

===========================================================================*/

//默认引用
#include "Ano_OPMV_CBTracking_Ctrl.h"
#include "Drv_OpenMV.h"
#include "Ano_OPMV_Ctrl.h"
#include "Ano_Math.h"

//
//数据接口定义：
//=========mapping===============
//需要引用的文件：
#include "ANO_IMU.h"
#include "Ano_OF.h"
#include "Ano_OF_DecoFusion.h"
#include "Ano_FlightCtrl.h"
#include "Ano_MotionCal.h"
//需要调用引用的外部变量：
#define IMU_ROL                 (imu_data.rol)     //横滚角
#define IMU_PIT                 (imu_data.pit)     //俯仰角
#define RELATIVE_HEIGHT_CM           (jsdata.valid_of_alt_cm)  //相对高度
//
#define OF_DATA_SOURCE               ((sens_hd_check.of_ok)?1:0)
#define VELOCITY_CMPS_X_S1           (OF_DX2FIX)                 //载体运动速度h_x
#define VELOCITY_CMPS_Y_S1           (OF_DY2FIX)                 //载体运动速度h_y
#define VELOCITY_CMPS_X_S2           (of_rdf.gnd_vel_est_h[0])   //载体运动速度h_x
#define VELOCITY_CMPS_Y_S2           (of_rdf.gnd_vel_est_h[1])   //载体运动速度h_y
//
#define CBT_KP                  (0.80f)  //比例项
#define CBT_KD                  (0.20f)  //微分项
#define CBT_KF                  (0.20f)  //前馈项

//需要操作赋值的外部变量：


//===============================
//全局变量：
static u16 target_loss_hold_time;
_ano_opmv_cbt_ctrl_st ano_opmv_cbt_ctrl;
static s16 ref_carrier_velocity[2];
static float decou_pos_pixel_lpf[2][2];

//参数设定：
#define PIXELPDEG_X    2.4f  //每1角度对应的像素个数，与分辨率和焦距有关，需要调试标定。//3.2f->(160*120,3.6mm)  2.4f->(160*120,2.8mm)
#define PIXELPDEG_Y    2.4f  //每1角度对应的像素个数，与分辨率和焦距有关，需要调试标定。
#define CMPPIXEL_X     0.01f     //每像素对应的地面距离，与焦距和高度有关，需要调试标定。//目前粗略标定
#define CMPPIXEL_Y     0.01f     //每像素对应的地面距离，与焦距和高度有关，需要调试标定。
#define TLH_TIME       1000   //判断目标丢失的保持时间。



	
/**********************************************************************************************************
*函 数 名: ANO_CBTracking_Task
*功能说明: 匿名科创色块跟踪任务
*参    数: 周期时间(ms)
*返 回 值: 无
**********************************************************************************************************/
void ANO_CBTracking_Task(u8 dT_ms)
{
	//开启控制的条件，可以自己修改
	if(opmv.mode_sta==1)// && opmv_ct_sta.en)
	{
		//跟踪数据旋转解耦合
		ANO_CBTracking_Decoupling(&dT_ms,IMU_ROL,IMU_PIT);
		//跟踪数据计算
		ANO_CBTracking_Calcu(&dT_ms,(s32)RELATIVE_HEIGHT_CM);
	}
	else
	{
		//reset
		ano_opmv_cbt_ctrl.target_loss = 1;
	}
}

/**********************************************************************************************************
*函 数 名: ANO_CBTracking_Decoupling
*功能说明: 匿名科创色块跟踪解耦合
*参    数: 周期时间(形参ms)，横滚角度，俯仰角度
*返 回 值: 无
**********************************************************************************************************/
static void ANO_CBTracking_Decoupling(u8 *dT_ms,float rol_degs,float pit_degs)
{
	float dT_s = (*dT_ms) *1e-3f;

	//有识别到目标
	if(opmv.cb.sta != 0)//(opmv.cb.color_flag!=0)
	{
		//
		ano_opmv_cbt_ctrl.target_loss = 0;
		target_loss_hold_time = 0;
	}
	else
	{
		//延迟一定时间
		if(target_loss_hold_time<TLH_TIME)
		{
			target_loss_hold_time += *dT_ms;
		}
		else
		{
			//目标丢失标记置位
			ano_opmv_cbt_ctrl.target_loss = 1;
		}
	}
	//换到飞控坐标系
	ano_opmv_cbt_ctrl.opmv_pos[0] =  opmv.cb.pos_y;
	ano_opmv_cbt_ctrl.opmv_pos[1] = -opmv.cb.pos_x;
	//
	if(opmv.cb.sta != 0)
	{
		//更新姿态量对应的偏移量
		ano_opmv_cbt_ctrl.rp2pixel_val[0] = -PIXELPDEG_X *pit_degs;
		ano_opmv_cbt_ctrl.rp2pixel_val[1] = -PIXELPDEG_Y *rol_degs;
		ano_opmv_cbt_ctrl.rp2pixel_val[0] = LIMIT(ano_opmv_cbt_ctrl.rp2pixel_val[0],-60,60);//高度120pixel
		ano_opmv_cbt_ctrl.rp2pixel_val[1] = LIMIT(ano_opmv_cbt_ctrl.rp2pixel_val[1],-80,80);//宽度160pixel
		//赋值参考的载体运动速度
		if(OF_DATA_SOURCE==1)
		{
			//来源1  ANO_OF
			ref_carrier_velocity[0] = VELOCITY_CMPS_X_S1;
			ref_carrier_velocity[1] = VELOCITY_CMPS_Y_S1;		
		}
		else
		{
			//来源2 UP_OF
			ref_carrier_velocity[0] = VELOCITY_CMPS_X_S2;
			ref_carrier_velocity[1] = VELOCITY_CMPS_Y_S2;					
		}
	}
	else
	{
		//姿态量对应偏移量保持不变
		//参考的载体运动速度复位0
		ref_carrier_velocity[0] = 0;
		ref_carrier_velocity[1] = 0;
	}

	//
	if(ano_opmv_cbt_ctrl.target_loss==0) //有效，没丢失
	{
		//得到平移偏移量，并低通滤波
		decou_pos_pixel_lpf[0][0] += 0.2f *((ano_opmv_cbt_ctrl.opmv_pos[0] - ano_opmv_cbt_ctrl.rp2pixel_val[0]) - decou_pos_pixel_lpf[0][0]);
		decou_pos_pixel_lpf[0][1] += 0.2f *((ano_opmv_cbt_ctrl.opmv_pos[1] - ano_opmv_cbt_ctrl.rp2pixel_val[1]) - decou_pos_pixel_lpf[0][1]);
		//再滤一次
		decou_pos_pixel_lpf[1][0] += 0.2f *(decou_pos_pixel_lpf[0][0] - decou_pos_pixel_lpf[1][0]);
		decou_pos_pixel_lpf[1][1] += 0.2f *(decou_pos_pixel_lpf[0][1] - decou_pos_pixel_lpf[1][1]);
		//赋值
		ano_opmv_cbt_ctrl.decou_pos_pixel[0] = decou_pos_pixel_lpf[1][0];
		ano_opmv_cbt_ctrl.decou_pos_pixel[1] = decou_pos_pixel_lpf[1][1];
	}
	else //丢失目标
	{
//		ano_opmv_cbt_ctrl.decou_pos_pixel[0] = ano_opmv_cbt_ctrl.decou_pos_pixel[1] = 0;
		//低通复位到0
		LPF_1_(0.2f,dT_s,0,ano_opmv_cbt_ctrl.decou_pos_pixel[0]);
		LPF_1_(0.2f,dT_s,0,ano_opmv_cbt_ctrl.decou_pos_pixel[1]);

	}
	//

}

/**********************************************************************************************************
*函 数 名: ANO_CBTracking_Calcu
*功能说明: 匿名科创色块跟踪计算处理
*参    数: 周期时间(形参ms)，相对高度
*返 回 值: 无
**********************************************************************************************************/
static void ANO_CBTracking_Calcu(u8 *dT_ms,s32 relative_height_cm)
{
	static float relative_height_cm_valid;
	static float g_pos_err_old[2];
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
	g_pos_err_old[0] = ano_opmv_cbt_ctrl.ground_pos_err_h_cm[0];
	g_pos_err_old[1] = ano_opmv_cbt_ctrl.ground_pos_err_h_cm[1];
	//得到地面偏差，单位厘米
	ano_opmv_cbt_ctrl.ground_pos_err_h_cm[0] = CMPPIXEL_X *relative_height_cm_valid *ano_opmv_cbt_ctrl.decou_pos_pixel[0];
	ano_opmv_cbt_ctrl.ground_pos_err_h_cm[1] = CMPPIXEL_Y *relative_height_cm_valid *ano_opmv_cbt_ctrl.decou_pos_pixel[1];
	//计算微分偏差，单位厘米每秒
	float gped_tmp[2];
	gped_tmp[0] = (ano_opmv_cbt_ctrl.ground_pos_err_h_cm[0] - g_pos_err_old[0])*(1000/(*dT_ms));
	gped_tmp[1] = (ano_opmv_cbt_ctrl.ground_pos_err_h_cm[1] - g_pos_err_old[1])*(1000/(*dT_ms));
	ano_opmv_cbt_ctrl.ground_pos_err_d_h_cmps[0] += 0.2f *(gped_tmp[0] - ano_opmv_cbt_ctrl.ground_pos_err_d_h_cmps[0]);
	ano_opmv_cbt_ctrl.ground_pos_err_d_h_cmps[1] += 0.2f *(gped_tmp[1] - ano_opmv_cbt_ctrl.ground_pos_err_d_h_cmps[1]);
	//计算目标的地面速度，单位厘米每秒
//	s16 temp[2];
//	temp[0] = (s16)(ano_opmv_cbt_ctrl.ground_pos_err_d_h_cmps[0] + ref_carrier_velocity[0]) ;
//	temp[1] = (s16)(ano_opmv_cbt_ctrl.ground_pos_err_d_h_cmps[1] + ref_carrier_velocity[1]) ;
	ano_opmv_cbt_ctrl.target_gnd_velocity_cmps[0] = (ano_opmv_cbt_ctrl.ground_pos_err_d_h_cmps[0] + ref_carrier_velocity[0]) ;
	ano_opmv_cbt_ctrl.target_gnd_velocity_cmps[1] = (ano_opmv_cbt_ctrl.ground_pos_err_d_h_cmps[1] + ref_carrier_velocity[1]) ;
}

/**********************************************************************************************************
*函 数 名: ANO_CBTracking_Ctrl
*功能说明: 匿名科创色块跟踪控制
*参    数: 周期时间(ms,形参)，使能
*返 回 值: 无
**********************************************************************************************************/
void ANO_CBTracking_Ctrl(u8 *dT_ms,u8 en)
{
	//开启控制
	if(en)
	{
		//距离偏差PD控制和速度前馈
		ano_opmv_cbt_ctrl.exp_velocity_h_cmps[0]\
		= CBT_KF *ano_opmv_cbt_ctrl.target_gnd_velocity_cmps[0]\
		+ CBT_KP *(ano_opmv_cbt_ctrl.ground_pos_err_h_cm[0])\
		+ CBT_KD *ano_opmv_cbt_ctrl.ground_pos_err_d_h_cmps[0];
		
		ano_opmv_cbt_ctrl.exp_velocity_h_cmps[1]\
		= CBT_KF *ano_opmv_cbt_ctrl.target_gnd_velocity_cmps[1]\
		+ CBT_KP *(ano_opmv_cbt_ctrl.ground_pos_err_h_cm[1])\
		+ CBT_KD *ano_opmv_cbt_ctrl.ground_pos_err_d_h_cmps[1];
	}
	else
	{
		ano_opmv_cbt_ctrl.exp_velocity_h_cmps[0] = ano_opmv_cbt_ctrl.exp_velocity_h_cmps[1] = 0;
	}

}	



