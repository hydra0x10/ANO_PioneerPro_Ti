/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CONFIG_H
#define __CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "sysconfig.h"
/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/***************换算******************/
#define ANGLE_TO_RADIAN 0.01745329f //*0.01745 = /57.3	角度转弧度
/***********************************************/

#define ANO_DT_USE_NRF24l01
/***********************************************/



#define SP_EST_DRAG 1.0f
#define BARO_WIND_COMP 0.10f


/***********************************************/
//==
//GYR_ACC_FILTER 参数大致范围参考
//500KV以下 0.15f~0.2f
//500~2000KV 0.2f~0.3f
//2000kv以上 0.3f-0.5f
//==
//FINAL_P 参数大致范围参考
//500KV以下 0.4f以上
//500~2000KV 0.4f~0.3f
//2000kv以上 0.3f-0.2f
//==
#define GYR_ACC_FILTER 0.25f //陀螺仪加速度计滤波系数
#define FINAL_P 			 0.35f  //电机输出量比例系数

#define MOTOR_ESC_TYPE 1  //2：无刷电机带刹车的电调，1：无刷电机不带刹车的电调，
#define MOTORSNUM 4

#define BAT_LOW_VOTAGE 3250    //mV
#define FLOAW_MAX_HEIGHT  450
#define FLOW_ROLL_CONDITION 8 //   0-12

#define APP_ROLL_CH CH_PIT //app翻滚

#define MAX_ANGLE     25.0f
//#define MAX_ANGLE_ROL 25.0f //角度
//#define MAX_ANGLE_PIT 25.0f //角度

#define MAX_SPEED_ROL 200  //角度每秒
#define MAX_SPEED_PIT 200  //角度每秒
#define MAX_SPEED_YAW 250  //角度每秒

#define MAX_ROLLING_SPEED 1600  //角度每秒

#define MAX_SPEED 500 //最大水平速度，厘米每秒 cm/s

#define MAX_Z_SPEED_UP 350 //厘米每秒 cm/s
#define MAX_Z_SPEED_DW 250 //厘米每秒 cm/s

#define MAX_EXP_XY_ACC   500 //厘米每平方秒 cm/ss
#define MAX_EXP_Z_ACC    600 //厘米每平方秒 cm/ss

#define CTRL_1_INTE_LIM 250 //角速度环积分限幅 ：输出

#define ANGULAR_VELOCITY_PID_INTE_D_LIM 300/FINAL_P  
#define X_PROPORTION_X_Y 1.0f //proportion
#define ROLL_ANGLE_KP 10.0f   //翻滚角度kp

#define MAX_THR_SET    85  //最大油门百分比 %
#define THR_INTE_LIM_SET   70  //油门积分百分比 % 

//#define MAX_THR       MAX_THR_SET/FINAL_P   
#define THR_INTE_LIM   THR_INTE_LIM_SET/FINAL_P  

#define THR_START      35  //油门起调量百分比 %

#define LAND_ACC              500  //着陆加速度检测
#define LAND_ACC_DELTA        300  //着陆加速度变化量检测


#define BARO_FIX -0                          //气压速度积分修正起调值/CM厘米
//#define AUTO_TAKE_OFF_HEIGHT 50  //自动起飞高度
//#define HEIGHT_FIX           0               //气压高度固定补偿/CM厘米



#endif


