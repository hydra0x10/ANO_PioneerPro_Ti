#include "Ano_ProgramCtrl_User.h"
#include "Ano_Math.h"


#define MAX_PC_XYVEL_CMPS  200
#define MAX_PC_ZVEL_CMPS   150
#define MAX_PC_PAL_DPS     100

_pc_user_st pc_user;

//=====1、航向水平坐标系程控速度功能接口函数=====
/**********************************************************************************************************
*函 数 名: Program_Ctrl_User_Set_HXYcmps
*功能说明: 程控功能，航向水平坐标系下速度设定（实时控制）
*参    数: X速度（厘米每秒，正为前进，负为后退，Y速度（厘米每秒，正为左移，负为右移）
*返 回 值: 无
**********************************************************************************************************/
void Program_Ctrl_User_Set_HXYcmps(float hx_vel_cmps,float hy_vel_cmps)
{
	//
	pc_user.vel_cmps_set_h[0] = hx_vel_cmps;
	pc_user.vel_cmps_set_h[1] = hy_vel_cmps;
	//限制XY速度模长
	length_limit(&pc_user.vel_cmps_set_h[0],&pc_user.vel_cmps_set_h[1],MAX_PC_XYVEL_CMPS,pc_user.vel_cmps_set_h);
}


//=====2、无头模式参考坐标系程控功速度能接口函数（暂无，稍后开发，或者参考上位机程控功能）=====
//
//
//

//=====3、通用程控速度功能接口函数=====
/**********************************************************************************************************
*函 数 名: Program_Ctrl_User_Set_WHZcmps
*功能说明: 程控功能，上升下降速度设定（实时控制）
*参    数: 速度（厘米每秒，正为上升，负为下降）
*返 回 值: 无
**********************************************************************************************************/
void Program_Ctrl_User_Set_Zcmps(float z_vel_cmps)
{
	//
	pc_user.vel_cmps_set_z = z_vel_cmps;
	//限幅
	pc_user.vel_cmps_set_z = LIMIT(pc_user.vel_cmps_set_z,-MAX_PC_ZVEL_CMPS,MAX_PC_ZVEL_CMPS);
}
/**********************************************************************************************************
*函 数 名: Program_Ctrl_User_Set_YAWdps
*功能说明: 程控功能，航向速度设定（实时控制）
*参    数: 速度（度每秒，正为右转，负为左转）
*返 回 值: 无
**********************************************************************************************************/
void Program_Ctrl_User_Set_YAWdps(float yaw_pal_dps)
{
	//
	pc_user.pal_dps_set = yaw_pal_dps;
	//限幅
	pc_user.pal_dps_set = LIMIT(pc_user.pal_dps_set,-MAX_PC_PAL_DPS,MAX_PC_PAL_DPS);
}





