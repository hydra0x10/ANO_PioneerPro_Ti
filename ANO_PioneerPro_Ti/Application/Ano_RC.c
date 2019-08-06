/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
  * 作者   ：匿名科创
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 * 描述    ：遥控器通道数据处理
**********************************************************************************/
#include "sysconfig.h"
#include "Ano_Parameter.h"
#include "Ano_RC.h"
#include "Ano_Math.h"
#include "Drv_icm20602.h"
#include "Ano_MagProcess.h"
#include "Drv_led.h"
#include "Drv_RcIn.h"
#include "Ano_DT.h"
#include "Ano_Sensor_Basic.h"
#include "Ano_LED.h"

//摇杆触发值，摇杆值范围为+-500，超过300属于触发范围
#define UN_YAW_VALUE  300
#define UN_THR_VALUE  300
#define UN_PIT_VALUE  300
#define UN_ROL_VALUE  300

static u8 RC_IN_MODE;
void Remote_Control_Init()
{
	//
	RC_IN_MODE = Ano_Parame.set.pwmInMode;
	//
	if(RC_IN_MODE == SBUS)
	{
		Drv_SbusInit();
	}
	else
	{
		Drv_PpmInit();
//		PWM_IN_Init(RC_IN_MODE);
	}
}

static u16 cwd_cnt[10] ;
u8 chn_en_bit = 0;
void ch_watch_dog_feed(u8 ch_n)
{
	ch_n = LIMIT(ch_n,0,7);
	cwd_cnt[ch_n] = 0;
}

static void ch_watch_dog(u8 dT_ms)//如果是PPM/SBUS模式，也只检测前8通道
{
	for(u8 i = 0;i<8;i++)
	{
		if(cwd_cnt[i]<500)
		{
			cwd_cnt[i] += dT_ms;
			chn_en_bit |= 0x01<<i;
		}
		else
		{
			chn_en_bit &= ~(0x01<<i);
//			Rc_Pwm_In[i] = 0;  //把捕获值复位
//			Rc_Ppm_In[i] = 0;
//			Rc_Sbus_In[i] = 0;
		}
	}
}

u16 signal_intensity;

s16 CH_N[CH_NUM] = {0,0,0,0};

_stick_f_lp_st unlock_f;
u8 stick_fun_0;
u16 unlock_time = 200;

void unlock(u8 dT_ms)
{
	
	if( flag.power_state <=2 && para_sta.save_trig == 0)//只有电池电压非最低并且没有操作flash时，才允许进行解锁
	{
		if(sens_hd_check.acc_ok && sens_hd_check.gyro_ok)
		{
			if(sens_hd_check.baro_ok)
			{
				if(flag.sensor_imu_ok  )//imu传感器正常时，才允许解锁
				{
					flag.unlock_err = 0;	//允许解锁标志位

				}
				else
				{
					flag.unlock_err = 1;//imu异常，不允许解锁

				}
			}
			else
			{
				LED_STA.errBaro = 1;
				flag.unlock_err = 2;//气压计异常，不允许解锁。
			}
		}
		else
		{
			LED_STA.errMpu = 1;
			flag.unlock_err = 3;//惯性传感器异常，不允许解锁。
		}
	}
	else
	{
		flag.unlock_err = 4;//电池电压异常，不允许解锁
	}
	
	//解锁
	if(flag.unlock_sta == 0)
	{
		if(flag.unlock_cmd != 0)
		{		
			if(flag.unlock_err == 0)
			{
				//
				flag.unlock_sta = flag.unlock_cmd;
				//
				ANO_DT_SendString("Unlock OK!");
				
			}
			else 
			{
				//reset
				flag.unlock_cmd = 0;
				//
				if(flag.unlock_err == 1)
				{
					ANO_DT_SendString("Unlock Fail!");
				}
				else if(flag.unlock_err == 2)
				{
					ANO_DT_SendString("Unlock Fail!");
				}
				else if(flag.unlock_err == 3)
				{
					ANO_DT_SendString("Unlock Fail!");
				}
				else if(flag.unlock_err == 4)
				{
					ANO_DT_SendString("Power Low,Unlock Fail!");
				}
				else
				{
				
				}
			}
		}
		else
		{
		
		}
	}
	else
	{
		if(flag.unlock_cmd == 0)
		{
			ANO_DT_SendString(" FC Output Locked! ");
		}		
		flag.unlock_sta = flag.unlock_cmd;
	}
	
	////////////////////////////////////////////
	//所有功能判断，都要油门在低值时才进行
	if(CH_N[CH_THR] < -UN_THR_VALUE  )
	{
		//判断用户是否想要上锁、解锁
		if(ABS(CH_N[CH_YAW])>0.1f*UN_YAW_VALUE && CH_N[CH_PIT]< -0.1f*UN_PIT_VALUE)
		{
			if(flag.locking == 0)
			{
				flag.locking = 1;
			}
		}
		else
		{
			flag.locking = 0;
		}

		//飞控上锁、解锁检测
		if(CH_N[CH_PIT]<-UN_PIT_VALUE && CH_N[CH_ROL]>UN_ROL_VALUE && CH_N[CH_YAW]<-UN_YAW_VALUE)
		{
			stick_fun_0 = 1;
			flag.locking = 2;
		}
		else if(CH_N[CH_PIT]<-UN_PIT_VALUE && CH_N[CH_ROL]<-UN_ROL_VALUE && CH_N[CH_YAW]>UN_YAW_VALUE)
		{
			stick_fun_0 = 1;
			flag.locking = 2;
		}
		else
		{
			stick_fun_0 = 0;
		}
			
		
		u8 f = 0;		
		if(flag.unlock_sta)
		{
			//如果为解锁状态，最终f=0，将f赋值给flag.unlock_sta，飞控完成上锁
			f = 0;
			unlock_time = 1000;
		}
		else
		{
			//如果飞控为锁定状态，则f=2，将f赋值给flag.unlock_sta，飞控解锁完成
			f = 2;
			unlock_time = 200;
		}
		//进行最终的时间积分判断，摇杆必须满足条件unlock_time时间后，才会执行锁定和解锁动作
		stick_function_check_longpress(dT_ms,&unlock_f,unlock_time,stick_fun_0,f,&flag.unlock_cmd);
	}
	else
	{
		flag.locking = 0; //油门高
		if(flag.unlock_cmd == 2)
		{
			flag.unlock_cmd = 1;
		}
	}

	
	if(CH_N[CH_THR]>-350)
	{
		flag.thr_low = 0;//油门非低
	}
	else
	{
		flag.thr_low = 1;//油门拉低
	}
}

void RC_duty_task(u8 dT_ms) //建议2ms调用一次
{
	if(flag.start_ok)	
	{
		/////////////获得通道数据////////////////////////
//		if(RC_IN_MODE == PWM)
//		{
//			for(u8 i=0;i<CH_NUM;i++)
//			{
//				if(chn_en_bit & (1<<i))//(Rc_Pwm_In[i]!=0)//该通道有值，==0说明该通道未插线（PWM）
//				{
//					//CH_N[]+1500为上位机显示通道值
//					CH_N[i] = 1.25f *((s16)Rc_Pwm_In[i] - 1500); //1100 -- 1900us,处理成大约+-500摇杆量

//				}
//				else
//				{
//					CH_N[i] = 0;
//				}
//				CH_N[i] = LIMIT(CH_N[i],-500,500);//限制到+—500
//			}
//		}
//		else if(RC_IN_MODE == PPM)
		if(RC_IN_MODE == PPM || RC_IN_MODE == PWM)
		{
			for(u8 i=0;i<CH_NUM;i++)
			{
				if(chn_en_bit & (1<<i))//(Rc_Ppm_In[i]!=0)//该通道有值
				{
					//CH_N[]+1500为上位机显示通道值
					CH_N[i] = ((s16)RC_PPM.Captures[i] - 1500); //1000 -- 2000us,处理成大约+-500摇杆量
				}
				else
				{
					CH_N[i] = 0;
				}
				CH_N[i] = LIMIT(CH_N[i],-500,500);//限制到+—500
			}		
		}
		else//sbus
		{
			for(u8 i=0;i<CH_NUM;i++)
			{
				if(chn_en_bit & (1<<i))//该通道有值
				{
					//CH_N[]+1500为上位机显示通道值
					CH_N[i] = 0.65f *((s16)Rc_Sbus_In[i] - 1024); //248 --1024 --1800,处理成大约+-500摇杆量
				}
				else
				{
					CH_N[i] = 0;
				}
				CH_N[i] = LIMIT(CH_N[i],-500,500);//限制到+—500
			}					
		}

		///////////////////////////////////////////////
		//解锁监测	
		unlock(dT_ms);
		//摇杆触发功能监测
		stick_function(dT_ms);	
		//通道看门狗
		ch_watch_dog(dT_ms);

		//失控保护检查
		fail_safe_check(dT_ms);//3ms


	}
}

void fail_safe()
{
	for(u8 i = 0;i<4;i++)
	{
		CH_N[i] = 0;
	}

	if(CH_N[CH_THR]>0)
	{
		CH_N[CH_THR] = 0;
	}

	CH_N[CH_ROL] = 0;
	CH_N[CH_PIT] = 0;
	CH_N[CH_YAW] = 0;
	
	//切记不能给 CH_N[AUX1]赋值，否则可能导致死循环。（根据AUX1特殊值判断接收机failsafe信号）
	
	if(flag.unlock_sta)
	{
		if(switchs.gps_on ==0)
		{
			flag.auto_take_off_land = AUTO_LAND; //如果解锁，自动降落标记置位
		}
		else
		{
			flag.rc_loss_back_home = 1;
		}
		
	}
}

u16 test_si_cnt;

void fail_safe_check(u8 dT_ms) //dT秒调用一次
{
	static u16 cnt;
	static s8 cnt2;
	
	cnt += dT_ms;
	if(cnt >= 500) //500*dT 秒
	{
		cnt=0;
		if((chn_en_bit & 0x0F) != 0x0F || flag.chn_failsafe ) //前4通道有任意一通道无信号或者受到接收机失控保护信号
		{
			cnt2 ++;
		}
		else
		{
			cnt2 --;	
		}
		
		if(cnt2>=2)
		{
			cnt2 = 0;
			
			flag.rc_loss = 1; //认为丢失遥控信号
			
			LED_STA.noRc = 1;
			
			fail_safe();


				
		}
		else if(cnt2<=-2) //认为信号正常
		{
			cnt2 = 0;
			
			if(flag.rc_loss)
			{
				flag.rc_loss = 0;
				LED_STA.noRc = 0;
				
					if(flag.taking_off)
					flag.auto_take_off_land = AUTO_TAKE_OFF_FINISH; //解除下降
			}
			
		}
		
		test_si_cnt = signal_intensity;
		signal_intensity=0; //累计接收次数
	}
	
	
}

void stick_function_check(u8 dT_ms,_stick_f_c_st *sv,u8 times_n,u16 reset_time_ms,u8 en,u8 trig_val,u8 *trig)
{
	if(en)
	{
		sv->s_cnt = 0; //清除计时
		if(sv->s_state==0)
		{
			if(sv->s_now_times!=0)
			{
				sv->s_now_times++;
			}
			sv->s_state = 1;
		}
	}
	else
	{
		sv->s_state = 0;
		/////
		sv->s_cnt += dT_ms;
		if(sv->s_cnt>reset_time_ms)
		{
			sv->s_now_times = 1; //清除记录次数
		}
	}

	if(sv->s_now_times> times_n)
	{
		*trig = trig_val;            //触发功能标记
		sv->s_now_times = 0;
	}

}
void stick_function_check_longpress(u8 dT_ms,u16 *time_cnt,u16 longpress_time_ms,u8 en,u8 trig_val,u8 *trig)
{
	//dT_ms：调用间隔时间
	//time_cnt：积分时间
	//longpress_time_ms：阈值时间，超过这个时间则为满足条件
	//en：摇杆状态是否满足
	//trig_val：满足后的触发值
	//trig：指向需要触发的寄存器
	if(en)//如果满足摇杆条件，则进行时间积分
	{
		if(*time_cnt!=0)
		{
			*time_cnt+=dT_ms;
		}
	}
	else//不满足条件，积分恢复1
	{
		*time_cnt=1;
	}
	//时间积分满足时间阈值，则触发标记
	if(*time_cnt>=longpress_time_ms)
	{
		*trig = trig_val;            //触发功能标记
		*time_cnt = 0;
	}

}

_stick_f_lp_st cali_gyro,cali_acc,cali_surface;
_stick_f_c_st cali_mag;

u8 stick_fun_1,stick_fun_2,stick_fun_3,stick_fun_4,stick_fun_5_magcali;
void stick_function(u8 dT_ms)
{
	//////////////状态监测
	//未解锁才允许检测摇杆功能
	if(flag.unlock_sta == 0)
	{
		//油门低，则继续
		if(flag.thr_low)
		{
			if(CH_N[CH_PIT]<-350 && CH_N[CH_ROL]>350 && CH_N[CH_THR]<-350 && CH_N[CH_YAW]>350)
			{
				stick_fun_1 = stick_fun_2 = 1;
			}
			else
			{
				stick_fun_1 = stick_fun_2 = 0;
			}
			
			if(CH_N[CH_PIT]>350 && CH_N[CH_ROL]>350 && CH_N[CH_THR]<-350 && CH_N[CH_YAW]<-350)
			{
				stick_fun_3 = 1;
			}
			else
			{
				stick_fun_3 = 0;
			}
			
			if(CH_N[CH_PIT]>350 && CH_N[CH_ROL]>350 && CH_N[CH_THR]<-350 && CH_N[CH_YAW]>350)
			{
				stick_fun_4 = 1;
			}
			else
			{
				stick_fun_4 = 0;
			}
			
			if(CH_N[CH_PIT]>350)
			{
				stick_fun_5_magcali =1;
			}
			else if(CH_N[CH_PIT]<50)
			{
				stick_fun_5_magcali =0;
			}
		}
		
			///////////////
		//触发陀螺仪校准
		stick_function_check_longpress(dT_ms,&cali_gyro,1000,stick_fun_1,1,&sensor.gyr_CALIBRATE);
		//触发加速度计校准
		stick_function_check_longpress(dT_ms,&cali_acc,1000,stick_fun_2,1,&sensor.acc_CALIBRATE);
		
//		stick_function_check_longpress(dT_ms,&cali_surface,1000,stick_fun_4,1,&sensor_rot.surface_CALIBRATE );
		//触发罗盘校准
		stick_function_check(dT_ms,&cali_mag,5,1000,stick_fun_5_magcali,1,&mag.mag_CALIBRATE);

		
	}

	//////////////
}











/******************* (C) COPYRIGHT 2017 ANO TECH *****END OF FILE************/

