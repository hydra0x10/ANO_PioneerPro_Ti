#include "ANO_LED.h"
#include "ANO_MATH.h"
#include "Drv_led.h"


u16 led_accuracy = 20;//��ʱ��Ӧ��LED_Duty()����������ͬ
float LED_Brightness[4] = {0,20,0,0}; //TO 20 //XBRG


//LED��1ms��������1ms��ʱ�ж�����á�
void LED_1ms_DRV( ) //0~20
{
	static u16 led_cnt[4];
	u8 i;
	for(i=0;i<4;i++)
	{
			
		if( led_cnt[i] < LED_Brightness[i] )
		{
			switch(i)
			{
				case 2:	
					Drv_LedOnOff(LED_R, 1);//LED1_ON;
				break;
				case 3:	
					Drv_LedOnOff(LED_G, 1);//LED2_ON;
				break;
				case 1:	
					Drv_LedOnOff(LED_B, 1);//LED3_ON;
				break;
				case 0:	
					Drv_LedOnOff(LED_S, 1);//LED4_ON;
				break;
			}
		}
		else
		{
			switch(i)
			{
				case 2:	
					Drv_LedOnOff(LED_R, 0);//LED1_OFF;
				break;
				case 3:	
					Drv_LedOnOff(LED_G, 0);//LED2_OFF;
				break;
				case 1:	
					Drv_LedOnOff(LED_B, 0);//LED3_OFF;
				break;
				case 0:	
					Drv_LedOnOff(LED_S, 0);//LED4_OFF;
				break;
			}
		}
		
		if(++led_cnt[i]>=led_accuracy)
		{
			led_cnt[i] = 0;
		}
	}
	

}

static void ledOnOff(u8 led)
{
	for(u8 i=0; i<LED_NUM; i++)
	{
		if(led & (1<<i))
			LED_Brightness[i] = 20;
		else
			LED_Brightness[i] = 0;
	}
}
static void ledBreath(u8 dT_ms,u8 led,u16 T)
{
	static u8 dir[LED_NUM];
	
	for(u8 i=0; i<LED_NUM; i++)
	{
		if(led & (1<<i))
		{
			switch(dir[i])
			{
				case 0:
					LED_Brightness[i] += safe_div(led_accuracy,((float)T/(dT_ms)),0);
					if(LED_Brightness[i]>20)
					{
						dir[i] = 1;
					}
				
				break;
				case 1:
					LED_Brightness[i] -= safe_div(led_accuracy,((float)T/(dT_ms)),0);
					if(LED_Brightness[i]<0)
					{
						dir[i] = 0;
					}
					
				break;
					
				default:
					dir[i] = 0;
				break;
			}
		}
		else
			LED_Brightness[i] = 0;
	}
}
//					��������   LED     ��ʱ��    ��ʱ��
static void ledFlash(u8 dT_ms,u8 led, u16 on_ms,u16 off_ms)
{
	static u16 tim_tmp;
	
	if(tim_tmp < on_ms)
		ledOnOff(led);
	else
		ledOnOff(0);
	
	tim_tmp += dT_ms;
	if(tim_tmp >= (on_ms + off_ms))
		tim_tmp = 0;
}

_led_sta LED_STA;
#include "Ano_FcData.h"
#include "Ano_OPMV_CBTracking_Ctrl.h"
#include "Ano_OPMV_LineTracking_Ctrl.h"
void LED_Task2(u8 dT_ms)
{
	static u16 timtmp = 0;

	if(LED_STA.errOneTime)
	{
		ledOnOff(BIT_RLED);
		timtmp += dT_ms;
		if(timtmp > 3000)
		{
			timtmp = 0;
			LED_STA.errOneTime = 0;
		}
	}
	else if(LED_STA.errMpu>0 || LED_STA.errMag>0 || LED_STA.errBaro>0)
	{
		u8 flashtims;
		static u8 cnttmp = 0;
		
		if(LED_STA.errMpu>0)
			flashtims = 2;
		else if(LED_STA.errMag>0)
			flashtims = 3;
		else 
			flashtims = 4;
		
		if(cnttmp < flashtims)
		{
			if(timtmp < 60)
				ledOnOff(BIT_RLED);
			else
				ledOnOff(0);
			timtmp += dT_ms;
			
			if(timtmp > 200)
			{
				timtmp = 0;
				cnttmp++;
			}
		}
		else
		{
			timtmp += dT_ms;
			if(timtmp > 1000)
			{
				timtmp = 0;
				cnttmp = 0;
			}
		}
	}

	else if(LED_STA.saving)
	{
		LED_Brightness[G_led] = 20;
		LED_Brightness[R_led] = 0;
		LED_Brightness[B_led] = 0;
	}
	else if(LED_STA.calAcc || LED_STA.calGyr || LED_STA.rst_imu)
	{
		ledFlash(dT_ms,BIT_WLED,40,40);
	}
	else if(LED_STA.calMag)
	{
		if(LED_STA.calMag == 1)
			ledBreath(dT_ms,BIT_GLED,300);
		else if(LED_STA.calMag == 2)
			ledFlash(dT_ms,BIT_PLED,40,40);
		else if(LED_STA.calMag == 100)
			ledFlash(dT_ms,BIT_RLED,40,40);
		else
			ledBreath(dT_ms,BIT_BLED,300);
			
	}
	else if(LED_STA.noRc)
	{
		ledBreath(dT_ms,BIT_RLED,500);
	}
	else if(LED_STA.lowVt)
	{
		ledFlash(dT_ms,BIT_RLED,60,60);
	}
	else	//��������ʾ��������ʾģʽ��λ�����ù�����Gps��״̬
	{
		static u8  statmp = 0;
		static u8  modtmp = 0;
		
		if(statmp == 0)			//��ʾ����ģʽ1��2��3����ɫδ��������ɫ������ģʽ����������
		{
			if(modtmp <= flag.flight_mode)
			{
				if(timtmp < 60)
				{
					if(flag.unlock_sta)
						ledOnOff(BIT_GLED);
					else
						ledOnOff(BIT_WLED);
				}
				else
					ledOnOff(0);
				timtmp += dT_ms;
				
				if(timtmp > 200)
				{
					timtmp = 0;
					modtmp++;
				}
			}
			else
			{
				modtmp = 0;
				statmp = 1;
			}
		}
		else if(statmp == 1)	//��ʾ������GPS��״̬
		{
			if(modtmp == 0)		//�ж�GPS�Ƿ�����
			{
				if(switchs.gps_on)
				{
					if(timtmp < 60)
						ledOnOff(BIT_BLED);
					else
						ledOnOff(0);
					timtmp += dT_ms;
					if(timtmp > 200)
					{
						timtmp = 0;
						modtmp++;
					}
				}
				else
					modtmp = 1;
			}
			else if(modtmp == 1)	//�жϹ����Ƿ�����
			{
				if(switchs.of_flow_on>0 && switchs.of_tof_on>0)
				{
					if(timtmp < 60)
						ledOnOff(BIT_PLED);
					else
						ledOnOff(0);
					timtmp += dT_ms;
					if(timtmp > 200)
					{
						timtmp = 0;
						modtmp++;
					}
				}
				else
					modtmp = 2;
			}
			else if(modtmp == 2)		//�жϷɿ����ANO_OPMV�Ƿ�����
			{
				if(switchs.opmv_on && (ano_opmv_cbt_ctrl.target_loss==0 || ano_opmv_lt_ctrl.target_loss==0) )
				{
					if(timtmp < 60)
						ledOnOff(BIT_YLED);
					else
						ledOnOff(0);
					timtmp += dT_ms;
					if(timtmp > 200)
					{
						timtmp = 0;
						modtmp++;
					}
				}
				else
					modtmp = 3;
			}
			
			if(modtmp == 3)
			{
				statmp = 2;
				modtmp = 0;
			}
		}
		else		//ÿһ��ѭ���󣬼�һ����Ƴ���ʱ
		{
			ledOnOff(0);
			timtmp += dT_ms;
			if(timtmp > 1000)
			{
				timtmp = 0;
				statmp = 0;
				modtmp = 0;
			}
		}
	}
}


