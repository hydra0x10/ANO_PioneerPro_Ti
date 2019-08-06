#include "Ano_Power.h"
#include "Ano_Parameter.h"
#include "Ano_Filter.h"
#include "Drv_led.h"
#include "Ano_Math.h"
#include "Ano_LED.h"

float Plane_Votage = 0;
static float voltage_f = 30000;
static u8 voltage_init_ok;
void Power_UpdateTask(u8 dT_ms)
{
	static s16 voltage_s16;
	float cut_off_freq;
	//触发ADC采样
	Drv_Adc0Trigger();
	//赋值电压数据
	voltage_s16 = Voltage*1000;
	
	if(voltage_init_ok == 0)
	{
		cut_off_freq = 2.0f;
		
		if(voltage_f >2000 && ABS(voltage_s16 - voltage_f) <200)
		{
			voltage_init_ok = 1;
		}
	}	
	else
	{
		cut_off_freq = 0.02f;
	}
	
	LPF_1_(cut_off_freq,dT_ms*1e-3f,voltage_s16,voltage_f);
	

	
	Plane_Votage = voltage_f *0.001f;
//	Plane_Votage = 15;


		
	if(Plane_Votage<Ano_Parame.set.lowest_power_voltage)
	{
		flag.power_state = 3;//将禁止解锁		
	}
	else
	{
		flag.power_state = 1;
	}

	if(Plane_Votage<Ano_Parame.set.warn_power_voltage)
	{
		LED_STA.lowVt = 1;
	}
	else if(Plane_Votage>Ano_Parame.set.warn_power_voltage+0.2f)
	{
		LED_STA.lowVt = 0;
	}
		
	if(Plane_Votage<Ano_Parame.set.return_home_power_voltage)
	{
		
	
	}
}





