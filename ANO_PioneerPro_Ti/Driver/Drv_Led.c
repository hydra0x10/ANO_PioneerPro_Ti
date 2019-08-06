#include "Drv_Led.h"

void Dvr_LedInit(void)
{
	ROM_SysCtlPeripheralEnable(LED1_SYSCTL);
	ROM_SysCtlPeripheralEnable(LED2_SYSCTL);
    ROM_SysCtlPeripheralEnable(LED3_SYSCTL);
    ROM_SysCtlPeripheralEnable(LEDS_SYSCTL);
	ROM_GPIOPinTypeGPIOOutput(LED1_PORT, LED1_PIN);
	ROM_GPIOPinTypeGPIOOutput(LED2_PORT, LED2_PIN);
	ROM_GPIOPinTypeGPIOOutput(LED3_PORT, LED3_PIN);
	ROM_GPIOPinTypeGPIOOutput(LEDS_PORT, LEDS_PIN);
	//初始化点亮蓝色
	Drv_LedOnOff(LED_B, 1);
}

void Drv_LedOnOff(u8 led, u8 onoff)
{
	if(led & LED_R)
	{
		if(onoff)
			ROM_GPIOPinWrite(LED2_PORT, LED2_PIN, LED2_PIN);
		else
			ROM_GPIOPinWrite(LED2_PORT, LED2_PIN, 0);
	}
	if(led & LED_G)
	{
		if(onoff)
			ROM_GPIOPinWrite(LED1_PORT, LED1_PIN, LED1_PIN);
		else
			ROM_GPIOPinWrite(LED1_PORT, LED1_PIN, 0);
	}
	if(led & LED_B)
	{
		if(onoff)
			ROM_GPIOPinWrite(LED3_PORT, LED3_PIN, LED3_PIN);
		else
			ROM_GPIOPinWrite(LED3_PORT, LED3_PIN, 0);
	}
	if(led & LED_S)
	{
		if(onoff)
			ROM_GPIOPinWrite(LEDS_PORT, LEDS_PIN, 0);
		else
			ROM_GPIOPinWrite(LEDS_PORT, LEDS_PIN, LEDS_PIN);
	}
}


