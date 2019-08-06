#ifndef _RC_H_
#define	_RC_H_
#include "Ano_FcData.h"

typedef struct//波动次数结构体
{
	u16 s_cnt;
	u8 s_now_times;
	u8 s_state;
}_stick_f_c_st;

#define _stick_f_lp_st u16 //长按

enum
{
	CH1 = 0,
	CH2,
	CH3,
	CH4,
	CH5,
	CH6,
	CH7,
	CH8
};
	
	



extern s16 CH_N[], RX_CH[CH_NUM];
extern u16 signal_intensity;
extern u8 chn_en_bit;

//static
void fail_safe_check(u8 dT_ms);

void stick_function(u8 dT_ms);

void stick_function_check_longpress(u8 dT_ms,u16 *time_cnt,u16 longpress_time_ms,u8 en,u8 trig_val,u8 *trig);

void ch_watch_dog(u8 dT_ms);


//public
void Remote_Control_Init(void);

void RC_duty_task(u8 dT_ms);

void signal_check_task(u8);

void ch_watch_dog_feed(u8 ch_n);


#endif


