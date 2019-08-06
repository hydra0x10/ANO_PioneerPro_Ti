/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _SCHEDULER_H_
#define _SCHEDULER_H_


/* Includes ------------------------------------------------------------------*/
#include "config.h"
/* Exported types ------------------------------------------------------------*/


typedef struct
{
	void(*task_func)(u32 dT_us);
//	u16 rate_hz;
	u32 interval_ticks;
	u32 last_run;
}sched_task_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

//static 

//user
u8 Main_Task(void);
void INT_1ms_Task(void);
#endif

