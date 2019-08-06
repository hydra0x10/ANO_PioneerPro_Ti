#include "Ano_Navigate.h"
#include "Ano_Math.h"
#include "Ano_Imu.h"

s32 dlon_180(s32 x) //10^-7
{
	return (x>1800000000?(x-3600000000):(x<-1800000000?(x+3600000000):x));

}

void dlon_dlat_dx_dy(s32 lon,s32 lat,s32 lon_ref,s32 lat_ref,s32 *dx,s32 *dy ) //10^-7  ->  cm
{
	s32 dlon_t;
	float lon_cos;
	
	dlon_t = dlon_180(lon - lon_ref);
	lon_cos = my_cos( ABS( (s16)(lat/10000000) ) *ANGLE_TO_RADIAN );
	
	*dx = 1.117f *dlon_t *lon_cos;
	*dy = 1.117f *(lat - lat_ref); //10^-7 *1000 *100 = 0.01fï¼?.01f *111.7	

}


