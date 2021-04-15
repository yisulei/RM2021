#ifndef _FUNCTION_H_
#define _FUNCTION_H_

#include "main.h"


typedef struct ccr
{
	int angle;
	int middle;
	int max;
	int min;
}ccr;


typedef struct ccr_system
{
	ccr ccr1;
	ccr ccr2;
	ccr ccr3;
}ccr_system;


void Gyro_Analyse(void);
void change(uint8_t *rx_data);
void Position_Data(void);
void Fly_Steady(ccr_system*CCR);
void Fly_Change(ccr_system*CCR);


#endif

