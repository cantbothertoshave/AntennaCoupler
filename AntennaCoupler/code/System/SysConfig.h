
#pragma once

#include "types.h"


//typedef	void (SysTickFunc)	(int timeMillisecs);
typedef	void (SysTickFunc)	(void);




void	System_Init(uint32 tickPeriodMsecs);

void	System_RegisterTickFunc(SysTickFunc* f);

uint32		System_GetTimeMsecs();


void Delay(volatile uint32 nCount);

void	Test_SetLed(int led, int on);
