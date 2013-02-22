#ifndef _MY_TIMERS_H
#define _MY_TIMERS_H
#include "lcdTask.h"
//#include "i2cTemp.h"
#include "navigation.h"
void startTimerForLCD(vtLCDStruct *vtLCDdata);
void startTimerForNav(vtNavStruct *vtNavdata);
#endif