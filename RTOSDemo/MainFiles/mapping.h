#ifndef MAP_TASK_H
#define MAP_TASK_H
#include "vtI2C.h"
#include "lcdTask.h"
#include "navigation.h"
// Structure used to pass parameters to the task
// Do not touch...
typedef struct __MapStruct {
	vtI2CStruct *dev;
	vtLCDStruct *lcdData;
	//vtNavStruct *navData;
	xQueueHandle inQ;
} vtMapStruct;
// Maximum length of a message that can be received by this task
#define vtMapMaxLen   (sizeof(portTickType))

// Public API
//
// Start the task
// Args:
//   mapData: Data structure used by the task
//   uxPriority -- the priority you want this task to be run at
//   i2c: pointer to the data structure for an i2c task
//   lcd: pointer to the data structure for an LCD task (may be NULL)
//   nav: pointer to the data structure for a nav task
void vStartMapTask(vtMapStruct *mapData,unsigned portBASE_TYPE uxPriority, vtI2CStruct *i2c,vtLCDStruct *lcd/*, vtNavStruct *nav*/);
//
// Send a value message to the Mapping task
// Args:
//   mapData -- a pointer to a variable of type vtMapStruct
//   msgType -- the type of the message to send	(turn right / turn left / wall) 
//   value -- the radius of the turn or the wall byte
//   rightDistance -- The distance the right wheels have travled since the last update point
//	 leftDistance -- The distance the left wheels have travled since the last update point
//   ticksToBlock -- how long the routine should wait if the queue is full
// Return:
//   Result of the call to xQueueSend()
portBASE_TYPE SendMapMsg(vtMapStruct *mapData,uint8_t msgType,uint8_t value,uint8_t rightDistance,uint8_t leftDistance,portTickType ticksToBlock);
#endif