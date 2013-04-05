#ifndef NAV_TASK_H
#define NAV_TASK_H
#include "vtI2C.h"
#include "lcdTask.h"
#include "mapping.h"
#include "testing.h"
// Structure used to pass parameters to the task
// Do not touch...
typedef struct __NavStruct {
	vtI2CStruct *dev;
	vtLCDStruct *lcdData;
	vtMapStruct *mapData;
	vtTestStruct *testData;
	xQueueHandle inQ;
} vtNavStruct;
// Maximum length of a message that can be received by this task
#define vtNavMaxLen   (sizeof(portTickType))

// Public API
//
// Start the task
// Args:
//   navData: Data structure used by the task
//   uxPriority -- the priority you want this task to be run at
//   i2c: pointer to the data structure for an i2c task
//   lcd: pointer to the data structure for an LCD task (may be NULL)
//   map: pointer to the data structure for a map task
void vStartNavTask(vtNavStruct *navData,unsigned portBASE_TYPE uxPriority, vtI2CStruct *i2c,vtLCDStruct *lcd,vtMapStruct *map,vtTestStruct *test);
//
// Send a timer message to the Navigation task
// Args:
//   navData -- a pointer to a variable of type vtNavLCDStruct
//   ticksElapsed -- number of ticks since the last message (this will be sent in the message)
//   ticksToBlock -- how long the routine should wait if the queue is full
// Return:
//   Result of the call to xQueueSend()
portBASE_TYPE SendNavTimerMsg(vtNavStruct *navData,portTickType ticksElapsed,portTickType ticksToBlock);

//
// Send a value message to the Navigation task
// Args:
//   navData -- a pointer to a variable of type vtNavStruct
//   msgType -- the type of the message to send
//   value -- Either the current speed or a byte containing sensor data depending on message type
//   radius -- The current turning radius
//   rightDistance -- The distance the right wheels have travled
//	 leftDistance -- The distance the left wheels have travled
//   ticksToBlock -- how long the routine should wait if the queue is full
// Return:
//   Result of the call to xQueueSend()
portBASE_TYPE SendNavMsg(vtNavStruct *navData,uint8_t msgType,uint8_t count,uint8_t value1,uint8_t value2,portTickType ticksToBlock);

void updateRun(uint8_t runNum);
#endif