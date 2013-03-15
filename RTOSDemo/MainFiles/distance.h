#ifndef DISTANCE_TASK_H
#define DISTANCE_TASK_H
#include "vtI2C.h"
#include "lcdTask.h"
// Structure used to pass parameters to the task
// Do not touch...
typedef struct __DistanceStruct {
	vtI2CStruct *dev;
	vtLCDStruct *lcdData;
	xQueueHandle inQ;
} vtDistanceStruct;
// Maximum length of a message that can be received by this task
#define vtDistanceMaxLen   (sizeof(portTickType))


// Public API
//
// Start the task
// Args:
//   distanceData: Data structure used by the task
//   uxPriority -- the priority you want this task to be run at
//   i2c: pointer to the data structure for an i2c task
//   lcd: pointer to the data structure for an LCD task (may be NULL)
void vStartDistanceTask(vtDistanceStruct *distanceData,unsigned portBASE_TYPE uxPriority, vtI2CStruct *i2c,vtLCDStruct *lcd);

/*//
// Send a timer message to the Distance task
// Args:
//   distanceData -- a pointer to a variable of type vtNavLCDStruct
//   ticksElapsed -- number of ticks since the last message (this will be sent in the message)
//   ticksToBlock -- how long the routine should wait if the queue is full
// Return:
//   Result of the call to xQueueSend()
portBASE_TYPE SendDistanceTimerMsg(vtDistanceStruct *distanceData,portTickType ticksElapsed,portTickType ticksToBlock);
*/

//
// Send a value message to the Distance task
// Args:
//   distanceData -- a pointer to a variable of type vtDistanceStruct
//   msgType -- the type of the message to send
//   value -- Either the current speed or a byte containing sensor data depending on message type
//   radius -- The current turning radius
//   rightDistance -- The distance the right wheels have travled
//	 leftDistance -- The distance the left wheels have travled
//   ticksToBlock -- how long the routine should wait if the queue is full
// Return:
//   Result of the call to xQueueSend()
portBASE_TYPE SendDistanceMsg(vtDistanceStruct *distanceData,uint8_t msgType,uint8_t count,uint8_t value1,uint8_t value2,portTickType ticksToBlock);

// A simple routine to use for filling out and sending a message to the Distance method
// Args
//   dev: pointer to the vtI2CStruct data structure
//   msgType: The message type value -- does not get sent on the wire, but is included in the response in the message queue
//   slvAddr: The address of the i2c slave device you are addressing
//   txLen: The number of bytes you want to send
//   txBuf: The buffer holding the bytes you want to send
//   rxLen: The number of bytes that you would like to receive
// Return:
//   Result of the call to xQueueSend()
portBASE_TYPE vtDistanceEnQ(vtDistanceStruct *dev,uint8_t msgType,uint8_t slvAddr,uint8_t txLen,const uint8_t *txBuf,uint8_t rxLen);

#endif