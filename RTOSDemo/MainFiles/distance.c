#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "projdefs.h"
#include "semphr.h"

/* include files. */
#include "vtUtilities.h"
#include "vtI2C.h"
#include "LCDtask.h"
#include "I2CTaskMsgTypes.h"
#include "distance.h"

/* *********************************************** */
// definitions and data structures that are private to this file
// Length of the queue to this task
#define vtDistanceQLen 20 
// actual data structure that is sent in a message
typedef struct __vtDistanceMsg {
	uint8_t msgType;
	uint8_t count;	 
	uint8_t value1;	 
	uint8_t value2;
} vtDistanceMsg;

typedef struct __vtDistanceI2CMsg {
	uint8_t msgType; // A field you will likely use in your communications between processors (and for debugging)
	uint8_t slvAddr; // Address of the device to whom the message is being sent (or was sent)
	uint8_t	rxLen;	 // Length of the message you *expect* to receive (or, on the way back, the length that *was* received)
	uint8_t txLen;   // Length of the message you want to sent (or, on the way back, the length that *was* sent)
	uint8_t status;  // status of the completed operation -- I've not done anything much here, you probably should...
	uint8_t buf[vtI2CMLen]; // On the way in, message to be sent, on the way out, message received (if any)
} vtDistanceI2CMsg;

// I have set this to a large stack size because of (a) using printf() and (b) the depth of function calls
//   for some of the i2c operations	-- almost certainly too large, see LCDTask.c for details on how to check the size
#define baseStack 3
#if PRINTF_VERSION == 1
#define distanceSTACK_SIZE		((baseStack+10)*configMINIMAL_STACK_SIZE)
#else
#define distanceSTACK_SIZE		((baseStack+10)*configMINIMAL_STACK_SIZE)
#endif

#define PRINTGRAPH 0


// end of defs
/* *********************************************** */

/* The distance task. */
static portTASK_FUNCTION_PROTO( vDistanceUpdateTask, pvParameters );

/*-----------------------------------------------------------*/
// Public API
void vStartDistanceTask(vtDistanceStruct *params,unsigned portBASE_TYPE uxPriority, vtI2CStruct *i2c,vtLCDStruct *lcd)
{
	// Create the queue that will be used to talk to this task
	if ((params->inQ = xQueueCreate(vtDistanceQLen,sizeof(vtDistanceI2CMsg))) == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	/* Start the task */
	portBASE_TYPE retval;
	params->dev = i2c;
	params->lcdData = lcd;
	if ((retval = xTaskCreate( vDistanceUpdateTask, ( signed char * ) "Distance", distanceSTACK_SIZE, (void *) params, uxPriority, ( xTaskHandle * ) NULL )) != pdPASS) {
		VT_HANDLE_FATAL_ERROR(retval);
	}
}

/*
portBASE_TYPE SendDistanceTimerMsg(vtDistanceStruct *distanceData,portTickType ticksElapsed,portTickType ticksToBlock)
{
	if (distanceData == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	vtDistanceI2CMsg distanceBuffer;
	distanceBuffer.msgType = DistanceMsgTypeTimer;
	return(xQueueSend(distanceData->inQ,(void *) (&distanceBuffer),ticksToBlock));
}
*/

portBASE_TYPE SendDistanceMsg(vtDistanceStruct *distanceData,uint8_t msgType,uint8_t count,uint8_t val1,uint8_t val2,portTickType ticksToBlock)
{
	vtDistanceMsg distanceBuffer;

	if (distanceData == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	distanceBuffer.msgType = msgType;
	distanceBuffer.count = count;
	distanceBuffer.value1 = val1;
	distanceBuffer.value2 = val2;
	return(xQueueSend(distanceData->inQ,(void *) (&distanceBuffer),ticksToBlock));
}
// A simple routine to use for filling out and sending a message to the I2C thread
//   You may want to make your own versions of these as they are not suited to all purposes
portBASE_TYPE vtDistanceEnQ(vtDistanceStruct *dev,uint8_t msgType,uint8_t slvAddr,uint8_t txLen,const uint8_t *txBuf,uint8_t rxLen)
{
	vtDistanceI2CMsg msgBuf;
	int i;

    msgBuf.slvAddr = slvAddr;
	msgBuf.msgType = msgType;
	msgBuf.rxLen = rxLen;
	if (msgBuf.rxLen > vtI2CMLen) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	msgBuf.txLen = txLen;
	if (msgBuf.txLen > vtI2CMLen) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	for (i=0;i<msgBuf.txLen;i++) {
		msgBuf.buf[i] = txBuf[i];
	}
	return(xQueueSend(dev->inQ,(void *) (&msgBuf),portMAX_DELAY));
}

// End of Public API
/*-----------------------------------------------------------*/
int getDistanceMsgType(vtDistanceI2CMsg *Buffer)
{
	return(Buffer->msgType);
}
uint8_t getDistanceCount(vtDistanceI2CMsg *Buffer)
{
	uint8_t val = (uint8_t) Buffer->buf[1];
	return(val);
}
uint8_t getDistanceVal1(vtDistanceI2CMsg *Buffer)
{
	uint8_t rad = (uint8_t) Buffer->buf[2];
	return(rad);
}
uint8_t getDistanceVal2(vtDistanceI2CMsg *Buffer)
{
	uint8_t rd = (uint8_t) Buffer->buf[3];
	return(rd);
}


// I2C commands for the Motor Encoder
	uint8_t i2cCmdDistance[] = {0x0A,0x00,0x00,0x00};
	uint8_t i2cCmdAccelVal[] = {0x0B,0x00,0x00,0x00};
// end of I2C command definitions


// This is the actual task that is run
static portTASK_FUNCTION( vDistanceUpdateTask, pvParameters )
{
	// Define local constants here
	uint8_t countStartIR1 = 0;
	uint8_t countStartIR2 = 0;
	uint8_t countStartIR3 = 0;
	uint8_t countIR1 = 0;
	uint8_t countIR2 = 0;
	uint8_t countIR3 = 0;
	uint8_t countDist = 0;

	// Get the parameters
	vtDistanceStruct *param = (vtDistanceStruct *) pvParameters;
	// Get the I2C device pointer
	vtI2CStruct *devPtr = param->dev;
	// Get the LCD information pointer
	vtLCDStruct *lcdData = param->lcdData;

	// String buffer for printing
	char lcdBuffer[vtLCDMaxLen+1];

	// Buffer for receiving messages
	vtDistanceI2CMsg msgBuffer;

	// Assumes that the I2C device (and thread) have already been initialized

	// Like all good tasks, this should never exit
	for(;;)
	{
		// Wait for a message from either a timer or from an I2C operation
		if (xQueueReceive(param->inQ,(void *) &msgBuffer,portMAX_DELAY) != pdTRUE) {
			VT_HANDLE_FATAL_ERROR(0);
		}

		// Now, based on the type of the message and the state, we decide on the new state and action to take
		switch(getDistanceMsgType(&msgBuffer)) {
		case vtI2CMsgTypeIRRead1: {

			int msgCount = getDistanceCount(&msgBuffer);
			int val1 = getDistanceVal1(&msgBuffer);
			int val2 = getDistanceVal2(&msgBuffer);
			//Strumsky is this how you sent the 10 bit value?
			// val1 = 0000 00(10)(9)
			// val2 = 8765 4321
			//if so the below undoes it and puts it back together
			//piece together 10 bit value
			int value = val1* 256 + val2;

			if(countStartIR1 == 0)
			{
				countStartIR1 = 1;
				countIR1 = msgCount;
			}
			else{
				if((countIR1 + 1) == msgCount)
				{
					countIR1 = msgCount;	
				}
				else{

				//Strumsky do something here if the count isn't continuous

					/*sprintf(lcdBuffer,"D IR1: %d %d",countIR1,msgCount);
					if (lcdData != NULL) {
						if (SendLCDPrintMsg(lcdData,strnlen(lcdBuffer,vtLCDMaxLen),lcdBuffer,6,portMAX_DELAY) != pdTRUE) {
							VT_HANDLE_FATAL_ERROR(0);
						}
					}*/
					countIR1 = msgCount;
				}
			}
			sprintf(lcdBuffer,"IR1: %d",value);
			if (lcdData != NULL) {
				if (SendLCDPrintMsg(lcdData,strnlen(lcdBuffer,vtLCDMaxLen),lcdBuffer,0,portMAX_DELAY) != pdTRUE) {
					VT_HANDLE_FATAL_ERROR(0);
				}
			}

			//send back a distance message
			//Sturmsky I don't know what method you want to put this in but this is how you send back a 
			//distance command 
			i2cCmdDistance[1] = countDist;
			countDist++;
			//you need to update the values
			//i2cCmdDistance[2] = left
			//i2cCmdDistance[3] = right

			if (vtI2CConQ(devPtr,DistanceMsg,0x4F,sizeof(i2cCmdDistance),i2cCmdDistance,sizeof(i2cCmdDistance)) != pdTRUE) {
				VT_HANDLE_FATAL_ERROR(0);
			}
			//end send of distance command

			break;
		}
		case vtI2CMsgTypeIRRead2: {

			int msgCount = getDistanceCount(&msgBuffer);
			int val1 = getDistanceVal1(&msgBuffer);
			int val2 = getDistanceVal2(&msgBuffer);
			//Strumsky is this how you sent the 10 bit value?
			// val1 = 0000 00(10)(9)
			// val2 = 8765 4321
			//if so the below undoes it and puts it back together
			//piece together 10 bit value
			int value = val1* 256 + val2;

			if(countStartIR2 == 0)
			{
				countStartIR2 = 1;
				countIR2 = msgCount;
			}
			else{
				if((countIR2 + 1) == msgCount)
				{
					countIR2 = msgCount;	
				}
				else{
				//Strumsky do something here for dropped packets
				/*
					sprintf(lcdBuffer,"Dropped IR2");
					if (lcdData != NULL) {
						if (SendLCDPrintMsg(lcdData,strnlen(lcdBuffer,vtLCDMaxLen),lcdBuffer,6,portMAX_DELAY) != pdTRUE) {
							VT_HANDLE_FATAL_ERROR(0);
						}
					}*/
					countIR2 = msgCount;
				}
			}

			sprintf(lcdBuffer,"IR2:%d",value);
			if (lcdData != NULL) {
				if (SendLCDPrintMsg(lcdData,strnlen(lcdBuffer,vtLCDMaxLen),lcdBuffer,1,portMAX_DELAY) != pdTRUE) {
					VT_HANDLE_FATAL_ERROR(0);
				}
			}
			
			break;
		}
		case vtI2CMsgTypeIRRead3: {

			int msgCount = getDistanceCount(&msgBuffer);
			int val1 = getDistanceVal1(&msgBuffer);
			int val2 = getDistanceVal2(&msgBuffer);
			//Strumsky is this how you sent the 10 bit value?
			// val1 = 0000 00(10)(9)
			// val2 = 8765 4321
			//if so the below undoes it and puts it back together
			//piece together 10 bit value
			int value = val1* 256 + val2;

			if(countStartIR3 == 0)
			{
				countStartIR3 = 1;
				countIR3 = msgCount;
			}
			else{
				if((countIR3 + 1) == msgCount)
				{
					countIR3 = msgCount;	
				}
				else{
				//Strumsky do something here for dropped packets
				/*
					sprintf(lcdBuffer,"Dropped IR3");
					if (lcdData != NULL) {
						if (SendLCDPrintMsg(lcdData,strnlen(lcdBuffer,vtLCDMaxLen),lcdBuffer,6,portMAX_DELAY) != pdTRUE) {
							VT_HANDLE_FATAL_ERROR(0);
						}
					} */
					countIR3 = msgCount;
				}
			}

			sprintf(lcdBuffer,"IR3:%d",value);
			if (lcdData != NULL) {
				if (SendLCDPrintMsg(lcdData,strnlen(lcdBuffer,vtLCDMaxLen),lcdBuffer,2,portMAX_DELAY) != pdTRUE) {
					VT_HANDLE_FATAL_ERROR(0);
				}
			}
			break;
		}

		default: {
			//VT_HANDLE_FATAL_ERROR(getMsgType(&msgBuffer));
			break;
		}
		}


	}
}

