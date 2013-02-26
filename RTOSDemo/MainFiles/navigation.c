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
#include "navigation.h"
#include "mapping.h"
#include "I2CTaskMsgTypes.h"

/* *********************************************** */
// definitions and data structures that are private to this file
// Length of the queue to this task
#define vtNavQLen 20 
// actual data structure that is sent in a message
typedef struct __vtNavMsg {
	uint8_t msgType;
	uint8_t count;	 
	uint8_t value1;	 
	uint8_t value2;
} vtNavMsg;

// I have set this to a large stack size because of (a) using printf() and (b) the depth of function calls
//   for some of the i2c operations	-- almost certainly too large, see LCDTask.c for details on how to check the size
#define baseStack 3
#if PRINTF_VERSION == 1
#define i2cSTACK_SIZE		((baseStack+5)*configMINIMAL_STACK_SIZE)
#else
#define i2cSTACK_SIZE		(baseStack*configMINIMAL_STACK_SIZE)
#endif

#define PRINTGRAPH 0


// end of defs
/* *********************************************** */

/* The nav task. */
static portTASK_FUNCTION_PROTO( vNavUpdateTask, pvParameters );

/*-----------------------------------------------------------*/
// Public API
void vStartNavTask(vtNavStruct *params,unsigned portBASE_TYPE uxPriority, vtI2CStruct *i2c,vtLCDStruct *lcd, vtMapStruct *map)
{
	// Create the queue that will be used to talk to this task
	if ((params->inQ = xQueueCreate(vtNavQLen,sizeof(vtNavMsg))) == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	/* Start the task */
	portBASE_TYPE retval;
	params->dev = i2c;
	params->lcdData = lcd;
	params->mapData = map;
	if ((retval = xTaskCreate( vNavUpdateTask, ( signed char * ) "Navigation", i2cSTACK_SIZE, (void *) params, uxPriority, ( xTaskHandle * ) NULL )) != pdPASS) {
		VT_HANDLE_FATAL_ERROR(retval);
	}
}

portBASE_TYPE SendNavTimerMsg(vtNavStruct *navData,portTickType ticksElapsed,portTickType ticksToBlock)
{
	if (navData == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	vtNavMsg navBuffer;
	navBuffer.msgType = NavMsgTypeTimer;
	return(xQueueSend(navData->inQ,(void *) (&navBuffer),ticksToBlock));
}


portBASE_TYPE SendNavMsg(vtNavStruct *navData,uint8_t msgType,uint8_t count,uint8_t val1,uint8_t val2,portTickType ticksToBlock)
{
	vtNavMsg navBuffer;

	if (navData == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	navBuffer.msgType = msgType;
	navBuffer.count = count;
	navBuffer.value1 = val1;
	navBuffer.value2 = val2;
	return(xQueueSend(navData->inQ,(void *) (&navBuffer),ticksToBlock));
}

portBASE_TYPE SendSensorTimerMsg(vtNavStruct *navData,portTickType ticksElapsed,portTickType ticksToBlock)
{
	if (navData == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	vtNavMsg navBuffer;
	navBuffer.msgType = SensorMsgTypeTimer;
	return(xQueueSend(navData->inQ,(void *) (&navBuffer),ticksToBlock));
}

// End of Public API
/*-----------------------------------------------------------*/
int getMsgType(vtNavMsg *Buffer)
{
	return(Buffer->msgType);
}
uint8_t getCount(vtNavMsg *Buffer)
{
	uint8_t val = (uint8_t) Buffer->count;
	return(val);
}
uint8_t getVal1(vtNavMsg *Buffer)
{
	uint8_t rad = (uint8_t) Buffer->value1;
	return(rad);
}
uint8_t getVal2(vtNavMsg *Buffer)
{
	uint8_t rd = (uint8_t) Buffer->value2;
	return(rd);
}

/* I2C commands for the temperature sensor
	const uint8_t i2cCmdInit[]= {0xAC,0x00};
	const uint8_t i2cCmdStartConvert[]= {0xEE};
	const uint8_t i2cCmdStopConvert[]= {0x22};
	const uint8_t i2cCmdReadVals[]= {0xAA};
	const uint8_t i2cCmdReadCnt[]= {0xA8};
	const uint8_t i2cCmdReadSlope[]= {0xA9};
// end of I2C command definitions */

// I2C commands for the Motor Encoder
	const uint8_t i2cCmdReadVals[]= {0xAA};
	const uint8_t i2cCmdStraight[]= {0x80,0x80};
// end of I2C command definitions

// Definitions of the states for the FSM below
const uint8_t fsmStateStraight = 0;
const uint8_t fsmStateTurnLeft = 1;
const uint8_t fsmStateTurnRight = 2;	  
const uint8_t fsmStateHault = 3;

// This is the actual task that is run
static portTASK_FUNCTION( vNavUpdateTask, pvParameters )
{
	// Define local constants here
	int countStartIR1 = 0;
	int countStartIR2 = 0;
	int countStartIR3 = 0;
	int countStartAcc = 0;
	int countStartMotor = 0;
	int countIR1 = 0;
	int countIR2 = 0;
	int countIR3 = 0;
	int countAcc = 0;
	int countMotor = 0;
	// Get the parameters
	vtNavStruct *param = (vtNavStruct *) pvParameters;
	// Get the I2C device pointer
	vtI2CStruct *devPtr = param->dev;
	// Get the LCD information pointer
	vtLCDStruct *lcdData = param->lcdData;
	// Get the Map information pointer
	vtMapStruct *mapData = param->mapData;

	// String buffer for printing
	char lcdBuffer[vtLCDMaxLen+1];

	// Buffer for receiving messages
	vtNavMsg msgBuffer;
	uint8_t currentState;

	// Assumes that the I2C device (and thread) have already been initialized

	// This task is implemented as a Finite State Machine.  The incoming messages are examined to see
	//   whether or not the state should change.
	
	currentState = fsmStateStraight;
	  
	// Like all good tasks, this should never exit
	for(;;)
	{
		// Wait for a message from either a timer or from an I2C operation
		if (xQueueReceive(param->inQ,(void *) &msgBuffer,portMAX_DELAY) != pdTRUE) {
			VT_HANDLE_FATAL_ERROR(0);
		}

		// Now, based on the type of the message and the state, we decide on the new state and action to take
		switch(getMsgType(&msgBuffer)) {
		case vtI2CMsgTypeMotorRead: {

			//Send a message to the map telling it what we got
			int msgCount = getCount(&msgBuffer);
			int val1 = getVal1(&msgBuffer);
			int val2 = getVal2(&msgBuffer);

		
			if (SendMapMsg(mapData,MapMessageMotor,msgCount,val1,val2,portMAX_DELAY) != pdTRUE) {
						VT_HANDLE_FATAL_ERROR(0);
					}
			
	
			sprintf(lcdBuffer,"Receiving");
			if (lcdData != NULL) {
				if (SendLCDPrintMsg(lcdData,strnlen(lcdBuffer,vtLCDMaxLen),lcdBuffer,1,portMAX_DELAY) != pdTRUE) {
					VT_HANDLE_FATAL_ERROR(0);
				}
			}
			break;
		}
		case vtI2CMsgTypeIRRead1: {

			int msgCount = getCount(&msgBuffer);
			int val1 = getVal1(&msgBuffer);
			int val2 = getVal2(&msgBuffer);

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
					sprintf(lcdBuffer,"Dropped IR1");
					if (lcdData != NULL) {
						if (SendLCDPrintMsg(lcdData,strnlen(lcdBuffer,vtLCDMaxLen),lcdBuffer,6,portMAX_DELAY) != pdTRUE) {
							VT_HANDLE_FATAL_ERROR(0);
						}
					}
					countIR1 = msgCount;
				}
			}

			sprintf(lcdBuffer,"Receiving IR1");
			if (lcdData != NULL) {
				if (SendLCDPrintMsg(lcdData,strnlen(lcdBuffer,vtLCDMaxLen),lcdBuffer,2,portMAX_DELAY) != pdTRUE) {
					VT_HANDLE_FATAL_ERROR(0);
				}
			}
			//For now just send back a command to go straight
			if (vtI2CEnQ(devPtr,vtI2CMsgTypeMotorRead,0x4F,sizeof(i2cCmdStraight),i2cCmdStraight,0) != pdTRUE) {
				VT_HANDLE_FATAL_ERROR(0);
			}
			break;
		}
		case vtI2CMsgTypeIRRead2: {

			int msgCount = getCount(&msgBuffer);
			int val1 = getVal1(&msgBuffer);
			int val2 = getVal2(&msgBuffer);

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
					sprintf(lcdBuffer,"Dropped IR2");
					if (lcdData != NULL) {
						if (SendLCDPrintMsg(lcdData,strnlen(lcdBuffer,vtLCDMaxLen),lcdBuffer,6,portMAX_DELAY) != pdTRUE) {
							VT_HANDLE_FATAL_ERROR(0);
						}
					}
					countIR2 = msgCount;
				}
			}

			sprintf(lcdBuffer,"Receiving IR2");
			if (lcdData != NULL) {
				if (SendLCDPrintMsg(lcdData,strnlen(lcdBuffer,vtLCDMaxLen),lcdBuffer,2,portMAX_DELAY) != pdTRUE) {
					VT_HANDLE_FATAL_ERROR(0);
				}
			}
			//For now just send back a command to go straight
			if (vtI2CEnQ(devPtr,vtI2CMsgTypeMotorRead,0x4F,sizeof(i2cCmdStraight),i2cCmdStraight,0) != pdTRUE) {
				VT_HANDLE_FATAL_ERROR(0);
			}
			break;
		}
		case vtI2CMsgTypeIRRead3: {

			int msgCount = getCount(&msgBuffer);
			int val1 = getVal1(&msgBuffer);
			int val2 = getVal2(&msgBuffer);

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
					sprintf(lcdBuffer,"Dropped IR3");
					if (lcdData != NULL) {
						if (SendLCDPrintMsg(lcdData,strnlen(lcdBuffer,vtLCDMaxLen),lcdBuffer,6,portMAX_DELAY) != pdTRUE) {
							VT_HANDLE_FATAL_ERROR(0);
						}
					}
					countIR3 = msgCount;
				}
			}

			sprintf(lcdBuffer,"Receiving IR3");
			if (lcdData != NULL) {
				if (SendLCDPrintMsg(lcdData,strnlen(lcdBuffer,vtLCDMaxLen),lcdBuffer,2,portMAX_DELAY) != pdTRUE) {
					VT_HANDLE_FATAL_ERROR(0);
				}
			}
			//For now just send back a command to go straight
			if (vtI2CEnQ(devPtr,vtI2CMsgTypeMotorRead,0x4F,sizeof(i2cCmdStraight),i2cCmdStraight,0) != pdTRUE) {
				VT_HANDLE_FATAL_ERROR(0);
			}
			break;
		}
		case vtI2CMsgTypeAccRead: {

			int msgCount = getCount(&msgBuffer);
			int val1 = getVal1(&msgBuffer);
			int val2 = getVal2(&msgBuffer);

			sprintf(lcdBuffer,"Receiving Acc");
			if (lcdData != NULL) {
				if (SendLCDPrintMsg(lcdData,strnlen(lcdBuffer,vtLCDMaxLen),lcdBuffer,1,portMAX_DELAY) != pdTRUE) {
					VT_HANDLE_FATAL_ERROR(0);
				}
			}
			//For now just send back a command to go straight
			if (vtI2CEnQ(devPtr,vtI2CMsgTypeMotorSend,0x4F,sizeof(i2cCmdStraight),i2cCmdStraight,0) != pdTRUE) {
				VT_HANDLE_FATAL_ERROR(0);
			}
			if(countStartAcc == 0)
			{
				countStartAcc = 1;
				countAcc = msgCount;
			}
			else{
				if((countAcc + 1) == msgCount)
				{
					countAcc = msgCount;	
				}
				else{
					sprintf(lcdBuffer,"Dropped Acc");
					if (lcdData != NULL) {
						if (SendLCDPrintMsg(lcdData,strnlen(lcdBuffer,vtLCDMaxLen),lcdBuffer,6,portMAX_DELAY) != pdTRUE) {
							VT_HANDLE_FATAL_ERROR(0);
						}
					}
					countAcc = msgCount;
				}
			}
			break;
		}
		case NavMsgTypeTimer: {
			if (vtI2CEnQ(devPtr,NavMsgTypeTimer,0x4F,sizeof(i2cCmdReadVals),i2cCmdReadVals,4) != pdTRUE) {
				VT_HANDLE_FATAL_ERROR(0);
			}
			
			/*sprintf(lcdBuffer,"Timer Messages");
			if (lcdData != NULL) {
				if (SendLCDPrintMsg(lcdData,strnlen(lcdBuffer,vtLCDMaxLen),lcdBuffer,6,portMAX_DELAY) != pdTRUE) {
					VT_HANDLE_FATAL_ERROR(0);
				}
			} */
			 
			break;
		}
		default: {
			printf("invalid data type");
			VT_HANDLE_FATAL_ERROR(getMsgType(&msgBuffer));
			break;
		}
		}


	}
}

