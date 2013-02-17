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
	uint8_t speed;	 // speed the rover is going
	uint8_t radius;	 // turning radius of the rover
	uint8_t rightDistance;
	uint8_t leftDistance;
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


portBASE_TYPE SendNavMsg(vtNavStruct *navData,uint8_t msgType,uint8_t value,uint8_t radius,uint8_t rightDistance,uint8_t leftDistance,portTickType ticksToBlock)
{
	vtNavMsg navBuffer;

	if (navData == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	navBuffer.msgType = msgType;
	navBuffer.speed = value;
	navBuffer.radius = radius;
	navBuffer.rightDistance = rightDistance;
	navBuffer.leftDistance = leftDistance;
	return(xQueueSend(navData->inQ,(void *) (&navBuffer),ticksToBlock));
}

// End of Public API
/*-----------------------------------------------------------*/
int getMsgType(vtNavMsg *Buffer)
{
	return(Buffer->msgType);
}
uint8_t getValue(vtNavMsg *Buffer)
{
	uint8_t val = (uint8_t) Buffer->speed;
	return(val);
}
uint8_t getRadius(vtNavMsg *Buffer)
{
	uint8_t rad = (uint8_t) Buffer->radius;
	return(rad);
}
uint8_t getRightDistance(vtNavMsg *Buffer)
{
	uint8_t rd = (uint8_t) Buffer->rightDistance;
	return(rd);
}
uint8_t getLeftDistance(vtNavMsg *Buffer)
{
	uint8_t ld = (uint8_t) Buffer->leftDistance;
	return(ld);
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
	const uint8_t i2cCmdStraight[]= {0xAA};
// end of I2C command definitions

// Definitions of the states for the FSM below
const uint8_t fsmStateClear = 0;
const uint8_t fsmStateWallFront = 1;
const uint8_t fsmStateWallRight = 2;	  
const uint8_t fsmStateWallLeft = 3;
const uint8_t fsmStateWallFrontRight = 4;
const uint8_t fsmStateWallFrontLeft = 5;
const uint8_t fsmStateWallFrontRightLeft = 6;
const uint8_t fsmStateWallALL = 7;
// This is the actual task that is run
static portTASK_FUNCTION( vNavUpdateTask, pvParameters )
{
	// Define local constants here

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
	char msgString[vtLCDMaxLen+1];
	char *msgChar;
	// Buffer for receiving messages
	vtNavMsg msgBuffer;
	uint8_t currentState;

	// Assumes that the I2C device (and thread) have already been initialized

	// This task is implemented as a Finite State Machine.  The incoming messages are examined to see
	//   whether or not the state should change.
	
	currentState = fsmStateClear;

	#if PRINTGRAPH == 1
	if (SendLCDLine(lcdData,30,200,320,200,portMAX_DELAY) != pdTRUE) {
						VT_HANDLE_FATAL_ERROR(0);
 	}
	if (SendLCDLine(lcdData,30,10,30,200,portMAX_DELAY) != pdTRUE) {
						VT_HANDLE_FATAL_ERROR(0);
 	} 
	#endif
	  
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
			int speed = getValue(&msgBuffer);
			int radius = getRadius(&msgBuffer);
			int rightD = getRightDistance(&msgBuffer);
			int leftD = getLeftDistance(&msgBuffer);
		
			if (SendMapMsg(mapData,MapMessageTurn,radius,rightD,leftD,portMAX_DELAY) != pdTRUE) {
						VT_HANDLE_FATAL_ERROR(0);
					}
			break;
		}
		case vtI2CMsgTypeSensorRead: {

			//For now just send back a command to go straight
			if (vtI2CEnQ(devPtr,vtI2CMsgTypeMotorRead,0x4F,sizeof(i2cCmdStraight),i2cCmdStraight,0) != pdTRUE) {
				VT_HANDLE_FATAL_ERROR(0);
			}
			break;
		}
		case NavMsgTypeTimer: {
			if (vtI2CEnQ(devPtr,NavMsgTypeTimer,0x4F,sizeof(i2cCmdReadVals),i2cCmdReadVals,1) != pdTRUE) {
				VT_HANDLE_FATAL_ERROR(0);
			}

			sprintf(lcdBuffer,"Timer Messages");
			if (lcdData != NULL) {
				if (SendLCDPrintMsg(lcdData,strnlen(lcdBuffer,vtLCDMaxLen),lcdBuffer,2,portMAX_DELAY) != pdTRUE) {
					VT_HANDLE_FATAL_ERROR(0);
				}
			}
			 
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

