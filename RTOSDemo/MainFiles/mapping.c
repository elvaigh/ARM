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
#define vtMapQLen 20 
// actual data structure that is sent in a message
typedef struct __vtMapMsg {
	uint8_t msgType;
	uint8_t value;	 // raidus / wall byte depending on the message type
	uint8_t rightDistance;	 //distance since last change
	uint8_t leftDistance;	 //distance since last change 
} vtMapMsg;

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

/* The map task. */
static portTASK_FUNCTION_PROTO( vMapUpdateTask, pvParameters );

/*-----------------------------------------------------------*/
// Public API
void vStartMapTask(vtMapStruct *params,unsigned portBASE_TYPE uxPriority, vtI2CStruct *i2c,vtLCDStruct *lcd)
{
	// Create the queue that will be used to talk to this task
	if ((params->inQ = xQueueCreate(vtMapQLen,sizeof(vtMapMsg))) == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	/* Start the task */
	portBASE_TYPE retval;
	params->dev = i2c;
	params->lcdData = lcd;
	if ((retval = xTaskCreate( vMapUpdateTask, ( signed char * ) "Mapping", i2cSTACK_SIZE, (void *) params, uxPriority, ( xTaskHandle * ) NULL )) != pdPASS) {
		VT_HANDLE_FATAL_ERROR(retval);
	}
}

portBASE_TYPE SendMapMsg(vtMapStruct *mapData,uint8_t msgType,uint8_t value,uint8_t rightDistance,uint8_t leftDistance,portTickType ticksToBlock)
{
	vtMapMsg mapBuffer;

	if (mapData == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	mapBuffer.msgType = msgType;
	mapBuffer.value = value;
	mapBuffer.rightDistance = rightDistance;
	mapBuffer.leftDistance = leftDistance;
	return(xQueueSend(mapData->inQ,(void *) (&mapBuffer),ticksToBlock));
}

// End of Public API
/*-----------------------------------------------------------*/
int getMsgType(vtMapMsg *Buffer)
{
	return(Buffer->msgType);
}
uint8_t getValue(vtMapMsg *Buffer)
{
	uint8_t val = (uint8_t) Buffer->value;
	return(val);
}
uint8_t getRightDistance(vtMapMsg *Buffer)
{
	uint8_t rd = (uint8_t) Buffer->rightDistance;
	return(rd);
}
uint8_t getLeftDistance(vtMapMsg *Buffer)
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

// This is the actual task that is run
static portTASK_FUNCTION( vMapUpdateTask, pvParameters )
{
	// Define local constants here

	// Get the parameters
	vtMapStruct *param = (vtMapStruct *) pvParameters;
	// Get the I2C device pointer
	vtI2CStruct *devPtr = param->dev;
	// Get the LCD information pointer
	vtLCDStruct *lcdData = param->lcdData;

	// String buffer for printing
	char lcdBuffer[vtLCDMaxLen+1];
	
	// Buffer for receiving messages
	vtMapMsg msgBuffer;

	// Assumes that the I2C device (and thread) have already been initialized


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
		case vtMsgTypeNavMsg: {

			//Send a message to the map telling it what we got
			int value = getValue(&msgBuffer);
			int rightD = getRightDistance(&msgBuffer);
			int leftD = getLeftDistance(&msgBuffer);
		
			sprintf(lcdBuffer,"Values are = %d, %d, %d",lrint(value),lrint(rightD),lrint(leftD));
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

