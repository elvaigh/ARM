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
#include "testing.h"
#include "I2CTaskMsgTypes.h"
#include "MotorControl.h"

/* *********************************************** */
// definitions and data structures that are private to this file
// Length of the queue to this task
#define vtNavQLen 20 

#define SAFEZONE 8	       // 8 cm
#define FRONTLARGE 10	   //10 cm
#define FRONTSMALL 5	   // 5 cm
#define ACCELMIN 15		   //Need to change based on real world accelerator values
#define ACCEPTABLEMISS 20  //1 second



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
void vStartNavTaskT(vtNavStruct *params,unsigned portBASE_TYPE uxPriority, vtI2CStruct *i2c,vtLCDStruct *lcd, vtMapStruct *map, vtTestStruct *test)
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
	params->testData = test;
	if ((retval = xTaskCreate( vNavUpdateTask, ( signed char * ) "Navigation", i2cSTACK_SIZE, (void *) params, uxPriority, ( xTaskHandle * ) NULL )) != pdPASS) {
		VT_HANDLE_FATAL_ERROR(retval);
	}
}

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
	uint8_t i2cCmdReadVals[]= {0xAA};
	//John change the speed here
	uint8_t i2cCmdStraight[]= {0x34,0x00,0x1E,0x00};
	uint8_t i2cCmdTurn[]= {0x34,0x00,0x1E,0x00};
	uint8_t i2cCmdHault[] = {0x34,0x00,0x00,0x00};
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
	uint8_t countStartAcc = 0;
	uint8_t countStartMotor = 0;
	uint8_t countStartDistance = 0;
	uint8_t countStartFront = 0;
	uint8_t countAcc = 0;
	uint8_t countMotor = 0;
	uint8_t countMotorCommand = 0;
	uint8_t countDistance = 0;
	uint8_t countFront = 0;

	//counts for missed messages
	uint8_t missedMotor = 0;
	uint8_t missedDistance = 0;

	// Get the parameters
	vtNavStruct *param = (vtNavStruct *) pvParameters;
	// Get the I2C device pointer
	vtI2CStruct *devPtr = param->dev;
	// Get the LCD information pointer
	vtLCDStruct *lcdData = param->lcdData;
	// Get the Map information pointer
	vtMapStruct *mapData = param->mapData;
	// Get the Test information pointer
	vtTestStruct *testData = param->testData;

	// String buffer for printing
	char lcdBuffer[vtLCDMaxLen+1];

	// Buffer for receiving messages
	vtNavMsg msgBuffer;

	unsigned int accelerationTestCount = 0;
	unsigned int straightToTurnTestCount = 0;
	unsigned char motorCount = 0;

	// Assumes that the I2C device (and thread) have already been initialized
	  
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
			uint8_t leftDistance = getLeftDistanceTraveledInMillimeters( &msgBuffer );
			uint8_t rightDistance = getRightDistanceTraveledInMillimeters( &msgBuffer );

			sprintf(lcdBuffer,"Left Distance: %d", leftDistance );
			if (lcdData != NULL) {
				if (SendLCDPrintMsg(lcdData,strnlen(lcdBuffer,vtLCDMaxLen),lcdBuffer,0,portMAX_DELAY) != pdTRUE) {
					VT_HANDLE_FATAL_ERROR(0);
				}
			}

			sprintf( lcdBuffer, "Right Distance: %d", rightDistance );
			if (lcdData != NULL) {
				if (SendLCDPrintMsg(lcdData,strnlen(lcdBuffer,vtLCDMaxLen),lcdBuffer,1,portMAX_DELAY) != pdTRUE) {
					VT_HANDLE_FATAL_ERROR(0);
				}
			}
			break;
		}
		case NavMsgTypeTimer: {
		if( motorCount == 5 ) {
			#if (TEST_STRAIGHT == 1)
				sendMotorCommand( devPtr, TWENTY_CM_S, STRAIGHT );
			#elif (TEST_RIGHT_TURN == 1)
				sendMotorCommand( devPtr, TWENTY_CM_S, RIGHT_TWENTY_CM );
			#elif (TEST_LEFT_TURN == 1)
				sendMotorCommand( devPtr, TWENTY_CM_S, LEFT_TWENTY_CM );
			#elif (TEST_RIGHT_PIVOT	 == 1)
				sendMotorCommand( devPtr, TWENTY_CM_S, PIVOT_RIGHT );
			#elif (TEST_LEFT_PIVOT	== 1)
				sendMotorCommand( devPtr, TWENTY_CM_S, PIVOT_LEFT );
			#elif (TEST_CHANGE_SPEED == 1)
				if( accelerationTestCount < 100 ) {
					sendMotorCommand( devPtr, TEN_CM_S, STRAIGHT );					
				} else if( accelerationTestCount < 200 ) {
					sendMotorCommand( devPtr, THIRTY_CM_S, STRAIGHT );
				} else if( accelerationTestCount < 300 ) {
					sendMotorCommand( devPtr, TEN_CM_S, STRAIGHT );
				} else {
					accelerationTestCount = 300;
					sendMotorCommand( devPtr, HALT, STRAIGHT );
				}
				accelerationTestCount++;
			#elif (TEST_STRAIGHT_TO_TURN == 1)
				if( straightToTurnTestCount < 100 ) {
					sendMotorCommand( devPtr, TWENTY_CM_S, STRAIGHT );
				} else if( straightToTurnTestCount < 200 ) {
					sendMotorCommand( devPtr, TEN_CM_S, RIGHT_TEN_CM );
				} else if( straightToTurnTestCount < 300 ) {
					sendMotorCommand( devPtr, TWENTY_CM_S, STRAIGHT );
				} else if( straightToTurnTestCount < 400 ) {
					sendMotorCommand( devPtr, TWENTY_CM_S, LEFT_TWENTY_CM );
				} else if( straightToTurnTestCount < 500 ) {
					sendMotorCommand( devPtr, THIRTY_CM_S, STRAIGHT );
				} else {
					straightToTurnTestCount = 500;
					sendMotorCommand( devPtr, HALT, STRAIGHT );
				}
				straightToTurnTestCount++;
			#endif
			motorCount = 0;
			}
			motorCount++;
			if (vtI2CEnQ(devPtr,NavMsgTypeTimer,0x4F,sizeof(i2cCmdReadVals),i2cCmdReadVals,4) != pdTRUE) {
				VT_HANDLE_FATAL_ERROR(0);
			}

			break;
		}
		default: {
			//printf("invalid data type");
			//VT_HANDLE_FATAL_ERROR(getMsgType(&msgBuffer));
			break;
		}
		}


	}
}

