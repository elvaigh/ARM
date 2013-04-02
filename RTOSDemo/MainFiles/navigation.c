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

/* *********************************************** */
// definitions and data structures that are private to this file
// Length of the queue to this task
#define vtNavQLen 20 

#define SAFEZONE 8

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

#define HAULT 0
#define STRAIGHT 1
#define LEFT 2
#define RIGHT 3

#define SMALLRAID  20
#define LARGERAID  60

#define USEMAPPING 0

uint8_t RUN = 1;

// end of defs
/* *********************************************** */

/* The nav task. */
static portTASK_FUNCTION_PROTO( vNavUpdateTask, pvParameters );

/*-----------------------------------------------------------*/
// Public API
void vStartNavTask(vtNavStruct *params,unsigned portBASE_TYPE uxPriority, vtI2CStruct *i2c,vtLCDStruct *lcd, vtMapStruct *map, vtTestStruct *test)
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
void updateRun(uint8_t runNum)
{
	RUN = runNum;
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
	uint8_t i2cCmdStraight[]= {0x34,0x00,0x01,0x00};
	uint8_t i2cCmdTurn[]= {0x34,0x00,0x01,0x00};
	uint8_t i2cCmdHault[] = {0x34,0x00,0x00,0x00};
// end of I2C command definitions

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

	//used to know when to tell the map we changed state
	uint8_t curState = HAULT;
	uint8_t curRaid = 0;

	float distanceF = 0.0;

	// Assumes that the I2C device (and thread) have already been initialized

	// This task is implemented as a Finite State Machine.  The incoming messages are examined to see
	//   whether or not the state should change.
	  
	// Like all good tasks, this should never exit
	for(;;)
	{
		// Wait for a message from either a timer or from an I2C operation
		if (xQueueReceive(param->inQ,(void *) &msgBuffer,portMAX_DELAY) != pdTRUE) {
			VT_HANDLE_FATAL_ERROR(0);
		}

		// Now, based on the type of the message and the state, we decide on the new state and action to take
		switch(getMsgType(&msgBuffer)) {
		case DistanceMsg: {
			int msgCount = getCount(&msgBuffer);
			int val1 = getVal1(&msgBuffer);
			int val2 = getVal2(&msgBuffer);
			if(countStartDistance == 0)
			{
				countStartDistance = 1;
				countDistance = msgCount;
			}
			else{
				if((countDistance + 1) == msgCount)
				{
					countDistance = msgCount;	
				}
				else{
					/*sprintf(lcdBuffer,"D IR1: %d %d",countDistance,msgCount);
					if (lcdData != NULL) {
						if (SendLCDPrintMsg(lcdData,strnlen(lcdBuffer,vtLCDMaxLen),lcdBuffer,6,portMAX_DELAY) != pdTRUE) {
							VT_HANDLE_FATAL_ERROR(0);
						}
					}*/
					countDistance = msgCount;
				}
			}
			
			i2cCmdTurn[1] = countMotorCommand;
			countMotorCommand++;

			if((val1 < SAFEZONE) || (val2 < SAFEZONE))
			{
				distanceF = (float)(val1-val2)/(float)(val1+val2);
				//set response
				//127 for straight
				//10 for small left (15-100) cm
				//11 for hard left (<15) cm
				//20 for small right (15-100) cm
				//21 for hard right (<15) cm
				// 0 = spin left
				// 128 = spin right
				// anything between 0 and 126 = turn left with a turn radius of (val2)
				// anything between 128 and 255 = turn right with a turn radius of (val2 + 128)
	
				//hard Left (SMALLRAID)
				if(distanceF > 0.4)
				{
					i2cCmdTurn[3] = SMALLRAID;
					//SMALLRAID is radius
					#if(USEMAPPING == 1)
					if((curState != LEFT) || (curRaid != SMALLRAID))
					{
						if (SendMapMsg(mapData,MapTurnLeft,0,0,SMALLRAID,portMAX_DELAY) != pdTRUE) {
							VT_HANDLE_FATAL_ERROR(0);
						}
						curState = LEFT;
						curRaid = SMALLRAID;
					}
					#endif
				}
				//small Left
				else if(distanceF > 0)
				{
					i2cCmdTurn[3] = LARGERAID;
					//20 is radius
					#if(USEMAPPING == 1)
					if((curState != LEFT) || (curRaid != LARGERAID))
					{
						if (SendMapMsg(mapData,MapTurnLeft,0,0,LARGERAID,portMAX_DELAY) != pdTRUE) {
							VT_HANDLE_FATAL_ERROR(0);
						}
						curState = LEFT;
						curRaid = 20;
					}
					#endif
				}
				//127 = straight
				else if(distanceF == 0)
				{
					i2cCmdTurn[3] = 127;
					//255 is radius
					#if(USEMAPPING == 1)
					if((curState != STRAIGHT))
					{
						if (SendMapMsg(mapData,MapStraight,0,0,127,portMAX_DELAY) != pdTRUE) {
							VT_HANDLE_FATAL_ERROR(0);
						}
						curState = STRAIGHT;
						curRaid = 127;
					}
					#endif
				}
				//hard Right (128+SMALLRAID)
				else if(distanceF < -0.4)
				{
					i2cCmdTurn[3] = 128 + SMALLRAID;
					#if(USEMAPPING == 1)
					if((curState != RIGHT)||(curRaid != SMALLRAID))
					{
						//10 is radius
						if (SendMapMsg(mapData,MapTurnRight,0,0,SMALLRAID,portMAX_DELAY) != pdTRUE) {
							VT_HANDLE_FATAL_ERROR(0);
						}
						curState = RIGHT;
						curRaid = 10;
					}
					#endif
				}
				//small Right (128+LARGERAID)
				else
				{
					i2cCmdTurn[3] = 128 + LARGERAID;
					#if(USEMAPPING == 1)
					if((curState != RIGHT)||(curRaid != 20))
					{
						//20 is radius
						if (SendMapMsg(mapData,MapTurnRight,0,0,LARGERAID,portMAX_DELAY) != pdTRUE) {
							VT_HANDLE_FATAL_ERROR(0);
						}
						curState = RIGHT;
						curRaid = 10;
					}
					#endif
				}
			}
			else
			{
				i2cCmdTurn[3] = 0;
				//255 is radius
				#if(USEMAPPING == 1)
				if((curState != STRAIGHT))
				{
					if (SendMapMsg(mapData,MapStraight,0,0,127,portMAX_DELAY) != pdTRUE) {
						VT_HANDLE_FATAL_ERROR(0);
					}
					curState = STRAIGHT;
					curRaid = 255;
				}
				#endif
			}
			//printf("S:%d:%d:%d \n",val1,val2,i2cCmdTurn[3]);
			/*if (lcdData != NULL) {
				if (SendLCDPrintMsg(lcdData,strnlen(lcdBuffer,vtLCDMaxLen),lcdBuffer,4,portMAX_DELAY) != pdTRUE) {
					VT_HANDLE_FATAL_ERROR(0);
				}
			} */
			#if TESTING == 0
			//For now just send back a command to go straight
			if (vtI2CEnQ(devPtr,vtI2CMsgTypeMotorSend,0x4F,sizeof(i2cCmdTurn),i2cCmdTurn,0) != pdTRUE) {
				VT_HANDLE_FATAL_ERROR(0);
			}
			#else
			//For now just send back a command to go straight
			if (vtTestEnQ(testData,vtI2CMsgTypeMotorSend,0x4F,sizeof(i2cCmdTurn),i2cCmdTurn,0) != pdTRUE) {
				VT_HANDLE_FATAL_ERROR(0);
			}
			#endif
			break;
		}
		case FrontValMsg: {
			int msgCount = getCount(&msgBuffer);
			int val1 = getVal1(&msgBuffer);
			int val2 = getVal2(&msgBuffer);
			if(countStartFront == 0)
			{
				countStartFront = 1;
				countFront = msgCount;
			}
			else{
				if((countFront + 1) == msgCount)
				{
					countFront = msgCount;	
				}
				else{
					/*sprintf(lcdBuffer,"D IR1: %d %d",countDistance,msgCount);
					if (lcdData != NULL) {
						if (SendLCDPrintMsg(lcdData,strnlen(lcdBuffer,vtLCDMaxLen),lcdBuffer,6,portMAX_DELAY) != pdTRUE) {
							VT_HANDLE_FATAL_ERROR(0);
						}
					}*/
					countFront = msgCount;
				}
			}
			sprintf(lcdBuffer,"F: %d %d",val1,val2);
			if (lcdData != NULL) {
				if (SendLCDPrintMsg(lcdData,strnlen(lcdBuffer,vtLCDMaxLen),lcdBuffer,6,portMAX_DELAY) != pdTRUE) {
					VT_HANDLE_FATAL_ERROR(0);
				}
			}

			break;
		}
		case UpdateSpeed: {
			int speed = getVal2(&msgBuffer);
			i2cCmdStraight[2] = speed;
			i2cCmdTurn[2] = speed;
			break;
		}
		case vtI2CMsgTypeAccRead: {

			int msgCount = getCount(&msgBuffer);
			int val1 = getVal1(&msgBuffer);
			int val2 = getVal2(&msgBuffer);
			
			/*sprintf(lcdBuffer,"Receiving Acc");
			if (lcdData != NULL) {
				if (SendLCDPrintMsg(lcdData,strnlen(lcdBuffer,vtLCDMaxLen),lcdBuffer,1,portMAX_DELAY) != pdTRUE) {
					VT_HANDLE_FATAL_ERROR(0);
				}
			}*/

			//checks count
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
				/*
					sprintf(lcdBuffer,"Dropped Acc");
					if (lcdData != NULL) {
						if (SendLCDPrintMsg(lcdData,strnlen(lcdBuffer,vtLCDMaxLen),lcdBuffer,6,portMAX_DELAY) != pdTRUE) {
							VT_HANDLE_FATAL_ERROR(0);
						}
					}*/
					countAcc = msgCount;
				}
			}

			//updates count in message to be sent
			i2cCmdHault[1] = countMotorCommand;
			countMotorCommand++;

			#if TESTING == 0
			//For now just send back a command to go straight
			if (vtI2CEnQ(devPtr,vtI2CMsgTypeMotorSend,0x4F,sizeof(i2cCmdHault),i2cCmdHault,0) != pdTRUE) {
				VT_HANDLE_FATAL_ERROR(0);
			}
			#else
			//For now just send back a command to go straight
			if (vtTestEnQ(testData,vtI2CMsgTypeMotorSend,0x4F,sizeof(i2cCmdHault),i2cCmdHault,0) != pdTRUE) {
				VT_HANDLE_FATAL_ERROR(0);
			}
			#endif
			
			//send the message to map that we are haulting
			if (SendMapMsg(mapData,MapHault,0,0,0,portMAX_DELAY) != pdTRUE) {
					VT_HANDLE_FATAL_ERROR(0);
			}
			//send the message to print the map
			if (SendMapMsg(mapData,PrintMap,0,0,0,portMAX_DELAY) != pdTRUE) {
					VT_HANDLE_FATAL_ERROR(0);
				}
			//hault all navigation
			while(RUN == 1);
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

