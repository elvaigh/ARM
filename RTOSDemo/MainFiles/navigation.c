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

#define SAFEZONE 8	       // 8 cm
#define FRONTLARGE 10	   //10 cm
#define FRONTSMALL 5	   // 5 cm
#define ACCELMIN 15		   //Need to change based on real world accelerator values
#define ACCEPTABLEMISS 20  //1 second

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

	//used to represent the last direction turned
	//this is used if the front IR sensor detects a wall too close
	//which will cause the robot to turn the opposite direction of this
	// 0 = left : 1 = right
	//default is to turn right if the robot sees a wall in front
	uint8_t lastTurn = 0;

	float distanceF = 0.0;

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

			
			int msgCount = getCount(&msgBuffer);
			int val1 = getVal1(&msgBuffer);
			int val2 = getVal2(&msgBuffer);

			missedMotor = 0;
			//Send a message to the map telling it what we got
			/*if (SendMapMsg(mapData,MapMessageMotor,msgCount,val1,val2,portMAX_DELAY) != pdTRUE) {
						VT_HANDLE_FATAL_ERROR(0);
					} */
			if(countStartMotor == 0)
			{
				countStartMotor = 1;
				countMotor = msgCount;
			}
			else{
				if((countMotor + 1) == msgCount)
				{
					countMotor = msgCount;	
				}
				else{
					/*sprintf(lcdBuffer,"D IR1: %d %d",countDistance,msgCount);
					if (lcdData != NULL) {
						if (SendLCDPrintMsg(lcdData,strnlen(lcdBuffer,vtLCDMaxLen),lcdBuffer,6,portMAX_DELAY) != pdTRUE) {
							VT_HANDLE_FATAL_ERROR(0);
						}
					}*/
					countMotor = msgCount;
				}
			}
			sprintf(lcdBuffer,"Receiving M");
			if (lcdData != NULL) {
				if (SendLCDPrintMsg(lcdData,strnlen(lcdBuffer,vtLCDMaxLen),lcdBuffer,0,portMAX_DELAY) != pdTRUE) {
					VT_HANDLE_FATAL_ERROR(0);
				}
			}
			#if TESTING == 0
			//For now just send back a command to go straight
			//John you can fake a turn here by changing i2cCmdStraight to i2cCmdTurn and setting i2cCmdTurn [2] to your value
			if (vtI2CEnQ(devPtr,vtI2CMsgTypeMotorSend,0x4F,sizeof(i2cCmdStraight),i2cCmdStraight,0) != pdTRUE) {
				VT_HANDLE_FATAL_ERROR(0);
			}
			#else
			//For now just send back a command to go straight
			//Currently sensors work in testing but not in real so resopond to motor in real

			/*if (vtTestEnQ(testData,vtI2CMsgTypeMotorSend,0x4F,sizeof(i2cCmdStraight),i2cCmdStraight,0) != pdTRUE) {
				VT_HANDLE_FATAL_ERROR(0);
			}*/
			#endif
			/*sprintf(lcdBuffer,"Receiving Motor");
			if (lcdData != NULL) {
				if (SendLCDPrintMsg(lcdData,strnlen(lcdBuffer,vtLCDMaxLen),lcdBuffer,0,portMAX_DELAY) != pdTRUE) {
					VT_HANDLE_FATAL_ERROR(0);
				}
			}*/

			break;
		}
		case DistanceMsg: {
			int msgCount = getCount(&msgBuffer);
			int val1 = getVal1(&msgBuffer);
			int val2 = getVal2(&msgBuffer);

			missedDistance = 0;
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
			}/*
			sprintf(lcdBuffer,"D: %d %d",val1,val2);
			if (lcdData != NULL) {
				if (SendLCDPrintMsg(lcdData,strnlen(lcdBuffer,vtLCDMaxLen),lcdBuffer,5,portMAX_DELAY) != pdTRUE) {
					VT_HANDLE_FATAL_ERROR(0);
				}
			}*/
			i2cCmdTurn[1] = countMotorCommand;
			countMotorCommand++;

			if((val1 < SAFEZONE) || (val2 < SAFEZONE))
			{
				distanceF = (float)(val1-val2)/(float)(val1+val2);
				//set response
				// 0 = straight
				// 127 = spin left
				// 128 = spin right
				// anything between 0 and 127 = turn left with a turn radius of (val2)
				// anything between 128 and 255 = turn right with a turn radius of (val2 - 128)
	
				//John actual turning here
				//hard Left (0 + 10cm radius) = 5
				if(distanceF > 0.4)
				{
					i2cCmdTurn[3] = 5;
					lastTurn = 0;
				}
				//small Left (0 + 20cm radius) = 10
				else if(distanceF > 0)
				{
					i2cCmdTurn[3] = 10;
					lastTurn = 0;
				}
				//0 = straight
				else if(distanceF == 0)
				{
					i2cCmdTurn[3] = 0;
				}
				//hard Right (128+5cm radius) = 133
				else if(distanceF < -0.4)
				{
					i2cCmdTurn[3] = 133;
					lastTurn = 1;
				}
				//small right (128+10cm radius) = 138
				else
				{
					i2cCmdTurn[3] = 138;
					lastTurn = 1;
				}
			}
			else
			{
				i2cCmdTurn[3] = 0;
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
			/*
			sprintf(lcdBuffer,"F: %d %d",val1,val2);
			if (lcdData != NULL) {
				if (SendLCDPrintMsg(lcdData,strnlen(lcdBuffer,vtLCDMaxLen),lcdBuffer,6,portMAX_DELAY) != pdTRUE) {
					VT_HANDLE_FATAL_ERROR(0);
				}
			}*/
			if(val2 < FRONTSMALL)
			{
				i2cCmdTurn[1] = countMotorCommand;
				countMotorCommand++;

				//hard Left (127-10cm radius) = 117
				if(lastTurn == 1)
				{
					i2cCmdTurn[3] = 117;
					lastTurn = 0;
				}
				//hard Right (128+10cm radius) = 138
				else
				{
					i2cCmdTurn[3] = 138;
					lastTurn = 1;
				}
			}
			else if(val2 < FRONTLARGE)
			{
				i2cCmdTurn[1] = countMotorCommand;
				countMotorCommand++;

				//small Left (127-20cm radius) = 107
				if(lastTurn == 1)
				{
					i2cCmdTurn[3] = 107;
					lastTurn = 0;
				}

				//small right (128+20cm radius) = 148
				else
				{
					i2cCmdTurn[3] = 148;
					lastTurn = 1;
				}
			}
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
		case vtI2CMsgTypeAccRead: {

			int msgCount = getCount(&msgBuffer);
			int val1 = getVal1(&msgBuffer);
			int val2 = getVal2(&msgBuffer);
			int value = val1 * 256 + val2;
			
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
			if(value > ACCELMIN)
			{
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
			}
			//hault all navigation
			while(1);
			break;
		}
		case NavMsgTypeTimer: {
			#if TESTING == 0
			if (vtI2CEnQ(devPtr,NavMsgTypeTimer,0x4F,sizeof(i2cCmdReadVals),i2cCmdReadVals,4) != pdTRUE) {
				VT_HANDLE_FATAL_ERROR(0);
			}
			#else
			//For now just send back a command to go straight
			if (vtTestEnQ(testData,NavMsgTypeTimer,0x4F,sizeof(i2cCmdReadVals),i2cCmdReadVals,4) != pdTRUE) {
				VT_HANDLE_FATAL_ERROR(0);
			}
			#endif
			missedMotor++;
			missedDistance++;
			//if so long since message received hault
			/*if((missedMotor > ACCEPTABLEMISS) || (missedDistance > ACCEPTABLEMISS))
			{
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
			
			}  */
			/*sprintf(lcdBuffer,"Timer Messages");
			if (lcdData != NULL) {
				if (SendLCDPrintMsg(lcdData,strnlen(lcdBuffer,vtLCDMaxLen),lcdBuffer,6,portMAX_DELAY) != pdTRUE) {
					VT_HANDLE_FATAL_ERROR(0);
				}
			} */
			 
			break;
		}
		default: {
			//printf("invalid data type");
			VT_HANDLE_FATAL_ERROR(getMsgType(&msgBuffer));
			break;
		}
		}


	}
}

