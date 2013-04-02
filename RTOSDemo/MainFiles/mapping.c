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
	uint8_t count;	 // raidus / wall byte depending on the message type
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

//change based on rover characteristics
#define MAXSTRAIGHT 50				//speed for straight aways
#define MAXSHARPTURN 20				//speed for sharp turns
#define MAXWIDETURN 30				//speed for wide turns
#define MINSTRAIGHT 10				//minimum straight distance to warrant a speed up
#define CHANGETOTURN 5  			//cm before change to a turn to update speed
#define MINWIDEDIST 11				//minimum radius to be counted as a wide turn

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

portBASE_TYPE SendMapMsg(vtMapStruct *mapData,uint8_t msgType,uint8_t count,uint8_t leftDistance,uint8_t rightDistance,portTickType ticksToBlock)
{
	vtMapMsg mapBuffer;

	if (mapData == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	mapBuffer.msgType = msgType;
	mapBuffer.count = count;
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
uint8_t getCount(vtMapMsg *Buffer)
{
	uint8_t c = (uint8_t) Buffer->count;
	return(c);
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
	uint8_t i2cCmdSpeed[] = {0x05,0x00,0x00,0x00};
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

	// Definitions of the states for the FSM below
	const uint8_t fsmStateStraight = 0;
	const uint8_t fsmStateTurnLeft = 1;
	const uint8_t fsmStateTurnRight = 2;	  
	const uint8_t fsmStateHault = 3;

	uint8_t currentState = fsmStateHault;
	uint8_t curRaid = 255;
	//ints for storing distance traveled in a current state
	int DL = 0;
	int DR = 0;

	//used to store the map
	int map[20][3];
	//map[i][0] = state
	//map[i][1] = distance
	//map[i][2] = radius
	
	//int to know the current state
	int stateCount = 0;

	uint8_t FIRST = 1;
	//bool to determine if an update speed has been sent or not before a turn
	uint8_t notSent = 1;

	  
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
			//int count = getCount(&msgBuffer);
			int rightD = getRightDistance(&msgBuffer);
			int leftD = getLeftDistance(&msgBuffer);

			DL = DL + leftD;
			DR = DR + rightD;

			printf("R: %d,L: %d\n",DR,DL);
		
			if(FIRST != 1)
			{
				if(currentState != fsmStateHault)
				{
					//if you need to slow before a turn
					if((map[stateCount+1][0] == fsmStateTurnLeft) || (map[stateCount+1][0] == fsmStateTurnRight))
					{
						//if within the distance to slow before a turn and you have not already sent a command to turn
						if(((map[stateCount][1] - (DL + DR + 0.5)/2) <= CHANGETOTURN) && notSent == 1)
						{
							//slow turn
							if(map[stateCount+1][2] > MINWIDEDIST)
							{
								i2cCmdSpeed[2] = MAXWIDETURN;
								if (vtI2CConQ(devPtr,UpdateSpeed,0x4F,sizeof(i2cCmdSpeed),i2cCmdSpeed,sizeof(i2cCmdSpeed)) != pdTRUE) {
									VT_HANDLE_FATAL_ERROR(0);
								}
							}
							//sharp turn
							else
							{
								i2cCmdSpeed[2] = MAXSHARPTURN;
								if (vtI2CConQ(devPtr,UpdateSpeed,0x4F,sizeof(i2cCmdSpeed),i2cCmdSpeed,sizeof(i2cCmdSpeed)) != pdTRUE) {
									VT_HANDLE_FATAL_ERROR(0);
								}
							}
							notSent = 0;
						}
					}	
				}
			}
			break;
		}
		case MapStraight: {
		printf("straight\n");
			int raid = getRightDistance(&msgBuffer);

			notSent = 1;
			//saves the state
			if(FIRST == 1)
			{
				map[stateCount][0] = currentState;
				
				//stores the inside tread distance
				if(currentState == fsmStateStraight)
				{
					//gets the average of the distance travled
					map[stateCount][1] = (DL + DR + 0.5)/2;	
				}
				else if(currentState == fsmStateTurnLeft)
				{
					//saves inside track distance
					map[stateCount][1] = DL;	
				}
				else if(currentState == fsmStateTurnRight)
				{
					//saves inside track distance
					map[stateCount][1] = DR;
				}
				else if(currentState == fsmStateHault)
				{
					map[stateCount][1] = 0;
				}
				//checks the state
	
	
				//radius of 255 = straight
				map[stateCount][2] = curRaid;
			}

			//sets the current state to straight
			currentState = fsmStateStraight;
			curRaid = raid;
			DR = 0;
			DL = 0;
			stateCount++;
			if((FIRST != 1) && (map[stateCount][1] > MINSTRAIGHT)){
				i2cCmdSpeed[2] = MAXSTRAIGHT;
				if (vtI2CConQ(devPtr,UpdateSpeed,0x4F,sizeof(i2cCmdSpeed),i2cCmdSpeed,sizeof(i2cCmdSpeed)) != pdTRUE) {
					VT_HANDLE_FATAL_ERROR(0);
				}
			}
			break;
		}
		case MapTurnLeft: {
			printf("Left\n");
			int raid = getRightDistance(&msgBuffer);

			notSent = 1;
			//saves the state
			if(FIRST == 1)
			{
				map[stateCount][0] = currentState;
				
				//stores the inside tread distance
				if(currentState == fsmStateStraight)
				{
					//gets the average of the distance travled
					map[stateCount][1] = (DL + DR + 0.5)/2;	
				}
				else if(currentState == fsmStateTurnLeft)
				{
					//saves inside track distance
					map[stateCount][1] = DL;	
				}
				else if(currentState == fsmStateTurnRight)
				{
					//saves inside track distance
					map[stateCount][1] = DR;
				}
				else if(currentState == fsmStateHault)
				{
					map[stateCount][1] = 0;
				}
	
				//sets radius
				map[stateCount][2] = curRaid;
			}

			//sets the current state to left
			currentState = fsmStateTurnLeft;
			curRaid = raid;
			DR = 0;
			DL = 0;
			stateCount++;
			break;
		}
		case MapTurnRight: {
			printf("right\n");

			notSent = 1;
			int raid = getRightDistance(&msgBuffer);
			//saves the state
			if(FIRST == 1)
			{
				map[stateCount][0] = currentState;
				
				//stores the inside tread distance
				if(currentState == fsmStateStraight)
				{
					//gets the average of the distance travled
					map[stateCount][1] = (DL + DR + 0.5)/2;	
				}
				else if(currentState == fsmStateTurnLeft)
				{
					//saves inside track distance
					map[stateCount][1] = DL;	
				}
				else if(currentState == fsmStateTurnRight)
				{
					//saves inside track distance
					map[stateCount][1] = DR;
				}
				else if(currentState == fsmStateHault)
				{
					map[stateCount][1] = 0;
				}
	
				//sets radius
				map[stateCount][2] = curRaid;
			}

			//sets the current state to straight
			currentState = fsmStateTurnRight;
			curRaid = raid;
			DR = 0;
			DL = 0;
			stateCount++;
			break;
		}
		case MapHault: {
			printf("Hault\n");

			notSent = 1;
			int raid = getRightDistance(&msgBuffer);

			//saves the state
			if(FIRST == 1)
			{
				map[stateCount][0] = currentState;
				
				//stores the inside tread distance
				if(currentState == fsmStateStraight)
				{
					//gets the average of the distance travled
					map[stateCount][1] = (DL + DR + 0.5)/2;	
				}
				else if(currentState == fsmStateTurnLeft)
				{
					//saves inside track distance
					map[stateCount][1] = DL;	
				}
				else if(currentState == fsmStateTurnRight)
				{
					//saves inside track distance
					map[stateCount][1] = DR;
				}
				else if(currentState == fsmStateHault)
				{
					map[stateCount][1] = 0;
				}
	
				//sets radius
				map[stateCount][2] = curRaid;
			}

			//sets the current state to hault
			currentState = fsmStateHault;
			curRaid = raid;
			DR = 0;
			DL = 0;
			stateCount++;
			break;
		}
		case PrintMap: {
			int i = 0;
			for(i=0;i<stateCount;i++)
			{
				sprintf(lcdBuffer,"%d,%d,%d",map[i][0],map[i][1],map[i][2]);
				if (lcdData != NULL) {
					if (SendLCDPrintMsg(lcdData,strnlen(lcdBuffer,vtLCDMaxLen),lcdBuffer,i+1,portMAX_DELAY) != pdTRUE) {
						VT_HANDLE_FATAL_ERROR(0);
					}
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

