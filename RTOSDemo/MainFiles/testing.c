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
#include "testing.h"
#include "mapping.h"

/* *********************************************** */
// definitions and data structures that are private to this file
// Length of the queue to this task
#define vtTestQLen 20 
// actual data structure that is sent in a message
typedef struct __vtTestMsg {
	uint8_t msgType;
	uint8_t count;	 
	uint8_t value1;	 
	uint8_t value2;
} vtTestMsg;

typedef struct __vtTestI2CMsg {
	uint8_t msgType; // A field you will likely use in your communications between processors (and for debugging)
	uint8_t slvAddr; // Address of the device to whom the message is being sent (or was sent)
	uint8_t	rxLen;	 // Length of the message you *expect* to receive (or, on the way back, the length that *was* received)
	uint8_t txLen;   // Length of the message you want to sent (or, on the way back, the length that *was* sent)
	uint8_t status;  // status of the completed operation -- I've not done anything much here, you probably should...
	uint8_t buf[vtI2CMLen]; // On the way in, message to be sent, on the way out, message received (if any)
} vtTestI2CMsg;

// I have set this to a large stack size because of (a) using printf() and (b) the depth of function calls
//   for some of the i2c operations	-- almost certainly too large, see LCDTask.c for details on how to check the size
#define baseStack 3
#if PRINTF_VERSION == 1
#define testSTACK_SIZE		((baseStack+10)*configMINIMAL_STACK_SIZE)
#else
#define testSTACK_SIZE		((baseStack+10)*configMINIMAL_STACK_SIZE)
#endif

#define PRINTGRAPH 0

//can change to place the car in different starting point
#define CARSTARTX 160
#define CARSTARTY 240
#define CARSTARTANGLE 0

//chose the type of course
#define TESTSTRAIGHT 0
#define TESTRIGHTTURN 0
#define TESTLEFTTURN 0
#define TESTSPACERIGHT 0
#define TESTINTERSECTION 1
#define TESTLEFTBRANCH 0

#if((TESTSTRAIGHT == 1)||(TESTINTERSECTION == 1))
#define MAXLEFT 100
#define MAXRIGHT 100
#define FINWIDTH 17
#endif

#if(TESTRIGHTTURN == 1)
#define MAXLEFT 100
#define MAXRIGHT 40
#define FINWIDTH 31
#endif

#if(TESTLEFTTURN == 1)
#define MAXLEFT 40
#define MAXRIGHT 100
#define FINWIDTH 31
#endif

#if(TESTLEFTBRANCH == 1)
#define MAXLEFT 100
#define MAXRIGHT 64
#define FINWIDTH 17
#endif

// end of defs
/* *********************************************** */

//Simulation Course
uint8_t simRightWall[MAXRIGHT][2];
uint8_t simLeftWall[MAXLEFT][2];
uint8_t simFinish[FINWIDTH][2];

/* The nav task. */
static portTASK_FUNCTION_PROTO( vTestUpdateTask, pvParameters );

/*-----------------------------------------------------------*/
// Public API
void vStartTestTask(vtTestStruct *params,unsigned portBASE_TYPE uxPriority, vtI2CStruct *i2c,vtLCDStruct *lcd)
{
	// Create the queue that will be used to talk to this task
	if ((params->inQ = xQueueCreate(vtTestQLen,sizeof(vtTestI2CMsg))) == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	/* Start the task */
	portBASE_TYPE retval;
	params->dev = i2c;
	params->lcdData = lcd;
	if ((retval = xTaskCreate( vTestUpdateTask, ( signed char * ) "Testing", testSTACK_SIZE, (void *) params, uxPriority, ( xTaskHandle * ) NULL )) != pdPASS) {
		VT_HANDLE_FATAL_ERROR(retval);
	}
}

portBASE_TYPE SendTestTimerMsg(vtTestStruct *testData,portTickType ticksElapsed,portTickType ticksToBlock)
{
	if (testData == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	vtTestI2CMsg testBuffer;
	testBuffer.msgType = TestMsgTypeTimer;
	return(xQueueSend(testData->inQ,(void *) (&testBuffer),ticksToBlock));
}

/*
portBASE_TYPE SendTestMsg(vtTestStruct *testData,uint8_t msgType,uint8_t count,uint8_t val1,uint8_t val2,portTickType ticksToBlock)
{
	vtTestMsg testBuffer;

	if (testData == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	testBuffer.msgType = msgType;
	testBuffer.count = count;
	testBuffer.value1 = val1;
	testBuffer.value2 = val2;
	return(xQueueSend(testData->inQ,(void *) (&testBuffer),ticksToBlock));
} */
// A simple routine to use for filling out and sending a message to the I2C thread
//   You may want to make your own versions of these as they are not suited to all purposes
portBASE_TYPE vtTestEnQ(vtTestStruct *dev,uint8_t msgType,uint8_t slvAddr,uint8_t txLen,const uint8_t *txBuf,uint8_t rxLen)
{
	vtTestI2CMsg msgBuf;
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
int getTestMsgType(vtTestI2CMsg *Buffer)
{
	return(Buffer->msgType);
}
uint8_t getTestCount(vtTestI2CMsg *Buffer)
{
	uint8_t val = (uint8_t) Buffer->buf[1];
	return(val);
}
uint8_t getTestVal1(vtTestI2CMsg *Buffer)
{
	uint8_t rad = (uint8_t) Buffer->buf[2];
	return(rad);
}
uint8_t getTestVal2(vtTestI2CMsg *Buffer)
{
	uint8_t rd = (uint8_t) Buffer->buf[3];
	return(rd);
}

int isWall(int px,int py)
{
	int j;

	for(j=0;j<MAXRIGHT;j++)
	{
		//checks to see if the new pixel = a wall pixel
		if(((px == simRightWall[j][0]) &&
		   (py == simRightWall[j][1])))
		{
		   	return 1;
		}
	}
	for(j=0;j<MAXLEFT;j++)
	{
		if(((px == simLeftWall[j][0]) &&
		   (py == simLeftWall[j][1])))
		{
			return 1;
		}
	}
	return 0;
}
int isFinish(int px,int py)
{
	int j;

	for(j=0;j<FINWIDTH;j++)
	{
		//checks to see if the new pixel = a wall pixel
		if(((px == simFinish[j][0]) &&
		   (py == simFinish[j][1])))
		{
		   	return 1;
		}
	}
	return 0;
}
int getDL(int cPX,int cPY, int angle)
{
	int px = cPX;
	int py = cPY;
	int count = 0;
	while(isWall(px,py) == 0)
	{
		count++;
		//sensor can only see 80 cm away
		if(count == 80)
			break;
		if(angle == 0)
			px--;
		else if(angle == 45)
		{
			px--;
			py++;
		}
		else if(angle == 90)
		{
			py++;
		}
		else if(angle == 135)
		{
			px++;
			py++;
		}
		else if(angle == 180)
		{
			px++;
		}
		else if(angle == 225)
		{
			px++;
			py--;
		}
		else if(angle == 270)
		{
			py--;
		}
		else if(angle == 315)
		{
			px--;
			py--;
		}	
	}
	int DL = sqrt(abs((float)px - (float)cPX)*abs((float)px - (float)cPX) + abs((float)py - (float)cPY)*abs((float)py - (float)cPY)) + 0.5;
	return DL;
}

int getDR(int cPX, int cPY, int angle)
{
	int px = cPX;
	int py = cPY;
	int count = 0;
	while(isWall(px,py) == 0)
	{
		count++;
		//sensors can only see 80 cm;
		if(count == 80)
			break;
		if(angle == 0)
			px++;
		else if(angle == 45)
		{
			px++;
			py--;
		}
		else if(angle == 90)
		{
			py--;
		}
		else if(angle == 135)
		{
			px--;
			py--;
		}
		else if(angle == 180)
		{
			px--;
		}
		else if(angle == 225)
		{
			px--;
			py++;
		}
		else if(angle == 270)
		{
			py++;
		}
		else if(angle == 315)
		{
			px++;
			py++;
		}	
	}
	int DR = sqrt(abs((float)px - (float)cPX)*abs((float)px - (float)cPX) + abs((float)py - (float)cPY)*abs((float)py - (float)cPY)) + 0.5;
	return DR;
}
// I2C commands for the Motor Encoder
	uint8_t i2cCmdReadVals[]= {0xAA};
	uint8_t i2cCmdDistance[] = {0x0A,0x00,0x00,0x00};
	uint8_t i2cCmdFinish[] = {0x36,0x00,0x00,0x00};
	uint8_t i2cCmdMotor[] = {0x35,0x00,0x01,0x01};
// end of I2C command definitions


// This is the actual task that is run
static portTASK_FUNCTION( vTestUpdateTask, pvParameters )
{
	// Define local constants here
	uint8_t countStartAcc = 0;
	uint8_t countStartMotor = 0;
	uint8_t countDist = 0;
	uint8_t countAcc = 0;
	uint8_t countMotor = 0;

	// Get the parameters
	vtTestStruct *param = (vtTestStruct *) pvParameters;
	// Get the I2C device pointer
	vtI2CStruct *devPtr = param->dev;
	// Get the LCD information pointer
	vtLCDStruct *lcdData = param->lcdData;

	// String buffer for printing
	char lcdBuffer[vtLCDMaxLen+1];

	// Buffer for receiving messages
	vtTestI2CMsg msgBuffer;
	
	//Simulation Car position
	int simCar[4];

	// Assumes that the I2C device (and thread) have already been initialized

	//create straight course
	#if (TESTSTRAIGHT == 1)
	uint8_t i = 0;
	for(i=0	; i<MAXLEFT; i++)
	{			
		//left wall x
		simLeftWall[i][0] = 152;
		//left wall y
	    simLeftWall[i][1] = 240-i;
		//right wall x
		simRightWall[i][2] = 168;
		//right wall y
		simRightWall[i][3] = 240-i;
	}
	//create finish line
	for(i=0;i<FINWIDTH;i++)
	{
		simFinish[i][0] = 152+i;
		simFinish[i][1] = 139;
	}
	#endif

	#if(TESTRIGHTTURN == 1)
	uint8_t i = 0;
	//left wall
	for(i=0	; i<25; i++)
	{
		//left wall x
		simLeftWall[i][0] = 145;
		//left wall y
	    simLeftWall[i][1] = 240-i;
	}
	for(i=25;i<75;i++)
	{
		simLeftWall[i][0] = 145 + 1 + (int)(i-25)/2;
		simLeftWall[i][1] = 240 - 25 - (int)(i-24)/2;		
	}
	for(i=75;i<100;i++)
	{
		simLeftWall[i][0] = 145 + 25 + (i-74);
		simLeftWall[i][1] = 240 - 50;		
	}

	//right wall
	for(i=0;i<10;i++)
	{
		//right wall x
		simRightWall[i][2] = 175;
		//right wall y
		simRightWall[i][3] = 240-i;
	}
	for(i=10;i<30;i++)
	{
		simRightWall[i][0] = 175 + 1 + (int)(i-10)/2;
		simRightWall[i][1] = 240 - 10 - (int)(i-9)/2;		
	}
	for(i=30;i<40;i++)
	{
		simRightWall[i][0] = 175 + 10 + (i-29);
		simRightWall[i][1] = 240 - 20;		
	}
	//create finish line
	for(i=0;i<FINWIDTH;i++)
	{
		simFinish[i][0] = 195;
		simFinish[i][1] = 220-i;
	}
	#endif

	#if(TESTLEFTTURN == 1)
	uint8_t i = 0;
	//right wall
	for(i=0	; i<25; i++)
	{
		//right wall x
		simRightWall[i][0] = 175;
		//right wall y
	    simRightWall[i][1] = 240-i;
	}
	for(i=25;i<75;i++)
	{
		simRightWall[i][0] = 175 - 1 - (int)(i-25)/2;
		simRightWall[i][1] = 240 - 25 - (int)(i-24)/2;		
	}
	for(i=75;i<100;i++)
	{
		simRightWall[i][0] = 175 - 25 - (i-74);
		simRightWall[i][1] = 240 - 50;		
	}

	//left wall
	for(i=0;i<10;i++)
	{
		//left wall x
		simLeftWall[i][2] = 145;
		//left wall y
		simLeftWall[i][3] = 240-i;
	}
	for(i=10;i<30;i++)
	{
		simLeftWall[i][0] = 145 - 1 - (int)(i-10)/2;
		simLeftWall[i][1] = 240 - 10 - (int)(i-9)/2;		
	}
	for(i=30;i<40;i++)
	{
		simLeftWall[i][0] = 145 - 10 - (i-29);
		simLeftWall[i][1] = 240 - 20;		
	}
	//create finish line
	for(i=0;i<FINWIDTH;i++)
	{
		simFinish[i][0] = 125;
		simFinish[i][1] = 220-i;
	}
	#endif

	//create straight course with a jut on the right side
	#if (TESTSPACERIGHT == 1)
	uint8_t i = 0;
	for(i=0	; i<MAXLEFT; i++)
	{			
		//left wall x
		simLeftWall[i][0] = 152;
		//left wall y
	    simLeftWall[i][1] = 240-i;
		if((i>24) && (i<75))
		{
			//right wall x
			simRightWall[i][2] = 178;
		}
		else
		{
			//right wall x
			simRightWall[i][2] = 168;
		}
		//right wall y
		simRightWall[i][3] = 240-i;
	}
	//create finish line
	for(i=0;i<FINWIDTH;i++)
	{
		simFinish[i][0] = 152+i;
		simFinish[i][1] = 139;
	}
	#endif

	//intersection
	//create straight course with an intersection
	#if (TESTINTERSECTION == 1)
	uint8_t i = 0;
	for(i=0	; i<25; i++)
	{			
		//left wall x
		simLeftWall[i][0] = 152;
		//left wall y
	    simLeftWall[i][1] = 240-i;
		//right wall x
		simRightWall[i][2] = 168;
		//right wall y
		simRightWall[i][3] = 240-i;
	}
	for(i=25; i<50; i++)
	{			
		//left wall x
		simLeftWall[i][0] = 152 - (i-25);
		//left wall y
	    simLeftWall[i][1] = 240-24;
		//right wall x
		simRightWall[i][2] = 168 + (i-25);
		//right wall y
		simRightWall[i][3] = 240-24;
	}
	for(i=50; i<75; i++)
	{			
		//left wall x
		simLeftWall[i][0] = 152 - 25 + (i-50);
		//left wall y
	    simLeftWall[i][1] = 240-40;
		//right wall x
		simRightWall[i][2] = 168 + 25 - (i-50);
		//right wall y
		simRightWall[i][3] = 240-40;
	}
	for(i=75; i<100; i++)
	{			
		//left wall x
		simLeftWall[i][0] = 152;
		//left wall y
	    simLeftWall[i][1] = 240-40-(i-75);
		//right wall x
		simRightWall[i][2] = 168;
		//right wall y
		simRightWall[i][3] = 240-40-(i-75);
	}
	//create finish line
	for(i=0;i<FINWIDTH;i++)
	{
		simFinish[i][0] = 152+i;
		simFinish[i][1] = 176;
	}
	#endif

	//branch left intersection
	//create straight course with a branch left
	#if (TESTLEFTBRANCH == 1)
	uint8_t i = 0;
	for(i=0	; i<64; i++)
	{			
		//right wall x
		simRightWall[i][2] = 168;
		//right wall y
		simRightWall[i][3] = 240-i;
	}
	for(i=0	; i<25; i++)
	{			
		//left wall x
		simLeftWall[i][0] = 152;
		//left wall y
	    simLeftWall[i][1] = 240-i;
	}
	for(i=25; i<50; i++)
	{	
		//left wall x
	    simLeftWall[i][0] = 152 - 1 - (int)(i-25)/2;
		//left wall y
		simLeftWall[i][1] = 240 - 24 - (int)(i-24)/2;
	}
	for(i=50; i<75; i++)
	{	
		//left wall x
	    simLeftWall[i][0] = 152 - 1 - (int)(i-50)/2;
		//left wall y
		simLeftWall[i][1] = 240 - 40 - (int)(i-49)/2;		
	}
	for(i=75; i<100; i++)
	{			
		//left wall x
		simLeftWall[i][0] = 152;
		//left wall y
	    simLeftWall[i][1] = 240-40-(i-75);
	}
	//create finish line
	for(i=0;i<FINWIDTH;i++)
	{
		simFinish[i][0] = 152+i;
		simFinish[i][1] = 176;
	}
	#endif


	for(i=0;i<MAXLEFT;i++)
	{
		//print left wall
		if (lcdData != NULL) {
			if (SendLCDPixel(lcdData,simLeftWall[i][0],simLeftWall[i][1],portMAX_DELAY) != pdTRUE) {
				VT_HANDLE_FATAL_ERROR(0);
			}
		}
	}
	for(i=0;i<MAXRIGHT;i++)
	{
		if (lcdData != NULL) {
			if (SendLCDPixel(lcdData,simRightWall[i][0],simRightWall[i][1],portMAX_DELAY) != pdTRUE) {
				VT_HANDLE_FATAL_ERROR(0);
			}
		}
	}

	//start car in center
	//X position
	simCar[0] = CARSTARTX;
	//Y position
	simCar[1] = CARSTARTY;
	//position in course
	simCar[2] = 0;
	//angle relative to start position (45 to left and 315 to right)
	simCar[3] = CARSTARTANGLE;

	//print first car pixel
	if (lcdData != NULL) {
		if (SendLCDPixel(lcdData,simCar[0],simCar[1],portMAX_DELAY) != pdTRUE) {
			VT_HANDLE_FATAL_ERROR(0);
		}
	}

	//Used to keep track of last command the car received
	//0 for straight
	//10 for small left (15-100) cm
	//11 for hard left (<15) cm
	//20 for small right (15-100) cm
	//21 for hard right (<15) cm
	uint8_t lastCommand = 0;
	
	//initialize the car to the bottom center
	int carSPX = CARSTARTX;
	int carSPY = CARSTARTY;
	 
 	//printf("pre: %d:%d:%d \n",simCar[0],simCar[1],lastCommand);
	// Like all good tasks, this should never exit
	for(;;)
	{
		// Wait for a message from either a timer or from an I2C operation
		if (xQueueReceive(param->inQ,(void *) &msgBuffer,portMAX_DELAY) != pdTRUE) {
			VT_HANDLE_FATAL_ERROR(0);
		}

		// Now, based on the type of the message and the state, we decide on the new state and action to take
		switch(getTestMsgType(&msgBuffer)) {
		case TestMsgTypeTimer: {

			//straight
			if(lastCommand == 0)
			{
				//car going straight
				if(simCar[3] == 0)
				{
					simCar[1] = simCar[1]-1;
				}
				//45 degrees left
				else if(simCar[3] == 45)
				{
					simCar[0] = simCar[0]-1;
					simCar[1] = simCar[1]-1;
				}
				//straight left
				else if(simCar[3] == 90)
				{
					simCar[0] = simCar[0]-1;
				}
				//down left
				else if(simCar[3] == 135)
				{
					simCar[0] = simCar[0]-1;
					simCar[1] = simCar[1]+1;
				}
				//straight down
				else if(simCar[3] == 180)
				{
					simCar[1] = simCar[1]+1;
				}
				//down right
				else if(simCar[3] == 225)
				{
					simCar[0] = simCar[0]+1;
					simCar[1] = simCar[1]+1;
				}
				//straight right
				else if(simCar[3] == 270)
				{
					simCar[0] = simCar[0]+1;
				}
				//up right
				else if(simCar[3] == 315)
				{
					simCar[0] = simCar[0]+1;
					simCar[1] = simCar[1]-1;
				}
				else
				{
					VT_HANDLE_FATAL_ERROR(0);
				}

				simCar[2]++;
			}
			//left (all commands are relative to current direction)
			else if(lastCommand == 10)
			{
			  	//car going straight
				if(simCar[3] == 0)
				{
					simCar[1] = simCar[1]-1;
					simCar[0] = simCar[0]-1;
				}
				//45 degrees left
				else if(simCar[3] == 45)
				{
					simCar[0] = simCar[0]-1;
				}
				//straight left
				else if(simCar[3] == 90)
				{
					simCar[0] = simCar[0]-1;
					simCar[1] = simCar[1]+1;
				}
				//down left
				else if(simCar[3] == 135)
				{
					simCar[1] = simCar[1]+1;
				}
				//straight down
				else if(simCar[3] == 180)
				{
					simCar[0] = simCar[0]+1;
					simCar[1] = simCar[1]+1;
				}
				//down right
				else if(simCar[3] == 225)
				{
					simCar[0] = simCar[0]+1;
				}
				//straight right
				else if(simCar[3] == 270)
				{
					simCar[0] = simCar[0]+1;
					simCar[1] = simCar[1]-1;
				}
				//up right
				else if(simCar[3] == 315)
				{
					simCar[1] = simCar[1]-1;
				}
				else
				{
					VT_HANDLE_FATAL_ERROR(0);
				}
				simCar[2]++;
				simCar[3] = simCar[3] + 45;
				if(simCar[3] == 360)
				{
					simCar[3] = 0;
				}
				
			}
			//slight right
			else if(lastCommand == 20)
			{
			  	//car going straight
				if(simCar[3] == 0)
				{
					simCar[1] = simCar[1]-1;
					simCar[0] = simCar[0]+1;
				}
				//45 degrees left
				else if(simCar[3] == 45)
				{
					simCar[1] = simCar[1]-1;
				}
				//straight left
				else if(simCar[3] == 90)
				{
					simCar[0] = simCar[0]-1;
					simCar[1] = simCar[1]-1;
				}
				//down left
				else if(simCar[3] == 135)
				{
					simCar[0] = simCar[0]-1;
				}
				//straight down
				else if(simCar[3] == 180)
				{
					simCar[0] = simCar[0]-1;
					simCar[1] = simCar[1]+1;
				}
				//down right
				else if(simCar[3] == 225)
				{
					simCar[1] = simCar[1]-1;
				}
				//straight right
				else if(simCar[3] == 270)
				{
					simCar[0] = simCar[0]+1;
					simCar[1] = simCar[1]+1;
				}
				//up right
				else if(simCar[3] == 315)
				{
					simCar[0] = simCar[0]+1;
				}
				else
				{
					VT_HANDLE_FATAL_ERROR(0);
				}
				simCar[2]++;
				if(simCar[3] == 0)
				{
					simCar[3] = 315;
				}
				else
				{
					simCar[3] = simCar[3] - 45;
				}
			}
			else
			{
			  	//printf(lastCommand+ "\n");
				VT_HANDLE_FATAL_ERROR(0);
			}

			//Send Motor Message
			if (vtI2CConQ(devPtr,vtI2CMsgTypeMotorRead,0x4F,sizeof(i2cCmdMotor),i2cCmdMotor,sizeof(i2cCmdMotor)) != pdTRUE) {
				VT_HANDLE_FATAL_ERROR(0);
			}

			//print Car
			if (lcdData != NULL) {
				if (SendLCDPixel(lcdData,simCar[0],simCar[1],portMAX_DELAY) != pdTRUE) {
					VT_HANDLE_FATAL_ERROR(0);
				}
			}
			
			//check for hitting wall
			if(isWall(simCar[0],simCar[1]) == 1)
			{
			  	sprintf(lcdBuffer,"Car Crashed");
				if (lcdData != NULL) {
					if (SendLCDPrintMsg(lcdData,strnlen(lcdBuffer,vtLCDMaxLen),lcdBuffer,0,portMAX_DELAY) != pdTRUE) {
						VT_HANDLE_FATAL_ERROR(0);
					}
				}
				//hault
				while(1);
			}

			//check for end of course
			if(isFinish(simCar[0],simCar[1]) == 1)
			{
				sprintf(lcdBuffer,"Car Finished");
				if (lcdData != NULL) {
					if (SendLCDPrintMsg(lcdData,strnlen(lcdBuffer,vtLCDMaxLen),lcdBuffer,0,portMAX_DELAY) != pdTRUE) {
						VT_HANDLE_FATAL_ERROR(0);
					}
				}
				//send finish line command
				if (vtI2CConQ(devPtr,vtI2CMsgTypeAccRead,0x4F,sizeof(i2cCmdFinish),i2cCmdFinish,sizeof(i2cCmdFinish)) != pdTRUE) {
					VT_HANDLE_FATAL_ERROR(0);
				}
			}

			i2cCmdDistance[1] = countDist;
			countDist++;
			//Calculating carSPX (where the car will be if car goes straight 10 units)
			//car going straight
			if(simCar[3] == 0)
			{
				carSPY = simCar[1]-10;
				carSPX = simCar[0];
			}
			//45 degrees left
			else if(simCar[3] == 45)
			{
				//7 used when going diagonal because 7^2 + 7^2 = 98 ~=10^2 
				carSPX = simCar[0]-7;
				carSPY = simCar[1]-7;
			}
			//straight left
			else if(simCar[3] == 90)
			{
				carSPX = simCar[0]-10;
				carSPY = simCar[1];
			}
			//down left
			else if(simCar[3] == 135)
			{
				carSPX = simCar[0]-7;
				carSPY = simCar[1]+7;
			}
			//straight down
			else if(simCar[3] == 180)
			{
				carSPX = simCar[0];
				carSPY = simCar[1]+10;
			}
			//down right
			else if(simCar[3] == 225)
			{
				carSPX = simCar[0]-7;
				carSPY = simCar[1]+7;
			}
			//straight right
			else if(simCar[3] == 270)
			{
				carSPX = simCar[0]+10;
				carSPY = simCar[1];
			}
			//up right
			else if(simCar[3] == 315)
			{
				carSPX = simCar[0]+7;
				carSPY = simCar[1]-7;
			}
			
			i2cCmdDistance[2] = getDL(carSPX,carSPY,simCar[3]);
			i2cCmdDistance[3] = getDR(carSPX,carSPY,simCar[3]);
			

			if (vtI2CConQ(devPtr,DistanceMsg,0x4F,sizeof(i2cCmdDistance),i2cCmdDistance,sizeof(i2cCmdDistance)) != pdTRUE) {
				VT_HANDLE_FATAL_ERROR(0);
			}
			break;
		}
		case vtI2CMsgTypeMotorSend: {
			
			uint8_t msgCount = getTestCount(&msgBuffer);
			//speed
			uint8_t val1 = getTestVal1(&msgBuffer);
			//turn radius see below for translation
			uint8_t val2 = getTestVal2(&msgBuffer);

			//hault at end
			if(val1 == 0)
			{
				while(1);
			}
			//check count
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
				/*
					sprintf(lcdBuffer,"D MS: %d %d",countMotor,msgCount);
					if (lcdData != NULL) {
						if (SendLCDPrintMsg(lcdData,strnlen(lcdBuffer,vtLCDMaxLen),lcdBuffer,1,portMAX_DELAY) != pdTRUE) {
							VT_HANDLE_FATAL_ERROR(0);
						}
					} */
					countMotor = msgCount;
				}
			}
			//set response
			//0 for straight
			//10 for small left (15-100) cm
			//11 for hard left (<15) cm
			//20 for small right (15-100) cm
			//21 for hard right (<15) cm
			// 0 = straight
			// 127 = spin left
			// 128 = spin right
			// anything between 0 and 127 = turn left with a turn radius of (127 - val2)
			// anything between 128 and 255 = turn right with a turn radius of (val2 - 128)
			
			if(val2 == 0)
				lastCommand = 0;
			else if(val2 < 128)
				lastCommand = 10;
			else
				lastCommand = 20;
			
			//printf("LastC: %d\n",lastCommand);	
			break;
		}

		default: {
			//printf("invalid data type");
			/*sprintf(lcdBuffer,"%d",getTestMsgType(&msgBuffer) );
				if (lcdData != NULL) {
					if (SendLCDPrintMsg(lcdData,strnlen(lcdBuffer,vtLCDMaxLen),lcdBuffer,1,portMAX_DELAY) != pdTRUE) {
						VT_HANDLE_FATAL_ERROR(0);
					}
				}*/
			//VT_HANDLE_FATAL_ERROR(getMsgType(&msgBuffer));
			break;
		}
		}


	}
}

