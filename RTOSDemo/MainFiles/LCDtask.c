#include <stdlib.h>
#include <stdio.h>
#include <math.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"

/* include files. */
#include "GLCD.h"
#include "vtUtilities.h"
#include "LCDtask.h"
#include "string.h"

// I have set this to a larger stack size because of (a) using printf() and (b) the depth of function calls
//   for some of the LCD operations
// I actually monitor the stack size in the code to check to make sure I'm not too close to overflowing the stack
//   This monitoring takes place if INPSECT_STACK is defined (search this file for INSPECT_STACK to see the code for this) 
#define INSPECT_STACK 1
#define baseStack 3
#if PRINTF_VERSION == 1
#define lcdSTACK_SIZE		((baseStack+5)*configMINIMAL_STACK_SIZE)
#else
#define lcdSTACK_SIZE		(baseStack*configMINIMAL_STACK_SIZE)
#endif

// definitions and data structures that are private to this file
// Length of the queue to this task
#define vtLCDQLen 20
// a timer message -- not to be printed
#define LCDMsgTypeTimer 1
// a message to be printed
#define LCDMsgTypePrint 2
// a message to be printed vertically
#define LCDMsgTypePrintVert 3
// a message to print a pixel
#define LCDMsgTypePixel 4
//a message to draw a line
#define LCDMsgTypeLine 5
//a message to clear a pixel
#define LCDMsgTypeClearPixel 6
//a message to clear a block
#define LCDMsgTypeClearBlock 7
//a message to clear a text line
#define LCDMsgTypeClearLine 8
//a message to clear the LCD
#define LCDMsgTypeClear 9
// a message from the ADC
#define LCDMsgTypePixelBuff 10
// a message saying set up graph
#define LCDMsgTypeGraph 11
// actual data structure that is sent in a message
typedef struct __vtLCDMsg {
	uint8_t msgType;
	uint8_t	length;	 // Length of the message to be printed
	uint8_t buf[vtLCDMaxLen+1]; // On the way in, message to be sent, on the way out, message received (if any)
	uint16_t x; // x value if needed
	uint16_t y; // y value if needed  also used to pass line values for strings
	uint16_t xf; //x finish value if needed
	uint16_t yf; //y finish value if needed
	int *ya; // array of y pixels for block

} vtLCDMsg;
// end of defs

/* definition for the LCD task. */
static portTASK_FUNCTION_PROTO( vLCDUpdateTask, pvParameters );

/*-----------------------------------------------------------*/

void StartLCDTask(vtLCDStruct *ptr, unsigned portBASE_TYPE uxPriority)
{
	if (ptr == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}

	// Create the queue that will be used to talk to this task
	if ((ptr->inQ = xQueueCreate(vtLCDQLen,sizeof(vtLCDMsg))) == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	/* Start the task */
	portBASE_TYPE retval;
	if ((retval = xTaskCreate( vLCDUpdateTask, ( signed char * ) "LCD", lcdSTACK_SIZE, (void*)ptr, uxPriority, ( xTaskHandle * ) NULL )) != pdPASS) {
		VT_HANDLE_FATAL_ERROR(retval);
	}
}

portBASE_TYPE SendLCDTimerMsg(vtLCDStruct *lcdData,portTickType ticksElapsed,portTickType ticksToBlock)
{
	if (lcdData == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	vtLCDMsg lcdBuffer;
	lcdBuffer.length = sizeof(ticksElapsed);
	if (lcdBuffer.length > vtLCDMaxLen) {
		// no room for this message
		VT_HANDLE_FATAL_ERROR(lcdBuffer.length);
	}
	memcpy(lcdBuffer.buf,(char *)&ticksElapsed,sizeof(ticksElapsed));
	lcdBuffer.msgType = LCDMsgTypeTimer;
	return(xQueueSend(lcdData->inQ,(void *) (&lcdBuffer),ticksToBlock));
}
portBASE_TYPE SendLCDPixel(vtLCDStruct *lcdData,int x, int y, portTickType ticksToBlock)
{
    if (lcdData == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	vtLCDMsg lcdBuffer;

	lcdBuffer.x = x;
	lcdBuffer.y = y;
	lcdBuffer.msgType = LCDMsgTypePixel;
	return(xQueueSend(lcdData->inQ,(void *) (&lcdBuffer),ticksToBlock));
}
portBASE_TYPE SendLCDPixelBuff(vtLCDStruct *lcdData,int x, int y[], portTickType ticksToBlock)
{
    if (lcdData == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	vtLCDMsg lcdBuffer;

	lcdBuffer.x = x;
	lcdBuffer.ya = y;
	lcdBuffer.msgType = LCDMsgTypePixelBuff;
	return(xQueueSend(lcdData->inQ,(void *) (&lcdBuffer),ticksToBlock));
} 
portBASE_TYPE SendLCDGraph(vtLCDStruct *lcdData, portTickType ticksToBlock)
{
	if (lcdData == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	vtLCDMsg lcdBuffer;

	lcdBuffer.msgType = LCDMsgTypeGraph;
	return(xQueueSend(lcdData->inQ,(void *) (&lcdBuffer),ticksToBlock));
}
portBASE_TYPE SendLCDPrintMsg(vtLCDStruct *lcdData,int length,char *pString, int l, portTickType ticksToBlock)
{
	if (lcdData == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	vtLCDMsg lcdBuffer;

	if (length > vtLCDMaxLen) {
		// no room for this message
		VT_HANDLE_FATAL_ERROR(lcdBuffer.length);
	}
	lcdBuffer.y = l;
	lcdBuffer.length = strnlen(pString,vtLCDMaxLen);
	lcdBuffer.msgType = LCDMsgTypePrint;
	strncpy((char *)lcdBuffer.buf,pString,vtLCDMaxLen);
	return(xQueueSend(lcdData->inQ,(void *) (&lcdBuffer),ticksToBlock));
}

portBASE_TYPE SendLCDPrintMsgVert(vtLCDStruct *lcdData,int length,char *pString, int l, portTickType ticksToBlock)
{
	if (lcdData == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	vtLCDMsg lcdBuffer;

	if (length > vtLCDMaxLen) {
		// no room for this message
		VT_HANDLE_FATAL_ERROR(lcdBuffer.length);
	}
	lcdBuffer.y = l;
	lcdBuffer.length = strnlen(pString,vtLCDMaxLen);
	lcdBuffer.msgType = LCDMsgTypePrintVert;
	strncpy((char *)lcdBuffer.buf,pString,vtLCDMaxLen);
	return(xQueueSend(lcdData->inQ,(void *) (&lcdBuffer),ticksToBlock));
}

portBASE_TYPE SendLCDLine(vtLCDStruct *lcdData,int Xs, int Ys, int Xf, int Yf, portTickType ticksToBlock)
{
	if (lcdData == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	vtLCDMsg lcdBuffer;
	lcdBuffer.x = Xs;
	lcdBuffer.y = Ys;
	lcdBuffer.xf = Xf;
	lcdBuffer.yf = Yf;
	lcdBuffer.msgType = LCDMsgTypeLine;
	return(xQueueSend(lcdData->inQ,(void *) (&lcdBuffer),ticksToBlock));
}

portBASE_TYPE ClearLCDPixel(vtLCDStruct *lcdData,int x, int y, portTickType ticksToBlock)
{
	if (lcdData == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	vtLCDMsg lcdBuffer;
	lcdBuffer.x = x;
	lcdBuffer.y = y;
	lcdBuffer.msgType = LCDMsgTypeClearPixel;
	return(xQueueSend(lcdData->inQ,(void *) (&lcdBuffer),ticksToBlock));
}

portBASE_TYPE ClearLCDBlock(vtLCDStruct *lcdData,int Xs, int Ys, int Xf, int Yf, portTickType ticksToBlock)
{
	if (lcdData == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	vtLCDMsg lcdBuffer;
	
	lcdBuffer.x = Xs;
	lcdBuffer.y = Ys;
	lcdBuffer.xf = Xf;
	lcdBuffer.yf = Yf;
	lcdBuffer.msgType = LCDMsgTypeClearBlock;
	return(xQueueSend(lcdData->inQ,(void *) (&lcdBuffer),ticksToBlock));
}

portBASE_TYPE ClearLCDTextLine(vtLCDStruct *lcdData,int l, portTickType ticksToBlock)
{
    if (lcdData == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	vtLCDMsg lcdBuffer;
	
	lcdBuffer.y = l;
	lcdBuffer.msgType = LCDMsgTypeClearLine;
	return(xQueueSend(lcdData->inQ,(void *) (&lcdBuffer),ticksToBlock)); 
}

portBASE_TYPE ClearLCD(vtLCDStruct *lcdData, portTickType ticksToBlock)
{
	if (lcdData == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	vtLCDMsg lcdBuffer;
	
	lcdBuffer.msgType = LCDMsgTypeClear;
	return(xQueueSend(lcdData->inQ,(void *) (&lcdBuffer),ticksToBlock)); 
}
// Private routines used to unpack the message buffers
//   I do not want to access the message buffer data structures outside of these routines
portTickType unpackTimerMsg(vtLCDMsg *lcdBuffer)
{
	portTickType *ptr = (portTickType *) lcdBuffer->buf;
	return(*ptr);
}

int getMsgType(vtLCDMsg *lcdBuffer)
{
	return(lcdBuffer->msgType);
} 
int getMsgX(vtLCDMsg *lcdBuffer)
{
	return(lcdBuffer->x);
}
int getMsgY(vtLCDMsg *lcdBuffer)
{
	return(lcdBuffer->y);
}
int getMsgXf(vtLCDMsg *lcdBuffer)
{
	return(lcdBuffer->xf);
}
int getMsgYf(vtLCDMsg *lcdBuffer)
{
	return(lcdBuffer->yf);
}
int getMsgYa(vtLCDMsg *lcdBuffer,int v)
{
	return(lcdBuffer->ya[v]);
}
int getMsgLength(vtLCDMsg *lcdBuffer)
{
	return(lcdBuffer->msgType);
}

void copyMsgString(char *target,vtLCDMsg *lcdBuffer,int targetMaxLen)
{
	strncpy(target,(char *)(lcdBuffer->buf),targetMaxLen);
}

// End of private routines for message buffers

// If LCD_EXAMPLE_OP=0, then accept messages that may be timer or print requests and respond accordingly
// If LCD_EXAMPLE_OP=1, then do a rotating ARM bitmap display
#define LCD_EXAMPLE_OP 0
#if LCD_EXAMPLE_OP==1
// This include the file with the definition of the ARM bitmap
#include "ARM_Ani_16bpp.c"
#endif


// This is the actual task that is run
static portTASK_FUNCTION( vLCDUpdateTask, pvParameters )
{
	#if LCD_EXAMPLE_OP==0
	unsigned short screenColor = 0;
	unsigned short tscr;
	unsigned char curLine;
	unsigned int x;
	#elif LCD_EXAMPLE_OP==1
	unsigned char picIndex = 0;
	#else
	Bad definition
	#endif
	vtLCDMsg msgBuffer;
	vtLCDStruct *lcdPtr = (vtLCDStruct *) pvParameters;

	#ifdef INSPECT_STACK
	// This is meant as an example that you can re-use in your own tasks
	// Inspect to the stack remaining to see how much room is remaining
	// 1. I'll check it here before anything really gets started
	// 2. I'll check during the run to see if it drops below 10%
	// 3. You could use break points or logging to check on this, but
	//    you really don't want to print it out because printf() can
	//    result in significant stack usage.
	// 4. Note that this checking is not perfect -- in fact, it will not
	//    be able to tell how much the stack grows on a printf() call and
	//    that growth can be *large* if version 1 of printf() is used.   
	unsigned portBASE_TYPE InitialStackLeft = uxTaskGetStackHighWaterMark(NULL);
	unsigned portBASE_TYPE CurrentStackLeft;
	float remainingStack = InitialStackLeft;
	remainingStack /= lcdSTACK_SIZE;
	if (remainingStack < 0.10) {
		// If the stack is really low, stop everything because we don't want it to run out
		// The 0.10 is just leaving a cushion, in theory, you could use exactly all of it
		VT_HANDLE_FATAL_ERROR(0);
	}
	#endif

	/* Initialize the LCD and set the initial colors */
	GLCD_Init();
	tscr = Maroon; // may be reset in the LCDMsgTypeTimer code below
	screenColor = Orange; // may be reset in the LCDMsgTypeTimer code below
	GLCD_SetTextColor(tscr);
	GLCD_SetBackColor(screenColor);
	GLCD_Clear(screenColor);

	curLine = 0;
	// This task should never exit
	for(;;)
	{	
		#ifdef INSPECT_STACK   
		CurrentStackLeft = uxTaskGetStackHighWaterMark(NULL);
		float remainingStack = CurrentStackLeft;
		remainingStack /= lcdSTACK_SIZE;
		if (remainingStack < 0.10) {
			// If the stack is really low, stop everything because we don't want it to run out
			VT_HANDLE_FATAL_ERROR(0);
		}
		#endif

		#if LCD_EXAMPLE_OP==0
		// Wait for a message
		if (xQueueReceive(lcdPtr->inQ,(void *) &msgBuffer,portMAX_DELAY) != pdTRUE) {
			VT_HANDLE_FATAL_ERROR(0);
		}
		
		//Log that we are processing a message -- more explanation of logging is given later on
		vtITMu8(vtITMPortLCDMsg,getMsgType(&msgBuffer));
		vtITMu8(vtITMPortLCDMsg,getMsgLength(&msgBuffer));

		// Take a different action depending on the type of the message that we received
		switch(getMsgType(&msgBuffer)) {
		case LCDMsgTypePrint: {
			// This will result in the text printing in the last five lines of the screen
			char   lineBuffer[lcdCHAR_IN_LINE+1];
			copyMsgString(lineBuffer,&msgBuffer,lcdCHAR_IN_LINE);
			curLine = getMsgY(&msgBuffer);
			// clear the line
			GLCD_ClearLn(curLine,1);
			// show the text
			GLCD_DisplayString(curLine,0,1,(unsigned char *)lineBuffer);
			break;
		}
		case LCDMsgTypePrintVert: {	//fixme
			// This will result in the text printing in the last five lines of the screen
			char   lineBuffer[lcdCHAR_IN_LINE+1];
			copyMsgString(lineBuffer,&msgBuffer,lcdCHAR_IN_LINE);
			curLine = getMsgY(&msgBuffer);
			// clear the line  Need to clear fixme
			// GLCD_ClearLn(curLine,1);
			// show the text
			int iter = 0;
			for(;;)
			{
				GLCD_DisplayChar(iter,curLine,1,(char)((unsigned char *)lineBuffer)[iter]);
				iter++;
				if(iter == strnlen(lineBuffer,vtLCDMaxLen))
				{
					break;
				}
			}
			break;
		}
		case LCDMsgTypeLine: {
			int xs = getMsgX(&msgBuffer);
			int ys = getMsgY(&msgBuffer);
			int xf = getMsgXf(&msgBuffer);
			int yf = getMsgYf(&msgBuffer);
			int i = 0;
			x = xs;
			if(xf == xs)
			{
				for(;;)
				{
					//GLCD_ClearWindow(xs,0,xs,200,screenColor);
					GLCD_PutPixel(xs,(ys+i));
					i++;
					if(i>(yf-ys))
					{
						break;
					}
				}
			}
			else if(yf == ys)
			{
				for(;;)
				{
					//GLCD_ClearWindow((xs+i),0,(xs+i),200,screenColor);
					GLCD_PutPixel((xs+i),ys);
					i++;
					if(i>(xf-xs))
					{
						break;
					}
				}
			}
			else
			{
				float slope = (float)(yf-ys)/((float)(xf-xs));
				for(;;)
				{
					//GLCD_ClearWindow((xs+i),0,(xs+i),200,screenColor);
					GLCD_PutPixel((xs+i),(ys+lrint(slope*i)));
					i++;
					if(i>=(xf-xs))
					{
						break;
					}
				}
			}
			break;
		}
		case LCDMsgTypePixel: {
			int x = getMsgX(&msgBuffer);
			int y = getMsgY(&msgBuffer);

			//GLCD_ClearWindow(x,0,1,200,screenColor);
			GLCD_PutPixel(x,y);
			
			break;
		}
		case LCDMsgTypePixelBuff: {
			int x = getMsgX(&msgBuffer);
			int y = 0;
			int i = 0;
			for(;;)
			{
				y  = getMsgYa(&msgBuffer,i);
				GLCD_ClearWindow(x,0,1,200,screenColor);
				GLCD_PutPixel(x,y);
				x++;
				if(x > 320)
				{
					x = 40;
				}
				i++;
				if(i>=100)
				{
					break;
				}

			}

			break;
		} 
		case LCDMsgTypeGraph: {
			//horizontal string
			GLCD_DisplayString(curLine,0,1,(unsigned char *)("     Time"));
			//vertical string
			char *vString = "  Voltage";
			int iter = 0;
			for(;;)
			{
				GLCD_DisplayChar(iter,curLine,1,(char)((unsigned char *)vString)[iter]);
				iter++;
				if(iter == 9)
				{
					break;
				}
			}
			//lines
			//vert
			int xLine = 30;
			int yLine = 0;
			int c = 0;
			for(;;)
			{
				GLCD_PutPixel(xLine,yLine + c);
				c++;
				if(c>(210))
				{
					break;
				}
			}
			//horizontal
			c = 0;
			yLine = 210;
			for(;;)
			{
				GLCD_PutPixel(xLine + c,yLine);
				c++;
				if(c>290)
				{
					break;
				}
			}
			break;
		}
		//Can i even clear a pixel?
		case LCDMsgTypeClearPixel: {
			break;
		}
		case LCDMsgTypeClearBlock: {
			int xs = getMsgX(&msgBuffer);
			int ys = getMsgY(&msgBuffer);
			int xf = getMsgXf(&msgBuffer);
			int yf = getMsgYf(&msgBuffer);
			GLCD_ClearWindow(xs,ys,(xf-xs),(yf-ys),screenColor);
			break;
		}
		case LCDMsgTypeClearLine: {
			int l = getMsgY(&msgBuffer);
			GLCD_ClearLn(l,1);
			break;
		}
		case LCDMsgTypeClear: {
			GLCD_Clear(screenColor);
			break;
		}
		case LCDMsgTypeTimer: {
			break;
		}
		default: {
			// In this configuration, we are only expecting to receive timer messages
			VT_HANDLE_FATAL_ERROR(getMsgType(&msgBuffer));
			break;
		}
		} // end of switch()

		// Here is a way to do debugging output via the built-in hardware -- it requires the ULINK cable and the
		//   debugger in the Keil tools to be connected.  You can view PORT0 output in the "Debug(printf) Viewer"
		//   under "View->Serial Windows".  You have to enable "Trace" and "Port0" in the Debug setup options.  This
		//   should not be used if you are using Port0 for printf()
		// There are 31 other ports and their output (and port 0's) can be seen in the "View->Trace->Records"
		//   windows.  You have to enable the prots in the Debug setup options.  Note that unlike ITM_SendChar()
		//   this "raw" port write is not blocking.  That means it can overrun the capability of the system to record
		//   the trace events if you go too quickly; that won't hurt anything or change the program execution and
		//   you can tell if it happens because the "View->Trace->Records" window will show there was an overrun.
		//vtITMu16(vtITMPortLCD,screenColor);

		#elif 	LCD_EXAMPLE_OP==1
		// In this alternate version, we just keep redrawing a series of bitmaps as
		//   we receive timer messages
		// Wait for a message
		if (xQueueReceive(lcdPtr->inQ,(void *) &msgBuffer,portMAX_DELAY) != pdTRUE) {
			VT_HANDLE_FATAL_ERROR(0);
		}
		if (getMsgType(&msgBuffer) != LCDMsgTypeTimer) {
			// In this configuration, we are only expecting to receive timer messages
			VT_HANDLE_FATAL_ERROR(getMsgType(&msgBuffer));
		}
  		/* go through a  bitmap that is really a series of bitmaps */
		picIndex = (picIndex + 1) % 9;
		GLCD_Bmp(99,99,120,45,(unsigned char *) &ARM_Ani_16bpp[picIndex*(120*45*2)]);
		#else
		Bad setting
		#endif	
	}
}
