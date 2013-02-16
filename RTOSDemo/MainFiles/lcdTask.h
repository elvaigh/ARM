#ifndef LCD_TASK_H
#define LCD_TASK_H
#include "queue.h"
#include "timers.h"

// NOTE: This is a reasonable API definition file because there is nothing in it that the
//   user of the API does not need (e.g., no private definitions) and it defines the *only*
//   way a user of the API is allowed to interact with the task


// Define a data structure that is used to pass and hold parameters for this task
// Functions that use the API should not directly access this structure, but rather simply
//   pass the structure as an argument to the API calls
typedef struct __vtLCDStruct {
	xQueueHandle inQ;					   	// Queue used to send messages from other tasks to the LCD task to print
} vtLCDStruct;

// Structure used to define the messages that are sent to the LCD thread
//   the maximum length of a message to be printed is the size of the "buf" field below
#define vtLCDMaxLen 20

/* ********************************************************************* */
// The following are the public API calls that other tasks should use to work with the LCD task
//   Note: This is *not* the API for actually manipulating the graphics -- that API is defined in GLCD.h
//         and is accessed by the LCD task (other tasks should not access it or conflicts may occur).
//
// Start the task
// Args:
//   lcdData -- a pointer to a variable of type vtLCDStruct
//   uxPriority -- the priority you want this task to be run at
void StartLCDTask(vtLCDStruct *lcdData,unsigned portBASE_TYPE uxPriority);
//
// Send a timer message to the LCD task
// Args:
//   lcdData -- a pointer to a variable of type vtLCDStruct
//   ticksElapsed -- number of ticks since the last message (this will be sent in the message)
//   ticksToBlock -- how long the routine should wait if the queue is full
// Return:
//   Result of the call to xQueueSend()
portBASE_TYPE SendLCDTimerMsg(vtLCDStruct *lcdData,portTickType ticksElapsed,portTickType ticksToBlock);//
//
// Send a pixel to the LCD task
// Args:
//   lcdData -- a pointer to a variable of type vtLCDStruct
//   x -- The x  value associated with a pixel to draw (0-319)
//   y -- The y value associated with the pixel to draw (0-239)
//   ticksToBlock -- how long the routine should wait if the queue is full
// Return:
//   Result of the call to xQueueSend()
portBASE_TYPE SendLCDPixel(vtLCDStruct *lcdData,int x, int y, portTickType ticksToBlock);
//
// Send a pixel buffer of 32 pixels to the LCD task
// Args:
//   lcdData -- a pointer to a variable of type vtLCDStruct
//   x -- The x  value associated with the first pixel to draw (0-319)
//   * y -- a pointer to the array of Y values to draw (0-239)
//   ticksToBlock -- how long the routine should wait if the queue is full
// Return:
//   Result of the call to xQueueSend()
portBASE_TYPE SendLCDPixelBuff(vtLCDStruct *lcdData,int x, int* y, portTickType ticksToBlock);
//
//   Tells the LCD Screen to print graph outline
portBASE_TYPE SendLCDGraph(vtLCDStruct *lcdData, portTickType ticksToBlock);
//
// Send a string message to the LCD task for it to print
// Args:
//   lcdData -- a pointer to a variable of type vtLCDStruct
//   length -- number of characters in the string -- the call will result in a fatal error if you exceed the maximum length
//   pString -- string to print to the LCD
//   line -- line that the string should be printed on (0-9)
//   ticksToBlock -- how long the routine should wait if the queue is full
// Return:
//   Result of the call to xQueueSend()
portBASE_TYPE SendLCDPrintMsg(vtLCDStruct *lcdData,int length,char *pString,int line, portTickType ticksToBlock);
//
// Send a string message to the LCD task for it to print vertically
// Args:
//   lcdData -- a pointer to a variable of type vtLCDStruct
//   length -- number of characters in the string -- the call will result in a fatal error if you exceed the maximum length
//   pString -- string to print to the LCD
//   line -- line that the string should be printed on (0-9)
//   ticksToBlock -- how long the routine should wait if the queue is full
// Return:
//   Result of the call to xQueueSend()
portBASE_TYPE SendLCDPrintMsgVert(vtLCDStruct *lcdData,int length,char *pString,int line, portTickType ticksToBlock);
//
// Send a line to the LCD task for it to print vertically
// Args:
//   lcdData -- a pointer to a variable of type vtLCDStruct
//   Xs -- The x value of the top left corner of the Line (0-319)
//   Ys -- The y value of the top left corner of the Line (0-239)
//   Xf -- The x value of the bottom right corner of Line (0-319)
//   Yf -- The y value of the bottom right corner of Line (0-239)
//   ticksToBlock -- how long the routine should wait if the queue is full
// Return:
//   Result of the call to xQueueSend()
portBASE_TYPE SendLCDLine(vtLCDStruct *lcdData,int Xs, int Ys, int Xf, int Yf, portTickType ticksToBlock);
//
// Sends a pixel block
//portBASE_TYPE SendLCDPixelBlock(vtLCDStruct *lcdData,int x, int y1,int y2,int y3, int y4, int y5, int y6, int y7, int y8, int y9, int y10, portTickType ticksToBlock);
//
// Clears a pixel
// Args:
//   lcdData -- a pointer to a variable of type vtLCDStruct
//   x -- The x  value associated with a pixel to clear (0-319)
//   y -- The y value associated with the pixel to clear (0-239)
//   ticksToBlock -- how long the routine should wait if the queue is full
// Return:
//   Result of the call to xQueueSend()
portBASE_TYPE ClearLCDPixel(vtLCDStruct *lcdData,int x, int y, portTickType ticksToBlock);
//
// Clears a block of the LCD
// Args:
//   lcdData -- a pointer to a variable of type vtLCDStruct
//   Xs -- The x value of the top left corner of the block to be cleared (0-319)
//   Ys -- The y value of the top left corner of the block to be cleared (0-239)
//   Xf -- The x value of the bottom right corner of the block to be cleared (0-319)
//   Yf -- The y value of the bottom right corner of the block to be cleared (0-239)
//   ticksToBlock -- how long the routine should wait if the queue is full
// Return:
//   Result of the call to xQueueSend()
portBASE_TYPE ClearLCDBlock(vtLCDStruct *lcdData,int Xs, int Ys, int Xf, int Yf, portTickType ticksToBlock);
//
// Clears a char line
// Args:
//   lcdData -- a pointer to a variable of type vtLCDStruct
//   line -- The line number associated with a char line to clear (0-9)
//   ticksToBlock -- how long the routine should wait if the queue is full
// Return:
//   Result of the call to xQueueSend()
portBASE_TYPE ClearLCDTextLine(vtLCDStruct *lcdData,int line, portTickType ticksToBlock);
//
// Clears the LCD
// Args:
//   lcdData -- a pointer to a variable of type vtLCDStruct
//   ticksToBlock -- how long the routine should wait if the queue is full
// Return:
//   Result of the call to xQueueSend()
portBASE_TYPE ClearLCD(vtLCDStruct *lcdData, portTickType ticksToBlock);
/* ********************************************************************* */


void LCDTimerCallback(xTimerHandle);

#endif