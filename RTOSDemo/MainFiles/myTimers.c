/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "projdefs.h"
#include "timers.h"

/* include files. */
#include "vtUtilities.h"
#include "LCDtask.h"
#include "myTimers.h"
#include "navigation.h"
#include "testing.h"


/* **************************************************************** */
// WARNING: Do not print in this file -- the stack is not large enough for this task
/* **************************************************************** */


/* *********************************************************** */
// Functions for the Temperature Task related timer
//
// how often the timer that sends messages to the temp task should run
// Set the task up to run every 100 ms

#define nav_RATE_BASE	( ( portTickType ) 30 / portTICK_RATE_MS)

// Callback function that is called by the NavTimer
//   Sends a message to the queue that is read by the Navigation Task
void NavTimerCallback(xTimerHandle pxTimer)
{
	if (pxTimer == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	} else {
		// When setting up this timer, I put the pointer to the 
		//   Nav structure as the "timer ID" so that I could access
		//   that structure here -- which I need to do to get the 
		//   address of the message queue to send to 
		vtNavStruct *ptr = (vtNavStruct *) pvTimerGetTimerID(pxTimer);
		// Make this non-blocking *but* be aware that if the queue is full, this routine
		// will not care, so if you care, you need to check something
		if (SendNavTimerMsg(ptr,nav_RATE_BASE,portMAX_DELAY) == errQUEUE_FULL) {
			// Here is where you would do something if you wanted to handle the queue being full
			VT_HANDLE_FATAL_ERROR(0);
		}
	}
}


void startTimerForNav(vtNavStruct *vtNavdata) {
	if (sizeof(long) != sizeof(vtNavStruct *)) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	xTimerHandle NavTimerHandle = xTimerCreate((const signed char *)"Nav Timer",nav_RATE_BASE,pdTRUE,(void *) vtNavdata,NavTimerCallback);
	if (NavTimerHandle == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	} else {
		if (xTimerStart(NavTimerHandle,0) != pdPASS) {
			VT_HANDLE_FATAL_ERROR(0);
		}
	}
}

#if TESTING == 1

#define test_RATE_BASE	( ( portTickType ) 50 / portTICK_RATE_MS)

// Callback function that is called by the NavTimer
//   Sends a message to the queue that is read by the Navigation Task
void TestTimerCallback(xTimerHandle pxTimer)
{
	if (pxTimer == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	} else {
		// When setting up this timer, I put the pointer to the 
		//   Test structure as the "timer ID" so that I could access
		//   that structure here -- which I need to do to get the 
		//   address of the message queue to send to 
		vtTestStruct *ptr = (vtTestStruct *) pvTimerGetTimerID(pxTimer);
		// Make this non-blocking *but* be aware that if the queue is full, this routine
		// will not care, so if you care, you need to check something
		if (SendTestTimerMsg(ptr,test_RATE_BASE,portMAX_DELAY) == errQUEUE_FULL) {
			// Here is where you would do something if you wanted to handle the queue being full
			VT_HANDLE_FATAL_ERROR(0);
		}
	}
}


void startTimerForTest(vtTestStruct *vtTestdata){
	if (sizeof(long) != sizeof(vtTestStruct *)) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	xTimerHandle TestTimerHandle = xTimerCreate((const signed char *)"Test Timer",test_RATE_BASE,pdTRUE,(void *) vtTestdata,TestTimerCallback);
	if (TestTimerHandle == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	} else {
		if (xTimerStart(TestTimerHandle,0) != pdPASS) {
			VT_HANDLE_FATAL_ERROR(0);
		}
	}
}
#endif