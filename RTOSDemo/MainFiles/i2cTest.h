#ifndef I2CTEST_H
#define I2CTEST_H
#include "vtI2C.h"

// Public API
//
// The job of this task is to mimic a PIC and put data into the I2C queue
//
// Start the task
// Args:
//   conductorData: Data structure used by the task
//   uxPriority -- the priority you want this task to be run at
//   i2c: pointer to the data structure for an i2c task
//   navigation: pointer to the data structure for an navigation task
//	 mapping: pointer to the data structure for a mapping task
void vStartTestrTask(unsigned portBASE_TYPE uxPriority, vtI2CStruct *i2c);
#endif