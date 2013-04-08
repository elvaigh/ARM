#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "freertos.h"
#include "vtI2C.h"
#include "navigation.h"

#define MSGT_MOTOR_DATA 53
#define MSGT_MOTOR_COMMAND 52

#define HALT				0
#define TEN_CM_S			10
#define TWENTY_CM_S			20
#define THIRTY_CM_S			30

#define STRAIGHT			127
#define PIVOT_LEFT			0
#define LEFT_FIVE_CM		5
#define LEFT_TEN_CM			10
#define LEFT_FIFTEEN_CM		15
#define LEFT_TWENTY_CM		20
#define LEFT_THIRTY_CM		30

#define PIVOT_RIGHT			128
#define RIGHT_FIVE_CM		133
#define RIGHT_TEN_CM		138
#define RIGHT_FIFTEEN_CM	143
#define RIGHT_TWENTY_CM		148
#define RIGHT_THIRTY_CM		158

#define TEST_STRAIGHT			1
#define TEST_RIGHT_TURN			0
#define TEST_LEFT_TURN			0
#define TEST_RIGHT_PIVOT		0
#define TEST_LEFT_PIVOT			0
#define TEST_CHANGE_SPEED		0
#define TEST_STRAIGHT_TO_TURN	0

void sendMotorCommand( vtI2CStruct * devPtr, uint8_t speed, uint8_t turning );
uint8_t getLeftDistanceTraveledInMillimeters( vtNavMsg * msg );
uint8_t getRightDistanceTraveledInMillimeters( vtNavMsg * msg );

#endif // MOTOR_CONTROL_H