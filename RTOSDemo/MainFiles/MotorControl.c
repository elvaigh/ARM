#include "MotorControl.h"
#include "vtI2C.h"
#include "navigation.h"

void sendMotorCommand( vtI2CStruct * devPtr, uint8_t speed, uint8_t turning ) {
	static uint8_t count = 0;
	uint8_t msgbuffer[4];
	
	msgbuffer[0] = MSGT_MOTOR_COMMAND;
	msgbuffer[1] = count;
	msgbuffer[2] = speed;
	msgbuffer[3] = turning;
	
	if( vtI2CEnQ( devPtr, MSGT_MOTOR_COMMAND, 0x4F, sizeof(msgbuffer), msgbuffer,0 ) != pdTRUE ) {
		VT_HANDLE_FATAL_ERROR(0);
	}
}

uint8_t getDistanceTraveledInMillimeters( uint8_t side, uint8_t * msgbuffer ) {
	return msgbuffer[side];
}

uint8_t getLeftDistanceTraveledInMillimeters( uint8_t * msgbuffer ) {
	return getDistanceTraveledInMillimeters( MOTOR_LEFT_SIDE, msgbuffer );
}

uint8_t getRightDistanceTraveledInMillimeters( uint8_t * msgbuffer ) {
	return getDistanceTraveledInMillimeters( MOTOR_RIGHT_SIDE, msgbuffer );
}