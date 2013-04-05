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
	
	if( vtI2CEnQ( devPtr, MSGT_MOTOR_COMMAND, 0x4F, sizeof(msgbuffer), msgbuffer, 0 ) != pdTRUE ) {
		VT_HANDLE_FATAL_ERROR(0);
	}

	count++;
}

uint8_t getDistanceTraveledInMillimeters( uint8_t value ) {
	float millimeters = (float) value / 8.0f;
	return (uint8_t) millimeters;
}

uint8_t getLeftDistanceTraveledInMillimeters( vtNavMsg * msg ) {
	return getDistanceTraveledInMillimeters( msg->value1 );
}

uint8_t getRightDistanceTraveledInMillimeters( vtNavMsg * msg ) {
	return getDistanceTraveledInMillimeters( msg->value2 );
}