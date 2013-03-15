#ifndef I2CTASK_MSG_TYPES_H
#define I2CTASK_MSG_TYPES_H

// Here is where I define the types of the messages that I am passing to the I2C task
//   --Note that none of these message types (as I have implemented this) actually go over the I2C bus, but they
//     are useful for matching up what is send to/from the I2C task message queues
//
// I have defined them all here so that they are unique

#define vtI2CMsgTypeMotorRead 53
#define vtI2CMsgTypeMotorSend 52  //Command send to you
#define vtI2CMsgTypeAccRead 54
#define vtI2CMsgTypeIRRead1 55	  //left IR sensor
#define vtI2CMsgTypeIRRead2 56	  //center IR sensor
#define vtI2CMsgTypeIRRead3 57	  //right IR sensor

// below is not actually an i2c message, but the value is reserved
#define NavMsgTypeTimer 3 
#define SensorMsgTypeTimer 4 
#define MapMessageMotor 5
#define MapMessageStateChange 6

//used for testing purposes
#define vtMsgTypeNavMsg	7
#define testNavMsg 8
#define TestMsgTypeTimer 9

//Strumsky to parse values from msg types 55-57 above into these
#define DistanceMsg 10
#define FrontValMsg 11


#endif