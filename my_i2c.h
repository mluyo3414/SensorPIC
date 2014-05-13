#ifndef __my_i2c_h
#define __my_i2c_h

#include "messages.h"

#define MAXI2CBUF MSGLEN
//I2C struct 
typedef struct __i2c_comm {
    unsigned char buffer[MAXI2CBUF];
    unsigned char buflen;
    unsigned char event_count;
    unsigned char status;
    unsigned char error_code;
    unsigned char error_count;
    unsigned char outbuffer[MAXI2CBUF];
    unsigned char outbuflen;
    unsigned char outbufind;
    unsigned char slave_addr;
} i2c_comm;

int bufferFlag;
int bufferFlag1;
unsigned char buff[I2C_MESSAGE_LENGTH];
unsigned char buff1[I2C_MESSAGE_LENGTH];
int sensorFlag =0;
int stopFlag =0 ;
int stopFlagEmergency=0;


int sendData =0;
int sideOut =0;
int frontOut =0;

int outOfBounds=0;
int outOfBoundsEmergency=0;
int emergencyStopOutofBounds=0;
int emergencyStopFlag = 0;


unsigned char sensors[SENSOR_NUMBER + 1];



#define I2C_IDLE 0x5
#define I2C_STARTED 0x6
#define	I2C_RCV_DATA 0x7
#define I2C_SLAVE_SEND 0x8

#define I2C_ERR_THRESHOLD 1
#define I2C_ERR_OVERRUN 0x4
#define I2C_ERR_NOADDR 0x5
#define I2C_ERR_NODATA 0x6
#define I2C_ERR_MSGTOOLONG 0x7
#define I2C_ERR_MSG_TRUNC 0x8



void init_i2c(i2c_comm *);
void i2c_int_handler(void);
void start_i2c_slave_reply(unsigned char,unsigned char *);
void i2c_configure_slave(unsigned char);
void i2c_configure_master(unsigned char);
unsigned char i2c_master_send(unsigned char,unsigned char *);
unsigned char i2c_master_recv(unsigned char);
void readMessages();
//ready to send
void sendReply( unsigned char* , int );
//NACK
void sendNotReady(void);

#endif