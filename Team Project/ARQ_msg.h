#include "mbed.h"

#define ARQMSG_TYPE_DATA    0
#define ARQMSG_TYPE_ACK     1

//(TX/RX COMMON)
#define ARQMSG_OFFSET_TYPE		0
#define ARQMSG_OFFSET_SEQ   	1

//TX header
#define ARQMSG_TYPE_FINISH    	0
#define ARQMSG_TYPE_GOING     	1
#define ARQMSG_OFFSET_LSF   	2

//have to change 
#define ARQMSG_OFFSET_SO1    	3
#define ARQMSG_OFFSET_SO2    	4

#define ARQMSG_OFFSET_DATA		5

#define ARQMSG_ACKSIZE      	3

#define ARQMSG_MAXDATASIZE  	23			//26
#define ARQMSSG_MAX_SEQNUM  	256			//1024


int arqMsg_checkIfData(uint8_t* msg);
int arqMsg_checkIfAck(uint8_t* msg);
uint8_t arqMsg_encodeAck(uint8_t* msg_ack, uint8_t seq);
uint8_t arqMsg_encodeData(uint8_t* msg_data, uint8_t* data, int seq, int len , uint8_t LSF , uint16_t SO);
uint8_t arqMsg_getSeq(uint8_t* msg);
uint8_t* arqMsg_getWord(uint8_t* msg);
