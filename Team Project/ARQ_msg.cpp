#include "mbed.h"
#include "ARQ_msg.h"


int arqMsg_checkIfData(uint8_t* msg)
{
    return (msg[ARQMSG_OFFSET_TYPE] == ARQMSG_TYPE_DATA);
}

int arqMsg_checkIfAck(uint8_t* msg)
{
    return (msg[ARQMSG_OFFSET_TYPE] == ARQMSG_TYPE_ACK);
}


uint8_t arqMsg_encodeAck(uint8_t* msg_ack, uint8_t seq)
{
    msg_ack[ARQMSG_OFFSET_TYPE] = ARQMSG_TYPE_ACK;
    msg_ack[ARQMSG_OFFSET_SEQ] = seq;
    msg_ack[2] = 1;

    return ARQMSG_ACKSIZE;
}

uint8_t arqMsg_encodeData(uint8_t* msg_data, uint8_t* data, int seq, int len , uint8_t LSF , uint16_t SO)
{
    msg_data[ARQMSG_OFFSET_TYPE] = ARQMSG_TYPE_DATA;
	
    msg_data[ARQMSG_OFFSET_SEQ] = seq;

	if(LSF ==1)
	{
		msg_data[ARQMSG_OFFSET_LSF] = ARQMSG_TYPE_GOING;
	}
	else
	{
		msg_data[ARQMSG_OFFSET_LSF] = ARQMSG_TYPE_FINISH;
	}
	
	
	if(SO<ARQMSSG_MAX_SEQNUM)
	{
		msg_data[ARQMSG_OFFSET_SO1] = 0;
		msg_data[ARQMSG_OFFSET_SO2] = SO;
	}
	else
	{
		msg_data[ARQMSG_OFFSET_SO1] = SO/ARQMSSG_MAX_SEQNUM;
		msg_data[ARQMSG_OFFSET_SO2] = SO%ARQMSSG_MAX_SEQNUM;

	}
	
	//printf("SO1 :  %d,  SO2: %d \n",msg_data[ARQMSG_OFFSET_SO1],msg_data[ARQMSG_OFFSET_SO2]);
	printf("PDU size: %d\n",len+ARQMSG_OFFSET_DATA);
		
    memcpy(&msg_data[ARQMSG_OFFSET_DATA], data, len*sizeof(uint8_t));

    return len+ARQMSG_OFFSET_DATA;
}

                    
int arqMsg_checkLSF(uint8_t* msg)
{
    return (msg[ARQMSG_OFFSET_LSF] == ARQMSG_TYPE_GOING);
}

uint8_t arqMsg_getSeq(uint8_t* msg)
{
    return msg[ARQMSG_OFFSET_SEQ];
}

uint8_t* arqMsg_getWord(uint8_t* msg)
{
    return &msg[ARQMSG_OFFSET_DATA];
}




