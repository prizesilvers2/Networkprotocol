#include "mbed.h"
#include "string.h"

#include "ARQ_FSMevent.h"
#include "ARQ_msg.h"
#include "ARQ_timer.h"
#include "ARQ_LLinterface.h"
#include "ARQ_parameters.h"

//FSM state -------------------------------------------------
#define MAINSTATE_IDLE              0
#define MAINSTATE_TX                1
#define MAINSTATE_ACK				2

//GLOBAL variables (DO NOT TOUCH!) ------------------------------------------
//serial port interface
Serial pc(USBTX, USBRX);

//state variables
uint8_t main_state = MAINSTATE_IDLE; //protocol state

//source/destination ID
uint8_t endNode_ID=1;
uint8_t dest_ID=0;

//PDU context/size
uint8_t arqPdu[200];
uint8_t pduSize;
uint8_t ackSize;

//SDU (input)
uint8_t originalWord[200];
uint8_t wordLen=0;

//ARQ parameters -------------------------------------------------------------
uint8_t seqNum = 0;     //ARQ sequence number
uint8_t retxCnt = 0;    //ARQ retransmission counter
uint8_t arqAck[5];      //ARQ ACK PDU


//application event handler : generating SDU from keyboard input
void arqMain_processInputWord(void)
{
    char c = pc.getc();
    if (main_state == MAINSTATE_IDLE &&
        !arqEvent_checkEventFlag(arqEvent_dataToSend))
    {
        if (c == '\n' || c == '\r')
        {
            originalWord[wordLen++] = '\0';
            arqEvent_setEventFlag(arqEvent_dataToSend);
            pc.printf("word is ready! ::: %s\n", originalWord);
        }
        else
        {
            originalWord[wordLen++] = c;
            if (wordLen >= ARQMSG_MAXDATASIZE-1)
            {
                originalWord[wordLen++] = '\0';
                arqEvent_setEventFlag(arqEvent_dataToSend);
                pc.printf("\n max reached! word forced to be ready :::: %s\n", originalWord);
            }
        }
    }
}




//FSM operation implementation ------------------------------------------------
int main(void){
    uint8_t flag_needPrint=1;
    uint8_t prev_state = 0;

    //initialization
    pc.printf("------------------ ARQ protocol starts! --------------------------\n");
    arqEvent_clearAllEventFlag();
    
    //source & destination ID setting
    pc.printf(":: ID for this node : ");
    pc.scanf("%d", &endNode_ID);
    pc.printf(":: ID for the destination : ");
    pc.scanf("%d", &dest_ID);
    pc.getc();

    pc.printf("endnode : %i, dest : %i\n", endNode_ID, dest_ID);

    arqLLI_initLowLayer(endNode_ID);
    pc.attach(&arqMain_processInputWord, Serial::RxIrq);

	int time_status = 0;
	
    while(1)
    {
        //debug message
        if (prev_state != main_state)
        {
            debug_if(DBGMSG_ARQ, "[ARQ] State transition from %i to %i\n", prev_state, main_state);
            prev_state = main_state;
        }

        //FSM should be implemented here! ---->>>>
        switch (main_state)
        {
            case MAINSTATE_IDLE: //IDLE state description
                
                if (arqEvent_checkEventFlag(arqEvent_dataRcvd)) //if data reception event happens :PDU IN
                {
                    //Retrieving data info.
                    uint8_t srcId = arqLLI_getSrcId();
                    uint8_t* dataPtr = arqLLI_getRcvdDataPtr();
                    uint8_t size = arqLLI_getSize();

                    pc.printf("\n -------------------------------------------------\(IDLE STATE) nRCVD from %i : %s (length:%i, seq:%i)\n -------------------------------------------------\n", 
                                srcId, arqMsg_getWord(dataPtr), size, arqMsg_getSeq(dataPtr));
					
					arqMsg_encodeAck(arqAck, arqMsg_getSeq(dataPtr));
					arqLLI_sendData(arqAck, ARQMSG_ACKSIZE, srcId);

					pc.printf("(IDLE STATE) RECEIVE seqNum : %i \n",seqNum);
					
                    main_state = MAINSTATE_TX;
                    flag_needPrint = 1;

                    arqEvent_clearEventFlag(arqEvent_dataRcvd);
                }
                else if (arqEvent_checkEventFlag(arqEvent_dataToSend)) //if data needs to be sent (keyboard input) : SDU IN
                {
                    //msg header setting
                    pduSize = arqMsg_encodeData(arqPdu, originalWord, seqNum, wordLen);
                    arqLLI_sendData(arqPdu, pduSize, dest_ID);
					
                    pc.printf("(IDLE STATE) sending to %i (seq:%i)\n", dest_ID, (seqNum-1)%ARQMSSG_MAX_SEQNUM);
					pc.printf("(IDLE STATE) SEND seqNum : %i \n",seqNum);

					seqNum = (seqNum+1)%ARQMSSG_MAX_SEQNUM ;
					retxCnt = 0;
					
                    main_state = MAINSTATE_TX;
                    flag_needPrint = 1;

                    wordLen = 0;
                    arqEvent_clearEventFlag(arqEvent_dataToSend);
					
                }

				// ignore events (arqEvent_dataTxDone, arqEvent_ackTxDone, arqEvent_ackRcvd,arqEvent_arqTimeout)
				else if (arqEvent_checkEventFlag(arqEvent_dataTxDone)) // cannot happended
				{
					pc.printf("(IDLE STATE) [WARNING] CANNOT HAPPENED \n");
					arqEvent_clearEventFlag(arqEvent_dataTxDone);
				}
				else if (arqEvent_checkEventFlag(arqEvent_ackTxDone)) // cannot happended
				{
					pc.printf("(IDLE STATE) [WARNING] CANNOT HAPPENED \n");
					arqEvent_clearEventFlag(arqEvent_ackTxDone);
				}
				else if (arqEvent_checkEventFlag(arqEvent_ackRcvd)) // cannot happended
				{
					pc.printf("(IDLE STATE) [WARNING] CANNOT HAPPENED \n");
					arqEvent_clearEventFlag(arqEvent_ackRcvd);
				}
				else if (arqEvent_checkEventFlag(arqEvent_arqTimeout)) // cannot happended
				{
					pc.printf("(IDLE STATE) [WARNING] CANNOT HAPPENED \n");
					arqEvent_clearEventFlag(arqEvent_arqTimeout);
				}
				
                else if (flag_needPrint == 1)
                {
                    pc.printf("(IDLE STATE) Give a word to send : ");
                    flag_needPrint = 0;
                }
				
                break;

            case MAINSTATE_TX: //TX state description

				time_status = arqTimer_getTimerStatus();
				
				if(arqEvent_checkEventFlag(arqEvent_ackTxDone)) //ack TX finished
				{
					if((time_status == 1) || (arqEvent_checkEventFlag(arqEvent_arqTimeout)))
					{
						pc.printf("(TX STATE) Before state: Idle, Next state: ACK \n");
						main_state = MAINSTATE_ACK;						
					}
					else
					{
						pc.printf("(TX STATE) Before state: ACK, Next state: ACK \n");
						main_state = MAINSTATE_IDLE;
					}

					arqEvent_clearEventFlag(arqEvent_ackTxDone);
					
				}
				else if (arqEvent_checkEventFlag(arqEvent_dataTxDone)) //data TX finished
                {
                	pc.printf("(TX STATE) Send data ! Start the timer!\n");
					arqTimer_startTimer();
				   
					main_state = MAINSTATE_ACK;
                    arqEvent_clearEventFlag(arqEvent_dataTxDone);
                }
				
				#if 1
				else if((arqEvent_checkEventFlag(arqEvent_arqTimeout)))  // can't happened => clear event flag
				{
					pc.printf("(TX STATE) [WARNING] CANNOT HAPPENED \n");
					arqEvent_clearEventFlag(arqEvent_arqTimeout);
					
				}
				
				else if(arqEvent_checkEventFlag(arqEvent_ackRcvd))	// can't happened => clear event flag
				{
					pc.printf("(TX STATE) [WARNING] CANNOT HAPPENED \n");
					arqEvent_clearEventFlag(arqEvent_ackRcvd);
				}
				else	// wait 
				{	
					main_state = MAINSTATE_TX;
				}
				#endif
				
                break;

			case MAINSTATE_ACK:	//waiting ACK

				if(arqEvent_checkEventFlag(arqEvent_ackRcvd))	// ACK receieved
				{					
					// if(arqEvent_checkEventFlag(arqEvent_arqTimeout)==0)   => corner case

					uint8_t* dataPtr = arqLLI_getRcvdDataPtr();
					if(arqMsg_getSeq(arqPdu) == arqMsg_getSeq(dataPtr))
					{
						pc.printf("(ACK STATE) ACK is correctly received!\n");
						arqTimer_stopTimer();
						main_state = MAINSTATE_IDLE;
					}
					else
					{
						pc.printf("(ACK STATE) ACK seq number is weird! (expected : %i, received : %i )\n", arqMsg_getSeq(arqPdu), arqMsg_getSeq(dataPtr));
					}
					
					arqEvent_clearEventFlag(arqEvent_ackRcvd);
				}
				else if(arqEvent_checkEventFlag(arqEvent_arqTimeout))
				{	
					if(retxCnt >= ARQ_MAXRETRANSMISSION)
					{
						pc.printf("(ACK STATE) Failed to send data &i , max retxcnt reached \n", arqMsg_getSeq(arqPdu));
						main_state = MAINSTATE_IDLE;
						// arqPdu clear
						// retxCnt clear
					}
					else 	//retx < max, then goto TX for retransmission
					{
						arqLLI_sendData(arqPdu, pduSize, dest_ID);
						//setting ARQ parameter
						retxCnt = retxCnt+1;
						pc.printf("(ACK STATE) RETXCNT : %i ! TIME OUT ! STATE change TX!\n", retxCnt);
						main_state = MAINSTATE_TX;
					}
					arqEvent_clearEventFlag(arqEvent_arqTimeout);
				}
				else if(arqEvent_checkEventFlag(arqEvent_dataRcvd)) // NOT TIME OUT &&DATA IN
				{
					//Retrieving data info.
                    uint8_t srcId = arqLLI_getSrcId();
                    uint8_t* dataPtr = arqLLI_getRcvdDataPtr();
                    uint8_t size = arqLLI_getSize();

                    pc.printf("\n -------------------------------------------------\nRCVD from %i : %s (length:%i, seq:%i)\n -------------------------------------------------\n", 
                                srcId, arqMsg_getWord(dataPtr), size, arqMsg_getSeq(dataPtr));

                    //ACK transmission
                    arqMsg_encodeAck(arqAck, arqMsg_getSeq(dataPtr));
                    arqLLI_sendData(arqAck, ARQMSG_ACKSIZE, srcId);

					pc.printf("(ACK STATE) RECEIVE seqNum : %i \n",seqNum);
					
                    main_state = MAINSTATE_TX; //goto TX state

                    arqEvent_clearEventFlag(arqEvent_dataRcvd);
					
				}
				else	// cannot happened
				{
					pc.printf("(ACK STATE) [WARNING] CANNOT HAPPENED \n");
					arqEvent_clearAllEventFlag();
				}
			
				break;
			
            default :
                break;
        }

    }
}