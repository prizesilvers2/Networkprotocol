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

//i made
uint8_t txbuffer[200];
uint8_t tx_wordLen=0;
uint8_t tx_pduSize;
uint8_t tx_arqPdu[200];




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

	pc.printf("------------------If you are a sender, please press 1--------------------------\n");

    arqLLI_initLowLayer(endNode_ID);
    pc.attach(&arqMain_processInputWord, Serial::RxIrq);


	//i made
	int time_status = 0;
	int max_reCnt = 10;


    while(1)
    {
        //debug message
        if (prev_state != main_state)
        {
            debug_if(DBGMSG_ARQ, "[ARQ] State transition from %i to %i\n", prev_state, main_state);
            prev_state = main_state;
        }

#if 1
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
					
					ackSize = arqMsg_encodeAck(arqAck, seqNum);
					arqLLI_sendData(arqAck, ackSize, endNode_ID);

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

					tx_pduSize = arqMsg_encodeData(tx_arqPdu, originalWord, seqNum, tx_wordLen);
					
                    pc.printf("(IDLE STATE) sending to %i (seq:%i)\n", dest_ID, (seqNum-1)%ARQMSSG_MAX_SEQNUM);
					pc.printf("(IDLE STATE) SEND seqNum : %i \n",seqNum);

					seqNum++;
					
                    main_state = MAINSTATE_TX;
                    flag_needPrint = 1;

                    wordLen = 0;
                    arqEvent_clearEventFlag(arqEvent_dataToSend);
                }
				#if 0
				else if (arqEvent_checkEventFlag(arqEvent_dataTxDone)) // cannot happended
				{
					arqEvent_clearEventFlag(arqEvent_dataTxDone);
				}
				else if (arqEvent_checkEventFlag(arqEvent_ackTxDone)) // cannot happended
				{
					arqEvent_clearEventFlag(arqEvent_ackTxDone);
				}
				else if (arqEvent_checkEventFlag(arqEvent_ackRcvd)) // cannot happended
				{
					arqEvent_clearEventFlag(arqEvent_ackRcvd);
				}
				else if (arqEvent_checkEventFlag(arqEvent_arqTimeout)) // cannot happended
				{
					arqEvent_clearEventFlag(arqEvent_arqTimeout);
				}
				#endif
				
                else if (flag_needPrint == 1)
                {
                    pc.printf("(IDLE STATE) Give a word to send : ");
                    flag_needPrint = 0;
                }
				
                break;

            case MAINSTATE_TX: //TX state description

				time_status = arqTimer_getTimerStatus();

				if (arqEvent_checkEventFlag(arqEvent_dataTxDone)) //data TX finished
                {
                	pc.printf("(TX STATE) Send data ! Start the timer!\n");
					arqTimer_startTimer();
				    flag_needPrint = 1;
					main_state = MAINSTATE_ACK;
                    arqEvent_clearEventFlag(arqEvent_dataTxDone);
                }
				else if(arqEvent_checkEventFlag(arqEvent_ackTxDone)) //ack TX finished
				{
					if(time_status == 0)
					{
						pc.printf("(TX STATE) Before state: Idle, Next state: ACK \n");
                    	flag_needPrint = 1;
						main_state = MAINSTATE_IDLE;
						arqEvent_clearEventFlag(arqEvent_ackTxDone);
					}
					else
					{
						pc.printf("(TX STATE) Before state: ACK, Next state: ACK \n");
                    	flag_needPrint = 1;
						main_state = MAINSTATE_ACK;
						arqEvent_clearEventFlag(arqEvent_ackTxDone);
					}
				}
				else if((arqEvent_checkEventFlag(arqEvent_arqTimeout))&&(retxCnt!=0))  //????
				{
					arqTimer_startTimer();
					arqLLI_sendData(tx_arqPdu, tx_pduSize, dest_ID);
					
				}
				else if(arqEvent_checkEventFlag(arqEvent_ackRcvd))	// can't happened => clear event flag
				{
					arqEvent_clearEventFlag(arqEvent_ackRcvd);
				}
				else	// wait 
				{	
                    flag_needPrint = 1;
					main_state = MAINSTATE_TX;
				}
                break;

			case MAINSTATE_ACK:	//waiting ACK

				if(arqEvent_checkEventFlag(arqEvent_ackRcvd))	// ACK in
				{
					pc.printf("(ACK STATE) ACK IN ! STOP THE TIMER ! STATE change Idle!\n");
					arqTimer_stopTimer();
					flag_needPrint = 1;
					retxCnt = 0;
					main_state = MAINSTATE_IDLE;
					arqEvent_clearEventFlag(arqEvent_ackRcvd);
				}
				else if((arqEvent_checkEventFlag(arqEvent_dataRcvd)) && (time_status==1)) // NOT TIME OUT &&DATA IN
				{
					pc.printf("(ACK STATE) DATA IN ! KEEP GOING THE TIMER ! STATE change TX !\n");
                    flag_needPrint = 1;
					main_state = MAINSTATE_TX;	
					arqEvent_clearEventFlag(arqEvent_dataRcvd);
				}
				else if(retxCnt <= max_reCnt) //time out state
				{
					retxCnt++;
					pc.printf("(ACK STATE) RETXCNT : %i ! TIME OUT ! STATE change TX!\n", retxCnt);
					flag_needPrint = 1;
					main_state = MAINSTATE_TX;
					arqTimer_stopTimer();
					// arqEvent_clearEventFlag(arqEvent_arqTimeout); //???????
				}
				else if(retxCnt > max_reCnt)
				{
					pc.printf("(ACK STATE) TOO MANY RETXCNT : %i, TIME OUT ! STATE change Idle!\n", retxCnt);
					flag_needPrint = 1;
					retxCnt = 0;					
					tx_wordLen = 0;
					main_state = MAINSTATE_IDLE;
					arqEvent_clearEventFlag(arqEvent_arqTimeout);
					
				}
				else if(arqEvent_checkEventFlag(arqEvent_dataToSend)) // wait
				{
					flag_needPrint = 1;
					main_state = MAINSTATE_ACK;
				}
				else	// cannot happened
				{
					arqEvent_clearAllEventFlag();
				}
			
				break;
			
            default :
                break;
        }
#else 
		
	switch (main_state)
	{
		case MAINSTATE_IDLE: //IDLE state description
			
			if (arqEvent_checkEventFlag(arqEvent_dataRcvd)) //if data reception event happens
			{
				//Retrieving data info.
				uint8_t srcId = arqLLI_getSrcId();
				uint8_t* dataPtr = arqLLI_getRcvdDataPtr();
				uint8_t size = arqLLI_getSize();
	
				pc.printf("\n -------------------------------------------------\nRCVD from %i : %s (length:%i, seq:%i)\n -------------------------------------------------\n", 
							srcId, arqMsg_getWord(dataPtr), size, arqMsg_getSeq(dataPtr));
	
				main_state = MAINSTATE_TX;
				flag_needPrint = 1;
	
				arqEvent_clearEventFlag(arqEvent_dataRcvd);
			}
			else if (arqEvent_checkEventFlag(arqEvent_dataToSend)) //if data needs to be sent (keyboard input)
			{
				//msg header setting
				pduSize = arqMsg_encodeData(arqPdu, originalWord, seqNum, wordLen);
				arqLLI_sendData(arqPdu, pduSize, dest_ID);
	
				pc.printf("[MAIN] sending to %i (seq:%i)\n", dest_ID, (seqNum-1)%ARQMSSG_MAX_SEQNUM);
	
				main_state = MAINSTATE_TX;
				flag_needPrint = 1;
	
				wordLen = 0;
				arqEvent_clearEventFlag(arqEvent_dataToSend);
			}
			else if (arqEvent_checkEventFlag(arqEvent_dataTxDone)) 
			{
				//return IDLE
				main_state = MAINSTATE_IDLE;
				flag_needPrint = 1;
	
				arqEvent_clearEventFlag(arqEvent_dataTxDone);
			}
			else if (arqEvent_checkEventFlag(arqEvent_ackTxDone)) 
			{
				//return IDLE
				main_state = MAINSTATE_IDLE;
				flag_needPrint = 1;
	
				arqEvent_clearEventFlag(arqEvent_ackTxDone);
			}
			else if (arqEvent_checkEventFlag(arqEvent_ackRcvd)) 
			{
				//return IDLE
				main_state = MAINSTATE_IDLE;
				flag_needPrint = 1;
	
				arqEvent_clearEventFlag(arqEvent_ackRcvd);
			}
			else if (arqEvent_checkEventFlag(arqEvent_arqTimeout)) 
			{
				//return IDLE
				main_state = MAINSTATE_IDLE;
				flag_needPrint = 1;
	
				arqEvent_clearEventFlag(arqEvent_arqTimeout);
			}
			else if (flag_needPrint == 1)
			{
				pc.printf("Give a word to send : ");
				flag_needPrint = 0;
			}	  
	
			break;
	
		case MAINSTATE_TX: //TX state description
	
			if (arqEvent_checkEventFlag(arqEvent_dataTxDone)) //data TX finished
			{
				main_state = MAINSTATE_ACK;
				if(arqTimer_getTimerStatus() == 0)
				{
					arqTimer_startTimer();
				}	
				arqEvent_clearEventFlag(arqEvent_dataTxDone);
			}
			else if (arqEvent_checkEventFlag(arqEvent_ackTxDone)) //ack TX finished
			{
				//return IDLE
				main_state = MAINSTATE_IDLE;
				arqEvent_clearEventFlag(arqEvent_ackTxDone);
			}
			else if (arqEvent_checkEventFlag(arqEvent_dataToSend))
			{
				main_state = MAINSTATE_TX;
			}
			else if (arqEvent_checkEventFlag(arqEvent_dataRcvd))
			{
				main_state = MAINSTATE_TX;
			}
			else if (arqEvent_checkEventFlag(arqEvent_ackRcvd))
			{
				if (arqEvent_checkEventFlag(arqEvent_dataTxDone))
				{
					main_state = MAINSTATE_TX;
					arqEvent_clearEventFlag(arqEvent_ackRcvd);
				}
				else if (arqEvent_checkEventFlag(arqEvent_ackTxDone))
				{
					main_state = MAINSTATE_TX;
				}
			}
			else if (arqEvent_checkEventFlag(arqEvent_arqTimeout))
			{
				main_state = MAINSTATE_TX;
			}
	
			break;
	
		case MAINSTATE_ACK: //ACK state description
	
			if (arqEvent_checkEventFlag(arqEvent_dataRcvd))
			{
				main_state = MAINSTATE_TX;
				arqEvent_clearEventFlag(arqEvent_dataRcvd);
			}
			else if (arqEvent_checkEventFlag(arqEvent_ackRcvd))
			{
				main_state = MAINSTATE_IDLE;
				arqEvent_clearEventFlag(arqEvent_ackRcvd);
			}
			else if (arqEvent_checkEventFlag(arqEvent_arqTimeout))
			{
				if (retxCnt < 10)
				{
					main_state = MAINSTATE_TX;
					retxCnt =+ 1;
					arqEvent_clearEventFlag(arqEvent_arqTimeout);
				}
				else
				{
					main_state = MAINSTATE_IDLE;
					retxCnt = 0;
					arqEvent_clearAllEventFlag();
				}
			}
			else if (arqEvent_checkEventFlag(arqEvent_dataToSend))
			{
				main_state = MAINSTATE_ACK;
			}
			else if (arqEvent_checkEventFlag(arqEvent_dataTxDone))
			{
				main_state = MAINSTATE_ACK;
				arqEvent_clearEventFlag(arqEvent_dataTxDone);
			}
			else if (arqEvent_checkEventFlag(arqEvent_ackTxDone))
			{
				main_state = MAINSTATE_ACK;
				arqEvent_clearEventFlag(arqEvent_ackTxDone);
			}
	
			break;
	
		default :
			break;
	}
	
#endif		
    }
}