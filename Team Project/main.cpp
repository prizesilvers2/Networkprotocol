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
#define MAINSTATE_ACK               2
#define MAINSTATE_REASSEMBLY        3


//GLOBAL variables (DO NOT TOUCH!) ------------------------------------------
//serial port interface
Serial pc(USBTX, USBRX);

//state variables
uint8_t main_state = MAINSTATE_IDLE; //protocol state

//source/destination ID
uint8_t endNode_ID=1;
uint8_t dest_ID=0;


uint8_t sendoption =0;
uint8_t Queue[10][5888] ={0,};

uint8_t SDUINcnt = 0;
uint8_t SENDSDUcnt = 0;
uint16_t SDU_wordlen[10] ={0,};

uint8_t MAX_WORDLEN = 23;
uint8_t fragWord[256][23] ={0,};
uint8_t frag_cnt = 0;
uint8_t seq_cnt = 0;
uint8_t reassem_Word[5888];
uint16_t totalSize = 0;


//PDU context/size
uint8_t arqPdu[200];
uint8_t pduSize;

//SDU (input)
uint8_t originalWord[5888];            // 256 *23 = 5888
uint16_t wordLen=0;

uint16_t wordLen_IDLE=0;
uint16_t wordLen_ACK =0;


//RX 
uint8_t RX_srcId;
uint8_t RX_data[28];
uint8_t RX_size;
uint16_t RX_rvd_SOCnt;
uint8_t RX_flag;
uint8_t RX_dataSize;

uint8_t RX_Seq = 0; 
uint8_t ACK_Seq = 0;



//ARQ parameters -------------------------------------------------------------
uint8_t seqNum = 0;     //ARQ sequence number
uint8_t retxCnt = 0;    //ARQ retransmission counter
uint8_t arqAck[5];      //ARQ ACK PDU

uint8_t LSF =0;
uint8_t SOCnt = 0;
uint8_t HeaderSize = 5;


//application event handler : generating SDU from keyboard input
void arqMain_processInputWord(void)
{
	char c = pc.getc();
    if (!arqEvent_checkEventFlag(arqEvent_SDUIN))
    {
    	if (c == '\n'|| c == '\r')
        {
        	arqEvent_setEventFlag(arqEvent_SDUIN);
           	pc.printf("*-*-*-*-*-*-*-*-*-*-word is ready! ::: %s\n", originalWord);
			pc.printf("originalWord length %i, wordLen_ACK: %i\n",wordLen, wordLen_ACK);
			
        }
        else
        {
        	if(main_state == MAINSTATE_IDLE)
			{
				originalWord[wordLen_IDLE++] = c;
	            if (wordLen_IDLE >= ARQMSG_MAXDATASIZE*ARQMSSG_MAX_SEQNUM-1)
	            {
	                arqEvent_setEventFlag(arqEvent_SDUIN);
	                pc.printf("\n max reached! word forced to be ready :::: %s\n", originalWord);
	            }
			}
			else if(main_state == MAINSTATE_ACK)
			{
				originalWord[wordLen_ACK++] = c;
	            if (wordLen_ACK >= ARQMSG_MAXDATASIZE*ARQMSSG_MAX_SEQNUM-1)
	            {
	                arqEvent_setEventFlag(arqEvent_SDUIN);
	                pc.printf("\n max reached! word forced to be ready :::: %s\n", originalWord);
	            }
			}
        }
    }
}

void clear_fragbuffer(uint8_t buffer[ARQMSSG_MAX_SEQNUM][ARQMSG_MAXDATASIZE], uint8_t seqNum)
{
	uint8_t start = 0;
	memset(&buffer[seqNum][start],0,ARQMSG_MAXDATASIZE*sizeof(uint8_t));
}

uint8_t arqMsg_fragDataQueue(uint8_t     frag_data[ARQMSSG_MAX_SEQNUM][ARQMSG_MAXDATASIZE], uint8_t data[10][5888], int len, uint8_t seqNum, uint8_t SENDSDUcnt)
{
	uint8_t frag_cnt = 0;
	uint8_t start = 0;
	uint8_t data_cnt = 0;
	
	if (len <= ARQMSG_MAXDATASIZE)
	{
	   memcpy(&(frag_data[seqNum][start]), &data[SENDSDUcnt][start], len*sizeof(uint8_t));
	   memcpy(&frag_data[seqNum][len], NULL, sizeof(uint8_t));
	   frag_cnt += 1;
	}
	else
	{
	   	for (uint8_t i = seqNum ; len > ARQMSG_MAXDATASIZE; i++)
	   	{
			memcpy(&frag_data[i][start], &data[SENDSDUcnt][ARQMSG_MAXDATASIZE*data_cnt], ARQMSG_MAXDATASIZE*sizeof(uint8_t));
			memcpy(&frag_data[i][ARQMSG_MAXDATASIZE], NULL, sizeof(uint8_t));
			len -= ARQMSG_MAXDATASIZE;

			for(uint8_t j =0; j<ARQMSG_MAXDATASIZE; j++)
			{
				printf(" %c", frag_data[i][j]);
			}
			 
			printf("frag_data[%d] inside \n",i);

			data_cnt += 1;
	   	}
	   
	  	if(len != 0)
		{
			frag_cnt = data_cnt;
			memcpy(&frag_data[data_cnt+seqNum][start], &data[SENDSDUcnt][ARQMSG_MAXDATASIZE*data_cnt], ARQMSG_MAXDATASIZE*sizeof(uint8_t));
			memcpy(&frag_data[data_cnt+seqNum][ARQMSG_MAXDATASIZE], NULL, sizeof(uint8_t));
			 
			for(uint8_t j =0; j<ARQMSG_MAXDATASIZE; j++)
			{
				printf(" %c", frag_data[data_cnt+seqNum][j]);
			}

			printf("frag_data[%d] inside \n",(data_cnt+seqNum));
		}
		else
		{
			frag_cnt = data_cnt - 1;
		}
	}
	return frag_cnt;
}



//FSM operation implementation ------------------------------------------------
int main(void){

    uint8_t flag_needPrint=1;
    uint8_t prev_state = 0;
	 uint8_t initial_num = 1;
    

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
   	pc.scanf("%d", &sendoption);
   	pc.getc();


   	if(sendoption == 1)
      	pc.printf("send option : %i, You are a sender \n", sendoption);
   	else
      	pc.printf("send option : %i, You are a receiver \n", sendoption);
      
    arqLLI_initLowLayer(endNode_ID);
    
    pc.attach(&arqMain_processInputWord, Serial::RxIrq);
   
   
    while(1)
   {
        //debug message
        if (prev_state != main_state)
        {
            debug_if(DBGMSG_ARQ, "[ARQ] State transition from %i to %i\n", prev_state, main_state);
            prev_state = main_state;
        }

        //FSM should be implemented here! ---->>>>
         if(sendoption == 1){  // TX MODE
           switch (main_state)
           {
               case MAINSTATE_IDLE: //IDLE state description
               if(LSF ==1)
               {   
                  printf("seqNum : %d, frag_cnt: %d,  seq_cnt: %d \n",seqNum, frag_cnt, seq_cnt);
                  
                  if(seqNum == seq_cnt)
                  {
                     LSF = 0;
                     SOCnt = 0;
                     pduSize = arqMsg_encodeData(arqPdu, &fragWord[seqNum][0], seqNum, wordLen-MAX_WORDLEN*frag_cnt, LSF, wordLen);
                     pc.printf("(IDLE STATE) Word length : %i \n",wordLen-MAX_WORDLEN*frag_cnt);
                     wordLen = 0;
                     frag_cnt = 0;

                  }
                  else
                  {
                     SOCnt = SOCnt +1;
                     pduSize = arqMsg_encodeData(arqPdu, &fragWord[seqNum][0], seqNum, MAX_WORDLEN, LSF, MAX_WORDLEN*SOCnt);
                     pc.printf("(IDLE STATE) Word length : %i \n",MAX_WORDLEN);
                  }
                  
                  arqLLI_sendData(arqPdu, pduSize, dest_ID);
                  clear_fragbuffer(fragWord, seqNum);
                     
                  pc.printf("(IDLE STATE) sending to %i (seq:%i)\n", dest_ID, (seqNum)%ARQMSSG_MAX_SEQNUM);
                  pc.printf("(IDLE STATE) SEND seqNum : %i \n",seqNum);

				  printf("Last LSF : %i SENDSDUcnt:%d SDUINcnt :%d \n",LSF, SENDSDUcnt, SDUINcnt);

                  seqNum = (seqNum+1)%ARQMSSG_MAX_SEQNUM ;
                  
                  main_state = MAINSTATE_TX;
                       
                  
               }
               else if (((SDUINcnt!=0)&&(SENDSDUcnt<SDUINcnt))|| arqEvent_checkEventFlag(arqEvent_SDUIN)||(SENDSDUcnt==9 &&SDUINcnt==0)) //if data needs to be sent (keyboard input) : SDU IN
               {	
					printf("SENDSDUcnt:%d SDUINcnt :%d \n",SENDSDUcnt, SDUINcnt);
						
		      		if(arqEvent_checkEventFlag(arqEvent_SDUIN))
				   	{
				   		memcpy(&Queue[SDUINcnt][0], &originalWord[0], sizeof(uint8_t)*5888);
						SDU_wordlen[SDUINcnt] = wordLen_IDLE;
						
						wordLen_IDLE = 0;
			
						memset(&originalWord[0],0,5888*sizeof(uint8_t));						
						SDUINcnt = (SDUINcnt+1)%10;
				   	}
					
                    //fragmentation
                    
                    wordLen = SDU_wordlen[SENDSDUcnt];
                    frag_cnt = arqMsg_fragDataQueue(fragWord, Queue, wordLen, seqNum, SENDSDUcnt);
					memset(&Queue[0][0],0,10*5888*sizeof(uint8_t));
					
					seq_cnt = (frag_cnt+seqNum)%ARQMSSG_MAX_SEQNUM;
						
				    SENDSDUcnt = (SENDSDUcnt+1)%10;
                    
					
		          	//msg header setting
		           	if(wordLen<=MAX_WORDLEN)
		           	{
		                LSF = 0;
		                frag_cnt = 0;
		                pc.printf("original wordlen : %i\n",wordLen);
		                pduSize = arqMsg_encodeData(arqPdu, &fragWord[seqNum][0], seqNum, wordLen, LSF, wordLen);
		                wordLen = 0;

		           	}
		           	else
		           	{
		                LSF = 1;
		                SOCnt = 1;
		                pc.printf("original wordlen!! : %i\n",wordLen);
		                pduSize = arqMsg_encodeData(arqPdu, &fragWord[seqNum][0], seqNum, MAX_WORDLEN, LSF, MAX_WORDLEN*SOCnt);
		           	}

				   	arqLLI_sendData(arqPdu, pduSize, dest_ID);
		           	clear_fragbuffer(fragWord, seqNum);
		              
		           	printf("\n");
		              
		           	pc.printf("(IDLE STATE) sending to %i (seq:%i)\n", dest_ID, (seqNum)%ARQMSSG_MAX_SEQNUM);
		           	pc.printf("(IDLE STATE) SEND seqNum : %i \n",seqNum);

			        seqNum = (seqNum+1)%ARQMSSG_MAX_SEQNUM ;                     
		           	retxCnt = 0;
              
                   	main_state = MAINSTATE_TX;
                   	flag_needPrint = 2;

                   	arqEvent_clearEventFlag(arqEvent_SDUIN);
              
               }

               // ignore events (arqEvent_dataTxDone, arqEvent_ackTxDone, arqEvent_ackRcvd,arqEvent_arqTimeout)
               else if (arqEvent_checkEventFlag(arqEvent_dataTxDone)) // cannot happended
               {
               		//pc.printf("(IDLE STATE) [WARNING] CANNOT HAPPENED \n");
                  	arqEvent_clearEventFlag(arqEvent_dataTxDone);
               }
               else if (arqEvent_checkEventFlag(arqEvent_ackRcvd)) // cannot happended
               {
                  	//pc.printf("(IDLE STATE) [WARNING] CANNOT HAPPENED \n");
                  	arqEvent_clearEventFlag(arqEvent_ackRcvd);
               }
               else if (arqEvent_checkEventFlag(arqEvent_arqTimeout)) // cannot happended
               {
                  	//pc.printf("(IDLE STATE) [WARNING] CANNOT HAPPENED \n");
                  	arqEvent_clearEventFlag(arqEvent_arqTimeout);
               }	
               
               else if (flag_needPrint == 1)
               {
                 	pc.printf("***********(IDLE STATE) Give a word to send : ");
              	 	pc.printf("\n");
				 	
				 	flag_needPrint = 0;
               }		   
			            
               break;

               case MAINSTATE_TX: //TX state description

               if (arqEvent_checkEventFlag(arqEvent_dataTxDone)) //data TX finished
               {
                   	pc.printf("(TX STATE) Send data ! Start the timer!\n");
                   	arqTimer_startTimer();
                   	main_state = MAINSTATE_ACK;
				   
                   	arqEvent_clearEventFlag(arqEvent_dataTxDone);
               }               
               else if(arqEvent_checkEventFlag(arqEvent_ackRcvd))   // can't happened => clear event flag
               {
                  	//pc.printf("(TX STATE) [WARNING] CANNOT HAPPENED \n");
                  	arqEvent_clearEventFlag(arqEvent_ackRcvd);
               }
               else if((arqEvent_checkEventFlag(arqEvent_arqTimeout)))  // can't happened => clear event flag
               {
                  	//pc.printf("(TX STATE) [WARNING] CANNOT HAPPENED \n");
                  	arqEvent_clearEventFlag(arqEvent_arqTimeout);
                  
               }
			   else if (arqEvent_checkEventFlag(arqEvent_SDUIN))  // ignore
               {                
                  	//pc.printf("(TX STATE) [WARNING] CANNOT HAPPENED \n");
					arqEvent_clearEventFlag(arqEvent_SDUIN); 
               }
               else // wait 
               {   
                  	main_state = MAINSTATE_TX;
               }
               
              break;

              case MAINSTATE_ACK:   //waiting ACK

			   if(flag_needPrint ==2)
			   {             
                  	pc.printf("***********(ACK STATE) Give a word to send : ");
                  	pc.printf("\n");
				  	flag_needPrint =1;
			   }
               if (arqEvent_checkEventFlag(arqEvent_SDUIN)) 
               {
	              	memcpy(&Queue[SDUINcnt][0], &originalWord, sizeof(uint8_t)*5888);
				  	SDU_wordlen[SDUINcnt] = wordLen_ACK;

					wordLen_ACK = 0;

					memset(&originalWord[0],0,5888*sizeof(uint8_t));
	              	SDUINcnt= (SDUINcnt+1)%10;

				  	arqEvent_clearEventFlag(arqEvent_SDUIN); 
               }
               
               if(arqEvent_checkEventFlag(arqEvent_ackRcvd))   // ACK receieved
               {               
                  	uint8_t* TX_dataPtr = arqLLI_getRcvdDataPtr();
                  	if(arqMsg_getSeq(arqPdu) == arqMsg_getSeq(TX_dataPtr))
                  	{
                     	pc.printf("(ACK STATE) ACK is correctly received!\n");
                     	arqTimer_stopTimer();
                     	main_state = MAINSTATE_IDLE;
					 	flag_needPrint = 1;
				  	}
                  	else
                  	{
                     	pc.printf("(ACK STATE) ACK seq number is weird! (expected : %i, received : %i )\n", arqMsg_getSeq(arqPdu), arqMsg_getSeq(TX_dataPtr));
                  	}
                  
                  	arqEvent_clearEventFlag(arqEvent_ackRcvd);
               }
               else if(arqEvent_checkEventFlag(arqEvent_arqTimeout))   //Time out
               {   
                  	if(retxCnt >= ARQ_MAXRETRANSMISSION)
                  	{
                     	pc.printf("(ACK STATE) Failed to send data &i , max retxcnt reached \n", arqMsg_getSeq(arqPdu));
                     	main_state = MAINSTATE_IDLE;

						LSF = 0;
						retxCnt = 0;

						for(uint8_t i= seqNum; i<=seq_cnt;i++)
						{
							clear_fragbuffer(fragWord, i);	
						}
                  	}
                  	else    //retx < max, then goto TX for retransmission
                  	{
                     	arqLLI_sendData(arqPdu, pduSize, dest_ID);
                     	//setting ARQ parameter
                     	retxCnt = retxCnt+1;
                     	pc.printf("(ACK STATE) RETXCNT : %i ! TIME OUT ! STATE change TX!\n", retxCnt);
                     	main_state = MAINSTATE_TX;
                  	}

					arqEvent_clearEventFlag(arqEvent_arqTimeout);
               }
               else if (arqEvent_checkEventFlag(arqEvent_dataTxDone)) 
               {
                  //pc.printf("(ACK STATE) [WARNING] CANNOT HAPPENED \n");
                  	arqEvent_clearEventFlag(arqEvent_dataTxDone);
               }
               
               break;
            
               default :
                   break;
              }
        }
      else
      {                  // RX MODE
         switch (main_state)
	     {
			case MAINSTATE_IDLE: //IDLE state description
				if (arqEvent_checkEventFlag(arqEvent_dataRcvd)) //if data reception event happens :PDU IN
	            {
					uint8_t* RX_dataPtr = arqLLI_getRcvdDataPtr();
					RX_srcId = arqLLI_getSrcId();
					RX_size = arqLLI_getSize();
					RX_dataSize = RX_size - HeaderSize;
					RX_flag = RX_dataPtr[2];
					RX_rvd_SOCnt = ARQMSSG_MAX_SEQNUM*RX_dataPtr[3] + RX_dataPtr[4];
					seqNum = arqMsg_getSeq(RX_dataPtr);

					memcpy(&RX_data[0],&RX_dataPtr[5],RX_dataSize*sizeof(uint8_t));
						
					pc.printf("\n ---------\(IDLE STATE) nRCVD from %i, seq: %i, size : %i, rvd_SOCnt : %i, flag : %i, dataSize : %i---------\n\n",
												RX_srcId, seqNum, RX_size, RX_rvd_SOCnt, RX_flag, RX_dataSize);

					if((initial_num == 1 && RX_rvd_SOCnt > 23))
					{
						pc.printf("Ignore the data because it is broken.\n");
						arqEvent_clearEventFlag(arqEvent_dataRcvd);
						continue;
					}							
												
					arqMsg_encodeAck(arqAck, arqMsg_getSeq(RX_dataPtr));
					arqLLI_sendData(arqAck, ARQMSG_ACKSIZE, RX_srcId);
					
					memset(&RX_dataPtr[0],0,ARQLLI_MAX_PDUSIZE*sizeof(uint8_t));
					
					main_state = MAINSTATE_TX;						
                    arqEvent_clearEventFlag(arqEvent_dataRcvd);
	            }

				else if(arqEvent_checkEventFlag(arqEvent_ackTxDone))
				{
					//pc.printf("(TX STATE) [WARNING] CANNOT HAPPENED \n");
					arqEvent_clearEventFlag(arqEvent_ackTxDone);	
				}
				else if(arqEvent_checkEventFlag(arqEvent_arqTimeout))
				{
					pc.printf("(IDLE STATE) Time out ! Clear the Reassembly buffer!\n");
					
					memset(&reassem_Word[0],0,5888*sizeof(uint8_t));
					totalSize = 0;
					
					arqEvent_clearEventFlag(arqEvent_arqTimeout);
					main_state = MAINSTATE_IDLE;
				}				
				else if(arqEvent_checkEventFlag(arqEvent_UPLOAD))
				{
					//pc.printf("(TX STATE) [WARNING] CANNOT HAPPENED \n");
					arqEvent_clearEventFlag(arqEvent_UPLOAD);
				}				
	            break;

			case MAINSTATE_TX :
				if(arqEvent_checkEventFlag(arqEvent_ackTxDone)) //ack TX finished
				{	

					pc.printf("(TX STATE) ACK seqNum : %i \n",seqNum);
					pc.printf("(TX STATE) RX_Seq : %i, received seqNum : %i, initial : %i\n",RX_Seq, seqNum, initial_num);

					if( RX_flag == 1 )// && RX_rvd_SOCnt == 23 
					{
						pc.printf("(TX STATE) Recived first data ! Start the timer!\n");
						RX_arqTimer_startTimer();
					}

					if( (seqNum == RX_Seq && initial_num != 1) ) //|| (initial_num == 1 && RX_rvd_SOCnt > 23)
					{
						pc.printf("Ignore this because the same sequence number is entered.\n");
						main_state = MAINSTATE_IDLE;
					}
					else if( seqNum == RX_Seq + 1 || initial_num == 1 ) // correct sequence
					{
						if(totalSize != 0 && RX_rvd_SOCnt == 23 )
						{	
							pc.printf("(TX STATE) Sequence number is weird ! Clear the Reassembly buffer!\n");
					
							memset(&reassem_Word[0],0,5888*sizeof(uint8_t));
							totalSize = 0;
							
							pc.printf("(TX STATE) start to store new data !\n");
							
							main_state = MAINSTATE_REASSEMBLY;
							arqEvent_setEventFlag(arqEvent_UPLOAD);
						}
						else
						{
							if (initial_num == 1 )
							{
								initial_num = 0;
							}
							main_state = MAINSTATE_REASSEMBLY;
							arqEvent_setEventFlag(arqEvent_UPLOAD);
						}
					}
					else
					{
						pc.printf("(TX STATE) Sequence number is weird ! Clear the Reassembly buffer!\n");
					
						memset(&reassem_Word[0],0,5888*sizeof(uint8_t));
						totalSize = 0;
						
						pc.printf("(TX STATE) start to store new data !\n");
						
						main_state = MAINSTATE_REASSEMBLY;
						arqEvent_setEventFlag(arqEvent_UPLOAD);
					}
					
					arqEvent_clearEventFlag(arqEvent_ackTxDone);						
				}
					
				else if(arqEvent_checkEventFlag(arqEvent_dataRcvd))
				{
					//pc.printf("(TX STATE) [WARNING] CANNOT HAPPENED \n");
					arqEvent_clearEventFlag(arqEvent_dataRcvd);
				}
				else if(arqEvent_checkEventFlag(arqEvent_arqTimeout))
				{
					pc.printf("(TX STATE) Time out ! Clear the Reassembly buffer!\n");
					
					memset(&reassem_Word[0],0,5888*sizeof(uint8_t));
					totalSize = 0;
					
					main_state = MAINSTATE_IDLE;
					arqEvent_clearEventFlag(arqEvent_arqTimeout);
				}
				else if(arqEvent_checkEventFlag(arqEvent_UPLOAD))
				{
					//pc.printf("(TX STATE) [WARNING] CANNOT HAPPENED \n");
					arqEvent_clearEventFlag(arqEvent_UPLOAD);
				}
					
				break;


			case MAINSTATE_REASSEMBLY:
				if(arqEvent_checkEventFlag(arqEvent_UPLOAD))
				{
					RX_Seq = seqNum;
					totalSize = totalSize + RX_dataSize;
					
					if( RX_flag == 1 )
					{
						memcpy(&reassem_Word[RX_rvd_SOCnt-RX_dataSize],&RX_data[0],RX_dataSize*sizeof(uint8_t));
					}
					else
					{
						arqTimer_stopTimer();
						pc.printf("(Reassembly STATE) Stop the timer!\n");
						memcpy(&reassem_Word[RX_rvd_SOCnt-RX_dataSize],&RX_data[0],RX_dataSize*sizeof(uint8_t));
						pc.printf("\n -------------------------------------------------\nRCVD from %i : %s (length:%i, seq:%i)\n -------------------------------------------------\n", 
                           					RX_srcId, reassem_Word, totalSize, seqNum);

						memset(&reassem_Word[0],0,5888*sizeof(uint8_t));
						totalSize = 0;
					}

					memset(&RX_data[0],0,28*sizeof(uint8_t));
					main_state = MAINSTATE_IDLE;
					arqEvent_clearEventFlag(arqEvent_UPLOAD);
				}
				else if(arqEvent_checkEventFlag(arqEvent_arqTimeout))
				{
					pc.printf("(Reassembly STATE) Time out ! Clear the Reassembly buffer!\n");
					
					memset(&reassem_Word[0],0,5888*sizeof(uint8_t));
					totalSize = 0;
					
					arqEvent_clearEventFlag(arqEvent_arqTimeout);
					main_state = MAINSTATE_IDLE;
				}
				else if(arqEvent_checkEventFlag(arqEvent_dataRcvd))
				{
					pc.printf("(Reassembly STATE) [WARNING] CANNOT HAPPENED 2222 \n");
					arqEvent_clearEventFlag(arqEvent_dataRcvd);
				}
				
				else if(arqEvent_checkEventFlag(arqEvent_ackTxDone))
				{
				    pc.printf("(Reassembly STATE) [WARNING] CANNOT HAPPENED 3333 \n");
					arqEvent_clearEventFlag(arqEvent_ackTxDone);
				}

				break;

				default : 
					  break;
		}
      }
   }
}
