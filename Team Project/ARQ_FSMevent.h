typedef enum arqEvent
{
	arqEvent_dataTxDone = 0,
	arqEvent_ackTxDone = 1,
    arqEvent_dataRcvd = 2,		// PDU IN
    arqEvent_ackRcvd = 3,
    arqEvent_SDUIN	= 4,		// DataToSend
    arqEvent_arqTimeout = 5,
    arqEvent_UPLOAD =6
    
} arqEvent_e;


void arqEvent_setEventFlag(arqEvent_e event);
void arqEvent_clearEventFlag(arqEvent_e event);
void arqEvent_clearAllEventFlag(void);
int arqEvent_checkEventFlag(arqEvent_e event);