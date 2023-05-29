#include "statemachine_cc.h"

extern const AP_HAL::HAL& hal;

/*
    generate CRC code
*/
static const uint8_t aucCRCHi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40
};

static const uint8_t aucCRCLo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,
    0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
    0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9,
    0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
    0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,
    0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 
    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF,
    0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,
    0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 
    0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
    0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97,
    0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
    0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89,
    0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
    0x41, 0x81, 0x80, 0x40
};


uint16_t usMBCRC16( uint8_t * pucFrame, uint16_t usLen )
{
    uint8_t           ucCRCHi = 0xFF;
    uint8_t           ucCRCLo = 0xFF;
    int             iIndex;

    while( usLen-- )
    {
        iIndex = ucCRCLo ^ *( pucFrame++ );
        ucCRCLo = ( uint8_t )( ucCRCHi ^ aucCRCHi[iIndex] );
        ucCRCHi = aucCRCLo[iIndex];
    }
    return ( uint16_t )( ucCRCHi << 8 | ucCRCLo );
}

void generateCRC(uint8_t *buffer)
{
    uint16_t tempCRC = 0;
    tempCRC = usMBCRC16(buffer, 6);
    // hal.serial(1)->printf("CRC = %x\n", tempCRC);
    buffer[6] = tempCRC & 0xFF;
    buffer[7] = (tempCRC & 0xFF00) >> 8;
}



/*
    ACTION_MAP_t instantiation array
*/
ACTION_MAP_t actionMap[] = 
{
	{STATE0,	state0_entry,	state0_do,	state0_exit},
	{STATE1,	state1_entry,	state1_2_3_4_do,	state1_exit},
	{STATE2,	state2_entry,	state1_2_3_4_do,	state2_exit},
	{STATE3,	state3_entry,	state1_2_3_4_do,	state3_exit},
    {STATE4,	state4_entry,	state1_2_3_4_do,	state4_exit},
};


/*
    EVENT_MAP_t instantiation array:
*/
EVENT_MAP_t eventMap[] = 
{
	{EVENT0,	STATE0,	STATE1},
	{EVENT1,	STATE1,	STATE0},
    {EVENT1,	STATE2,	STATE0},
    {EVENT1,	STATE3,	STATE0},
    {EVENT1,	STATE4,	STATE0},	
	{EVENT2,	STATE1,	STATE2},
    {EVENT2,	STATE4,	STATE2},
	{EVENT3,	STATE1,	STATE3},
    {EVENT3,	STATE4,	STATE3},
	{EVENT4,	STATE2,	STATE4},
    {EVENT5,	STATE3,	STATE4},
	{EVENT_MAP_END,	STATE_END,	STATE_END},		
};



extern FSM_t stFsmWeather;	//定义状态机
static uint16_t cpgPhase;   //bit 15-12 : represent current state, bit 11 - 0 : represent cpg synchronization phase
static uint8_t modbusFrame[8]={0x0, 0x06, 0x0, 0x0, 0x0, 0x1, 0x0, 0x0};


int ts = 50;                //CPG sample time interval
int T = 1000;               //CPG sin function period
int Nm = T/ts;              //CPG sampling number
double inc = 2*180/Nm;      //present phase increasing in task 50hz


/*
    define all state corresponding action function below
*/
void state0_entry(void) 
{
    cpgPhase = 0;
    cpgPhase |= MASK_STATE(STATE0);                 //state -> cpgPhase
}

void state0_do(void)
{
    static uint8_t cnt=0;
    cnt++;
    if(cnt == 10)                                   //reduce the frequency of sending data
    {
        cnt=0;
        modbusFrame[4] = (cpgPhase >> 8) & 0xFF;    
        modbusFrame[5]=cpgPhase;
        generateCRC(modbusFrame);
        hal.serial(1)->write(modbusFrame, sizeof(modbusFrame));
    }
}


void state0_entry(void);
void state0_do(void);
void state0_exit(void);
void state1_entry(void);
void state1_2_3_4_do(void);
void state1_exit(void);
void state2_entry(void);
void state2_do(void);
void state2_exit(void);
void state3_entry(void);
void state3_do(void);
void state3_exit(void);
void state4_entry(void);
void state4_do(void);
void state4_exit(void);


void state0_exit(void)      
{
    cpgPhase &= ~MASK_STATE(STATE0);                //clear the state bit of cpgPhase
}

void state1_entry(void)
{
    cpgPhase = 0;
    cpgPhase |= MASK_STATE(STATE1);

    modbusFrame[4] = (cpgPhase >> 8) & 0xFF;                    //cpgPhase high 8 bit -> modbusFrame[4]
    modbusFrame[5] = (cpgPhase & 0xFF);                         //cpgPhase low 8 bit -> modbusFrame[5]
    generateCRC(modbusFrame);                                   //generate CRC for array modbusFrame

    hal.serial(1)->write(modbusFrame, sizeof(modbusFrame));     //send modbusFrame via serial port
    hal.scheduler->delay(150);                                  //delay 150ms 
}

void state1_2_3_4_do(void)
{
    cpgPhase+=inc;                                              //cpgPhase increase inc
    if(MASK_PHASE(cpgPhase) >= 360)                             //judge whether the value of cpgPhase exceeds 360
    {
        cpgPhase -= 360;                                        //minus 360 
    }
    modbusFrame[4] = (cpgPhase >> 8) & 0xFF;                    //cpgPhase high 8 bit -> modbusFrame[4]
    modbusFrame[5] = (cpgPhase & 0xFF);                         //cpgPhase low 8 bit -> modbusFrame[5]
    generateCRC(modbusFrame);                                   //generate CRC for array modbusFrame

    hal.serial(1)->write(modbusFrame, sizeof(modbusFrame));     //send modbusFrame via serial port     
}


void state1_exit(void)
{
    cpgPhase &= ~MASK_STATE(STATE1);
}

void state2_entry(void)
{
    cpgPhase |= MASK_STATE(STATE2);
}

void state2_do(void)
{
    cpgPhase+=inc;
    if(MASK_PHASE(cpgPhase) >= 360)
    {
        cpgPhase -= 360;
    }
    modbusFrame[4] = (cpgPhase >> 8) & 0xFF;
    modbusFrame[5] = (cpgPhase & 0xFF);
    generateCRC(modbusFrame);

    hal.serial(1)->write(modbusFrame, sizeof(modbusFrame));

}

void state2_exit(void)
{
    cpgPhase &= ~MASK_STATE(STATE2);
}

void state3_entry(void)
{
    cpgPhase |= MASK_STATE(STATE3);
}

void state3_do(void)
{
    cpgPhase+=inc;
    if(MASK_PHASE(cpgPhase) >= 360)
    {
        cpgPhase -= 360;
    }
    modbusFrame[4] = (cpgPhase >> 8) & 0xFF;
    modbusFrame[5] = (cpgPhase & 0xFF);
    generateCRC(modbusFrame);

    hal.serial(1)->write(modbusFrame, sizeof(modbusFrame));
}


void state3_exit(void)
{
	cpgPhase &= ~MASK_STATE(STATE3);
}

void state4_entry(void)
{
    cpgPhase |= MASK_STATE(STATE4);
}

void state4_do(void)
{
    cpgPhase+=inc;                              //cpgPhase increase inc
    if(MASK_PHASE(cpgPhase) >= 360)             //judge whether the value of cpgPhase exceeds 360
    {
        cpgPhase -= 360;                        //minus 360
    }
    modbusFrame[4] = (cpgPhase >> 8) & 0xFF;    //
    modbusFrame[5] = (cpgPhase & 0xFF);
    generateCRC(modbusFrame);

    hal.serial(1)->write(modbusFrame, sizeof(modbusFrame));
}

void state4_exit(void)
{
    cpgPhase &= ~MASK_STATE(STATE4);
}



/*
    FSM initial function
    */
void fsm_init(FSM_t* pFsm,EVENT_MAP_t* pEventMap,ACTION_MAP_t *pActionMap)
{
	pFsm->stCurState = STATE0;
	pFsm->stNextState = STATE_END;
	pFsm->pEventMap = pEventMap;
	pFsm->pActionMap = pActionMap;
}

/*
    FSM state transfer function
    */
void fsm_state_transfer(FSM_t* pFsm, EVENT_t stEventID)
{
	uint8_t i = 0;
	
	for(i=0; pFsm->pEventMap[i].stEventID<EVENT_MAP_END; i++)
	{
		if((stEventID == pFsm->pEventMap[i].stEventID) 
			&& (pFsm->stCurState == pFsm->pEventMap[i].stCurState))
		{
			pFsm->stNextState = pFsm->pEventMap[i].stNextState;
			
			return;
		}
	}	
}

/*
    FSM actually action function
    */
void action_perfrom(FSM_t* pFsm)
{
	if(STATE_END != pFsm->stNextState)
	{
		pFsm->pActionMap[pFsm->stCurState].ExitAct();
		pFsm->pActionMap[pFsm->stNextState].EnterAct();
		
		pFsm->stCurState = pFsm->stNextState;
		pFsm->stNextState = STATE_END;
	}
	else
	{
		pFsm->pActionMap[pFsm->stCurState].RunningAct();
	}
}



FSM_t stFsmWeather;	//定义状态机
/*
    define myself task.
    channel_throttle : meandering channel
    channel_lateral : turn channel
    */
void Rover::my_serial1(void)
{
    static char flag = 0;
    if(flag == 0)
    {
        fsm_init(&stFsmWeather,eventMap,actionMap);	//注册状态机
        flag = 1;
    }
    
    action_perfrom(&stFsmWeather);

    switch(stFsmWeather.stCurState)
    {
        case STATE0:
            if(channel_throttle->get_radio_in() < 1300 && channel_throttle->get_radio_in() > 1000)
            {
                fsm_state_transfer(&stFsmWeather, EVENT0);
            }
            break;
        
        case STATE1:
            if(channel_throttle->get_radio_in() > 1300)
            {
                fsm_state_transfer(&stFsmWeather, EVENT1);
            }
            if(channel_lateral->get_radio_in() > 1700)
            {
                fsm_state_transfer(&stFsmWeather, EVENT3);
            }
            if(channel_lateral->get_radio_in() < 1300 && channel_lateral->get_radio_in() > 1000)
            {
                fsm_state_transfer(&stFsmWeather, EVENT2);
            }
            break;

        case STATE2:
            if(channel_throttle->get_radio_in() > 1300)
            {
                fsm_state_transfer(&stFsmWeather, EVENT1);
            }
            if(channel_lateral->get_radio_in() > 1300)
            {
                fsm_state_transfer(&stFsmWeather, EVENT4);
            }
            break;

        case STATE3:
            if(channel_throttle->get_radio_in() > 1300)
            {
                fsm_state_transfer(&stFsmWeather,EVENT1);
            }
            if(channel_lateral->get_radio_in() < 1700)
            {
                fsm_state_transfer(&stFsmWeather,EVENT5);
            }
            break;

        case STATE4:
            if(channel_throttle->get_radio_in() > 1300)
            {
                fsm_state_transfer(&stFsmWeather,EVENT1);
            }
            if(channel_lateral->get_radio_in() > 1700)
            {
                fsm_state_transfer(&stFsmWeather, EVENT3);
            }
            if(channel_lateral->get_radio_in() < 1300 && channel_lateral->get_radio_in() > 1000)
            {
                fsm_state_transfer(&stFsmWeather, EVENT2);
            }
            break;

        case STATE_END:
            break;
    }
}



