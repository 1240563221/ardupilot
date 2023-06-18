#include "cc_statemachine.h"

extern const AP_HAL::HAL& hal;


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



