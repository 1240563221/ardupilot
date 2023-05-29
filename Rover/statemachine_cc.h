#ifndef __STATEMACHINE_H
#define __STATEMACHINE_H

#include "Rover.h"
#include "AP_Gripper/AP_Gripper.h"

//my code
#define MASK_STATE(x)   ((uint16_t)x<<12)
#define MASK_PHASE(x)   ((uint16_t)x&0xFFF)


extern const AP_HAL::HAL& hal;

/*  enum STATE_t
    STATE0:represent initial state
    STATE1:represent meandering state(from state0)
    STATE2:represent left turn state
    STATE3:represent right turn state  
    STATE4:represent meandering state(from state2 and state3)
    STATE_END:represent the end state 
*/
typedef enum{
	STATE0 = 0,
	STATE1,
	STATE2,
	STATE3,
    STATE4,
    STATE_END,     
}STATE_t; 

/* 
    define an alias for a function pointer
*/
typedef void (*STATE_ACTION)(void);	


/*
    define a structure that represents state and corresponding action function
*/
typedef struct ACTION_MAP
{
	STATE_t 		stStateID;
	STATE_ACTION 	EnterAct;	
	STATE_ACTION 	RunningAct;	
	STATE_ACTION 	ExitAct;
}ACTION_MAP_t;


/*
    define the type and number of evvents.

    V_throttle = channel_throttle->get_radio_in()
    V_steering = channel_lateral->get_radio_in()

    EVENT0: radio channel 1000 < V_throttle < 1300
    EVENT1: radio channel        V_throttle > 1300
    EVENT2: radio channel 1000 < V_steering < 1300
    EVENT3: radio channel        V_steering > 1700
    EVENT4: radio channel        V_steering > 1300
    EVENT5: radio channel        V_steering < 1700
    
*/
typedef enum
{
    EVENT0 = 0,
	EVENT1,
	EVENT2,
	EVENT3,
	EVENT4,
	EVENT5,
	EVENT_MAP_END
}EVENT_t;


/*
    define alias for struct:
    represent switch to the next state when an event occurs in a particular state
*/
typedef struct EVENT_MAP
{
	EVENT_t	stEventID;
	STATE_t stCurState;
	STATE_t stNextState;
}EVENT_MAP_t;

/*
    define alias for struct.
    finite state machine(FSM).
    define FSM's current state and next state,
    and ACTION_MAP_t's instantiations and EVENT_MAP_t instantiation's pointer
    */
typedef struct FSM
{
	STATE_t stCurState;
	STATE_t stNextState;
	ACTION_MAP_t *pActionMap;
	EVENT_MAP_t *pEventMap;
}FSM_t;




//my code -- function declaration 

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
uint16_t usMBCRC16( uint8_t * pucFrame, uint16_t usLen );
void generateCRC(uint8_t *buffer);

void fsm_init(FSM_t* pFsm,EVENT_MAP_t* pEventMap,ACTION_MAP_t *pActionMap);
void fsm_state_transfer(FSM_t* pFsm, EVENT_t stEventID);
void action_perfrom(FSM_t* pFsm);

#endif