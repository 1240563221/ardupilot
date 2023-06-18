#ifndef __CC_KM_SOLUTION_H__
#define __CC_KM_SOLUTION_H__

#include "Rover.h"
#include "AP_Gripper/AP_Gripper.h"
#include "cc_CRC.h"

#define RC_CHANEL_ROCKER_LEFT_LONGITUDINAL      2
#define RC_CHANEL_ROCKER_LEFT_LATERAL           3
#define RC_CHANEL_ROCKER_RIGHT_LONGITUDINAL     1
#define RC_CHANEL_ROCKER_RIGHT_LATERAL          0
#define RC_CHANEL_ROCKER_LEFTUP                 7
#define RC_CHANEL_KEY_A                         8
#define RC_CHANEL_KEY_B                         9
#define RC_CHANEL_KEY_C                         10


typedef struct
{
    double frequency;         //control frequency
    uint8_t phaseOffset;       
    uint16_t amplitude;
    uint8_t key[3];              //KeyA control mode; KeyB line; KeyC bend 

}
valueRockerKey_TPDF; 



void detectionKeyRocker(void);
void detectionKey(uint8_t ch, uint8_t *buff);
void detectionRocker(void);


#endif