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


class KM_Solution
{
    public:
    friend class Rover;
    struct valueRockerKey
    {
        double frequency;           //control frequency
        uint8_t phaseOffset;       
        uint16_t amplitude;
        int16_t synPhase;          //synchronous phase 
        uint8_t key[3];             //KeyA control mode; KeyB line; KeyC bend 
        bool resetFlag;             //indicate the flag of keyB and KeyC, lock synchronise phase,waiting for frequency unlock
        bool reversalFlag;          //the flag of back car
    }valueRockerKey;

    uint8_t modbusFrame[8];         //modbus protocol frame
    uint16_t phase;                 //temp synchronous phase
    uint16_t byteModbus;            //modbus byte
    uint16_t valueKey[3];           //temp key value buffer
    double  delta_t;                //s

    KM_Solution(void);
    void runStep(void);
    void detectionKeyRocker(void);
    void detectionKey(uint8_t ch, uint8_t *buff);
    void detectionRocker(void);
    private:
};

#endif