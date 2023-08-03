#ifndef __CC_KM_SOLUTION_H__
#define __CC_KM_SOLUTION_H__

#include "Rover.h"
#include "AP_Gripper/AP_Gripper.h"
#include "cc_CRC.h"


/*
    the difference of choose fly controller is the video transfer module,
    the Cube means use herelink v1.1 as the video transfer module because it has both radio and video transmission together.
    the Kakute means radio and video transmission are work independently.(like three angle video module and tx18s)
    */
#define FLY_CONTROLLER_CHOOSE                          0       //0--herelink_v1.1   1--radio and video transmission independently

#if  0 == FLY_CONTROLLER_CHOOSE
#define BOTH_HERELINK_V11_MODULE_ENABLE                     //serial(1)
#elif 1 == FLY_CONTROLLER_CHOOSE
#define SEPERATE_ELSR_915M_ENAB                      //serial(3)
#endif


/*
    the difference between two macro is radio's channel correspond to high level commands(like forward, backward, turn, etc)
    */
#if defined(BOTH_HERELINK_V11_MODULE_ENABLE)
    #define RC_CHANEL_ROCKER_LEFT_LONGITUDINAL      2
    #define RC_CHANEL_ROCKER_LEFT_LATERAL           3
    #define RC_CHANEL_ROCKER_RIGHT_LONGITUDINAL     1
    #define RC_CHANEL_ROCKER_RIGHT_LATERAL          0
    #define RC_CHANEL_ROCKER_LEFTUP                 7
    #define RC_CHANEL_KEY_A                         8
    #define RC_CHANEL_KEY_B                         9
    #define RC_CHANEL_KEY_C                         10

    #define AMPLITITUDE_THRESHOLD_0_4               1300
    #define AMPLITITUDE_THRESHOLD_1_4               1500
    #define AMPLITITUDE_THRESHOLD_2_4               1700
    #define AMPLITITUDE_THRESHOLD_3_4               1900
#elif defined(SEPERATE_ELSR_915M_ENAB)
    #define RC_CHANEL_ROCKER_LEFT_LONGITUDINAL      2
    #define RC_CHANEL_ROCKER_LEFT_LATERAL           3
    #define RC_CHANEL_ROCKER_RIGHT_LONGITUDINAL     1
    #define RC_CHANEL_ROCKER_RIGHT_LATERAL          0
    #define RC_CHANEL_ROCKER_AMPLITITUDE            7
    #define RC_CHANEL_KEY_A                         4
    #define RC_CHANEL_KEY_B_C                       5
    #define RC_CHANEL_KEY_B                         5
    #define RC_CHANEL_KEY_C                         6

    #define AMPLITITUDE_THRESHOLD_0_4               1300
    #define AMPLITITUDE_THRESHOLD_1_4               1500
    #define AMPLITITUDE_THRESHOLD_2_4               1700
    #define AMPLITITUDE_THRESHOLD_3_4               1900

#endif

#define MAX_FREQUENCY_FORWARD                           1.3f
#define MAX_FREQUENCY_BACKWARD                          0.8f

#define FREQUENCY_FIRST_ORDER_FILTER_GAIN               0.07f
#define PHASE_OFFSET_FIRST_ORDER_FILTER_GAIN            0.02f

class KM_Solution
{
    public:
        friend class Rover;             //friend class -- the class of rover could access KM_Solution
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

        struct 
        {
            double frequency_input;      //contribute to first order filter -> frequency
            int16_t phaseOffset_input;   //contribute to first order filter -> phase offset
        }firstOrderParameter;            //the struct of first order filter

        uint8_t modbusFrame[8];         //modbus protocol frame
        uint16_t phase;                 //temp synchronous phase
        uint16_t byteModbus;            //modbus byte
        uint16_t valueKey[3];           //temp key value buffer
        double  delta_t;                //s

        bool sendFlag;                  //determine send flag

        KM_Solution(void);
        void runStep(void);
        void detectionKeyRocker(void);
        void detectionKey(uint8_t ch, uint8_t *buff);
        void detectionRocker(void);

    private:
};


#endif