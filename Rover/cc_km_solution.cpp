#include "cc_km_solution.h"

const uint16_t ResetLine = 0b1110111111111110;
const uint16_t ResetSLine = 0b1110111111111111;

KM_Solution km_solution;

#if defined(CUBE_ORANGE_MINI_SET_ENABLE) + defined(KAKUTE_H7_MINI_NAND_ENABLE) != 1
    #error "you can't enable two macro at the same time !!!!!!!"
    #error "Please select one of two macro to enable !!!!!"
#endif 

/*
    define kmModelSolution task.
    Use steady-state solution of Kuramoto(KM) Model 
    
    */
void Rover::kmModelSolution(void)
{

#if defined(CUBE_ORANGE_MINI_SET_ENABLE)

    km_solution.detectionKeyRocker();
    km_solution.runStep();

#elif defined(KAKUTE_H7_MINI_NAND_ENABLE)
    // hal.serial(3)->printf("origin phase:%d\n", km_solution.valueRockerKey.synPhase);
    // hal.serial(3)->printf("synchronous phase:%d\n", km_solution.phase);
    // hal.serial(3)->printf("ch0_val:%d\n", hal.rcin->read(0));
    // hal.serial(3)->printf("ch1_val:%d\n", hal.rcin->read(1));
    // hal.serial(3)->printf("ch2_val:%d\n", hal.rcin->read(2));
    // hal.serial(3)->printf("ch3_val:%d\n", hal.rcin->read(3));
    // km_solution.detectionKeyRocker();
    // hal.serial(3)->printf("key[A]:%d\n", km_solution.valueRockerKey.key[0]);
    // hal.serial(3)->printf("ch4_val:%d\n", hal.rcin->read(4));
    // hal.serial(3)->printf("ch5_val:%d\n", hal.rcin->read(5));
    // hal.serial(3)->printf("ch6_val:%d\n", hal.rcin->read(6));
    // hal.serial(3)->printf("ch7_val:%d\n", hal.rcin->read(7));
    // hal.serial(3)->printf("ch8_val:%d\n", hal.rcin->read(8));

    km_solution.detectionKeyRocker();
    km_solution.runStep();
#endif
}   

/*
    construct function - init public varies
    */
KM_Solution::KM_Solution(void)
{
    valueRockerKey.resetFlag = true;
    valueRockerKey.reversalFlag = false;
    valueRockerKey.synPhase = 0;
    phase = 0;
    valueKey[0] = valueKey[1] = valueKey[2] = 0;
    modbusFrame[0] = modbusFrame[2] = modbusFrame[3] = modbusFrame[4] = modbusFrame[5] = modbusFrame[6] = modbusFrame[7] = 0;
    modbusFrame[1] = 0x06;
    delta_t = 0.04;
    sendFlag = false;
}

/*
   send the data of synchronous phase, mode and turn to slave, the frequency of run determine by 1/delta_t
    */
void KM_Solution::runStep(void)
{
    byteModbus = 0;
    if(valueRockerKey.resetFlag)
    {
    }
    else if(valueRockerKey.reversalFlag) //phase relationship when reversal 
    {
        valueRockerKey.synPhase -= (double)(2*180*valueRockerKey.frequency*delta_t);
    }
    else
    {
        valueRockerKey.synPhase += (double)(2*180*valueRockerKey.frequency*delta_t);
    }


    if(valueRockerKey.synPhase > 360)
    {
        valueRockerKey.synPhase -= 360;
    }
    else if(valueRockerKey.synPhase < 0)
    {
        valueRockerKey.synPhase += 360;
    }

    if(valueRockerKey.reversalFlag)     //phase relationship when reversal 
    {
        phase = 3*180 - valueRockerKey.synPhase;
    }
    else
    {
        phase = valueRockerKey.synPhase;
    }

    if(phase > 360)
    {
        phase = phase % 360;
    }


    if(valueRockerKey.key[RC_CHANEL_KEY_B-RC_CHANEL_KEY_A] == 1)        //detect KeyB

    {
        byteModbus = ResetLine;
        valueRockerKey.synPhase = 0;
        valueRockerKey.resetFlag = true;
        valueRockerKey.key[RC_CHANEL_KEY_B-RC_CHANEL_KEY_A] = 0;

        modbusFrame[4] = (byteModbus >> 8) & 0xFF;
        modbusFrame[5] = (byteModbus & 0xFF);
        generateCRC(modbusFrame);
#if defined(CUBE_ORANGE_MINI_SET_ENABLE)
        hal.serial(1)->write(modbusFrame, sizeof(modbusFrame));
#elif defined(KAKUTE_H7_MINI_NAND_ENABLE)
        hal.serial(3)->write(modbusFrame, sizeof(modbusFrame));
#endif
    }
    else if(valueRockerKey.key[RC_CHANEL_KEY_C-RC_CHANEL_KEY_A] == 1)   //detect KeyC
    {
        byteModbus = ResetSLine;
        valueRockerKey.synPhase = 0;
        valueRockerKey.resetFlag = true;
        valueRockerKey.key[RC_CHANEL_KEY_C-RC_CHANEL_KEY_A] = 0;

        modbusFrame[4] = (byteModbus >> 8) & 0xFF;
        modbusFrame[5] = (byteModbus & 0xFF);
        generateCRC(modbusFrame);
#if defined(CUBE_ORANGE_MINI_SET_ENABLE)
        hal.serial(1)->write(modbusFrame, sizeof(modbusFrame));
#elif defined(KAKUTE_H7_MINI_NAND_ENABLE)
        hal.serial(3)->write(modbusFrame, sizeof(modbusFrame));
#endif
    }
    else
    {
        if(!valueRockerKey.resetFlag)
        {
            
            byteModbus |= valueRockerKey.phaseOffset << 12;
            byteModbus |= valueRockerKey.key[RC_CHANEL_KEY_A-RC_CHANEL_KEY_A] << 11;
            byteModbus |= valueRockerKey.amplitude << 9;
            byteModbus |= phase;

            modbusFrame[4] = (byteModbus >> 8) & 0xFF;
            modbusFrame[5] = (byteModbus & 0xFF);
            generateCRC(modbusFrame);
            if((valueRockerKey.frequency < 0.1) && (valueRockerKey.phaseOffset == 0b0111) && (!sendFlag))  //stop uart transmit when f=0
            {

            }
            else
            {
#if defined(CUBE_ORANGE_MINI_SET_ENABLE)
                hal.serial(1)->write(modbusFrame, sizeof(modbusFrame));
#elif defined(KAKUTE_H7_MINI_NAND_ENABLE)
                hal.serial(3)->write(modbusFrame, sizeof(modbusFrame));
#endif
                if(sendFlag)
                {
                    sendFlag = false;
                }
            }
        }
    }
}

/*
    detect all value of rocker and key
    */
void KM_Solution::detectionKeyRocker(void)
{
#if defined(CUBE_ORANGE_MINI_SET_ENABLE)
    detectionKey(RC_CHANEL_KEY_B, valueRockerKey.key);
    detectionKey(RC_CHANEL_KEY_C, valueRockerKey.key);
#elif defined(KAKUTE_H7_MINI_NAND_ENABLE)
    detectionKey(RC_CHANEL_KEY_B_C, valueRockerKey.key);
#endif
    detectionRocker();
}

/*
    detect all value of key
    ch: the channel of radio
    buff: the pointer of buffer
    */
void KM_Solution::detectionKey(uint8_t ch, uint8_t *buff)
{
#if defined(CUBE_ORANGE_MINI_SET_ENABLE)
    if(valueKey[ch-RC_CHANEL_KEY_A] != hal.rcin->read(ch))
    {
        buff[ch-RC_CHANEL_KEY_A] = 1;
    }
    valueKey[ch-RC_CHANEL_KEY_A] = hal.rcin->read(ch);
#elif defined(KAKUTE_H7_MINI_NAND_ENABLE)
    static uint16_t tem_value = 0;
    static uint8_t  temCnt = 0;

    tem_value = hal.rcin->read(ch);
    if(tem_value < 1200)
    {
        if(temCnt == 0)
        {
            buff[1] = 1;
        }
        temCnt = 1;
    }else if(tem_value < 1800)
    {
        temCnt = 0;
    }else
    {
        if(temCnt == 0)
        {
            buff[2] = 1;
        }
        temCnt = 2;
    }
#endif
}

/*
    read frequency/amplitude/offset from remote control
    */
void KM_Solution::detectionRocker(void)
{
#if defined(CUBE_ORANGE_MINI_SET_ENABLE)
    uint16_t temAmplitude = hal.rcin->read(RC_CHANEL_ROCKER_LEFTUP);
    static uint8_t temAmVal = 0;
    if(temAmplitude <= AMPLITITUDE_THRESHOLD_0_4)
    {
        temAmVal = 0b00;
    }else if(temAmplitude <= AMPLITITUDE_THRESHOLD_1_4)
    {
        temAmVal = 0b01;
    }else if(temAmplitude <= AMPLITITUDE_THRESHOLD_2_4)
    {
        temAmVal = 0b10;
    }else
    {
        temAmVal = 0b11;
    }

    if(valueRockerKey.amplitude != temAmVal)
    {
        if(!sendFlag)
        {
            sendFlag = true;
        }
    }

    valueRockerKey.amplitude = temAmVal;

#elif defined(KAKUTE_H7_MINI_NAND_ENABLE)
    static uint8_t temAmVal = 0;
    uint16_t temAmplitude = hal.rcin->read(RC_CHANEL_ROCKER_AMPLITITUDE);
    if(temAmplitude <= AMPLITITUDE_THRESHOLD_0_4)
    {
        temAmVal = 0b00;
    }else if(temAmplitude <= AMPLITITUDE_THRESHOLD_2_4)
    {
        temAmVal = 0b01;
    }else
    {
        temAmVal = 0b10;
    }

    if(valueRockerKey.amplitude != temAmVal)
    {
        if(!sendFlag)
        {
            sendFlag = true;
        }
    }

#endif

    //get phase offset from remote control
    int16_t temOffset = (int16_t)hal.rcin->read(RC_CHANEL_ROCKER_LEFT_LATERAL) - 1500;
    //phase offset first order filter
    firstOrderParameter.phaseOffset_input = PHASE_OFFSET_FIRST_ORDER_FILTER_GAIN * temOffset + 
                  (1 - PHASE_OFFSET_FIRST_ORDER_FILTER_GAIN) * firstOrderParameter.phaseOffset_input;
    if( firstOrderParameter.phaseOffset_input < -400 )
    {
        valueRockerKey.phaseOffset = 0b0111;
    }else if( firstOrderParameter.phaseOffset_input <= -350 )
    {
        valueRockerKey.phaseOffset = 0b1110;
    }else if( firstOrderParameter.phaseOffset_input <= -300 )
    {
        valueRockerKey.phaseOffset = 0b1101;
    }
    else if( firstOrderParameter.phaseOffset_input <= -250 )
    {
        valueRockerKey.phaseOffset = 0b1100;
    }else if( firstOrderParameter.phaseOffset_input <= -200 )
    {
        valueRockerKey.phaseOffset = 0b1011;
    }
    else if( firstOrderParameter.phaseOffset_input <= -150 )
    {
        valueRockerKey.phaseOffset = 0b1010;
    }else if( firstOrderParameter.phaseOffset_input <= -100 )
    {
        valueRockerKey.phaseOffset = 0b1001;
    }
    else if( firstOrderParameter.phaseOffset_input <= -50 )
    {
        valueRockerKey.phaseOffset = 0b1000;
    }else if( firstOrderParameter.phaseOffset_input <= 50 )
    {
        valueRockerKey.phaseOffset = 0b0111;
    }else if( firstOrderParameter.phaseOffset_input <= 100 )
    {
        valueRockerKey.phaseOffset = 0b0110;
    }
    else if( firstOrderParameter.phaseOffset_input <= 150 )
    {
        valueRockerKey.phaseOffset = 0b0101;
    }else if( firstOrderParameter.phaseOffset_input <= 200 )
    {
        valueRockerKey.phaseOffset = 0b0100;
    }
    else if( firstOrderParameter.phaseOffset_input <= 250 )
    {
        valueRockerKey.phaseOffset = 0b0011;
    }else if( firstOrderParameter.phaseOffset_input <= 300 )
    {
        valueRockerKey.phaseOffset = 0b0010;
    }
    else if( firstOrderParameter.phaseOffset_input <= 350 )
    {
        valueRockerKey.phaseOffset = 0b0001;
    }else
    {
        valueRockerKey.phaseOffset = 0b0000;
    }




    // if(temOffset == 0)
    // {
    //     valueRockerKey.phaseOffset = 0b0111;
    // }else if( temOffset <=1150 )
    // {
    //     valueRockerKey.phaseOffset = 0b1110;
    // }else if( temOffset <=1200 )
    // {
    //     valueRockerKey.phaseOffset = 0b1101;
    // }
    // else if( temOffset <=1250 )
    // {
    //     valueRockerKey.phaseOffset = 0b1100;
    // }else if( temOffset <=1300 )
    // {
    //     valueRockerKey.phaseOffset = 0b1011;
    // }
    // else if( temOffset <=1350 )
    // {
    //     valueRockerKey.phaseOffset = 0b1010;
    // }else if( temOffset <=1400 )
    // {
    //     valueRockerKey.phaseOffset = 0b1001;
    // }
    // else if( temOffset <=1450 )
    // {
    //     valueRockerKey.phaseOffset = 0b1000;
    // }else if( temOffset <=1550 )
    // {
    //     valueRockerKey.phaseOffset = 0b0111;
    // }else if( temOffset <=1600 )
    // {
    //     valueRockerKey.phaseOffset = 0b0110;
    // }
    // else if( temOffset <=1650 )
    // {
    //     valueRockerKey.phaseOffset = 0b0101;
    // }else if( temOffset <=1700 )
    // {
    //     valueRockerKey.phaseOffset = 0b0100;
    // }
    // else if( temOffset <=1750 )
    // {
    //     valueRockerKey.phaseOffset = 0b0011;
    // }else if( temOffset <=1800 )
    // {
    //     valueRockerKey.phaseOffset = 0b0010;
    // }
    // else if( temOffset <=1850 )
    // {
    //     valueRockerKey.phaseOffset = 0b0001;
    // }else
    // {
    //     valueRockerKey.phaseOffset = 0b0000;
    // }

    //get forward/backward frequency
    uint16_t temForwardFrequency = hal.rcin->read(RC_CHANEL_ROCKER_LEFT_LONGITUDINAL);
    uint16_t temBackwardFrequency = hal.rcin->read(RC_CHANEL_ROCKER_RIGHT_LONGITUDINAL);
    if((temForwardFrequency > 1100) && (temForwardFrequency < 1450))
    {
        firstOrderParameter.frequency_input = (double)((1500 - temForwardFrequency) *(double)MAX_FREQUENCY_FORWARD) / (double)400; 
        if(valueRockerKey.resetFlag)
        {
            valueRockerKey.resetFlag = false;
        }
        if(valueRockerKey.reversalFlag)
        {
            valueRockerKey.reversalFlag = false;
        }
    }else if(temBackwardFrequency > 1650)
    {
        firstOrderParameter.frequency_input = (double)((temBackwardFrequency - 1500) *(double)MAX_FREQUENCY_BACKWARD) / (double)400; 
        valueRockerKey.phaseOffset = 0b1111;
        if(!valueRockerKey.reversalFlag)
        {
            valueRockerKey.reversalFlag = true;
        }
    }else
    {
        firstOrderParameter.frequency_input = 0; 
        if(valueRockerKey.reversalFlag)
        {
            valueRockerKey.reversalFlag = false;
        }
    }
    //first order filter
    valueRockerKey.frequency = FREQUENCY_FIRST_ORDER_FILTER_GAIN * firstOrderParameter.frequency_input + 
                              (1 - FREQUENCY_FIRST_ORDER_FILTER_GAIN) * valueRockerKey.frequency;


    uint16_t temKeyA = hal.rcin->read(RC_CHANEL_KEY_A);
    static uint8_t temKeyVal;

    if(temKeyA <= 1100)
    {
        temKeyVal = 0;
    }else if(temKeyA >= 1900)
    {
        temKeyVal = 1;
    }

    if(valueRockerKey.key[RC_CHANEL_KEY_A-RC_CHANEL_KEY_A] != temKeyVal)
    {
        if(!sendFlag)
        {
            sendFlag = true;
        }
    }

    valueRockerKey.key[RC_CHANEL_KEY_A-RC_CHANEL_KEY_A] = temKeyVal;

}