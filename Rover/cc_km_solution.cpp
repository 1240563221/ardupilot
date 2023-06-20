#include "cc_km_solution.h"


valueRockerKey_TPDF valueRockerKey;

uint16_t valueKey[3];
uint16_t byteModbus = 0;
static uint8_t modbusFrame[8]={0x0, 0x06, 0x0, 0x0, 0x0, 0x1, 0x0, 0x0};
uint16_t phase = 0;
double  delta_t=0.040;          //s

const uint16_t ResetLine = 0b1110111111111110;
const uint16_t ResetSLine = 0b1110111111111111;



/*
    define kmModelSolution task.
    Use steady-state solution of Kuramoto(KM) Model 
    
    */
void Rover::kmModelSolution(void)
{
    // double t_last=0, t_now=0;
    byteModbus = 0;
    detectionKeyRocker();
    phase += 2*180*valueRockerKey.frequency*delta_t;
    if(phase > 360)
    {
        phase -= 360;
    }


    if(valueRockerKey.key[RC_CHANEL_KEY_B-RC_CHANEL_KEY_A] == 1)
    {
        byteModbus = ResetLine;
        phase = 0;
        valueRockerKey.resetFlag = true;
        valueRockerKey.key[RC_CHANEL_KEY_B-RC_CHANEL_KEY_A] = 0;

        modbusFrame[4] = (byteModbus >> 8) & 0xFF;
        modbusFrame[5] = (byteModbus & 0xFF);
        generateCRC(modbusFrame);
        hal.serial(1)->write(modbusFrame, sizeof(modbusFrame));
    }
    else if(valueRockerKey.key[RC_CHANEL_KEY_C-RC_CHANEL_KEY_A] == 1)
    {
        byteModbus = ResetSLine;
        phase = 0;
        valueRockerKey.resetFlag = true;
        valueRockerKey.key[RC_CHANEL_KEY_C-RC_CHANEL_KEY_A] = 0;

        modbusFrame[4] = (byteModbus >> 8) & 0xFF;
        modbusFrame[5] = (byteModbus & 0xFF);
        generateCRC(modbusFrame);
        hal.serial(1)->write(modbusFrame, sizeof(modbusFrame));
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
            hal.serial(1)->write(modbusFrame, sizeof(modbusFrame));
        }
    }
}

/*
    detect all value of rocker and key
    */
void detectionKeyRocker(void)
{
    detectionKey(RC_CHANEL_KEY_B, valueRockerKey.key);
    detectionKey(RC_CHANEL_KEY_C, valueRockerKey.key);
    detectionRocker();
}

/*
    detect all value of key
    ch: the channel of radio
    buff: the pointer of buffer
    */
void detectionKey(uint8_t ch, uint8_t *buff)
{
    if(valueKey[ch-RC_CHANEL_KEY_A] != hal.rcin->read(ch))
    {
        buff[ch-RC_CHANEL_KEY_A] = 1;
    }
    valueKey[ch-RC_CHANEL_KEY_A] = hal.rcin->read(ch);
}

/*
    read frequency/amplitude/offset
    */
void detectionRocker(void)
{

    uint16_t temAmplitude = hal.rcin->read(RC_CHANEL_ROCKER_LEFTUP);
    if(temAmplitude <= 1300)
    {
        valueRockerKey.amplitude = 0b00;
    }else if(temAmplitude <= 1500)
    {
        valueRockerKey.amplitude = 0b01;
    }else if(temAmplitude <= 1700)
    {
        valueRockerKey.amplitude = 0b10;
    }else
    {
        valueRockerKey.amplitude = 0b11;
    }

    uint16_t temOffset = hal.rcin->read(RC_CHANEL_ROCKER_LEFT_LATERAL);
    if(temOffset == 0)
    {
        valueRockerKey.phaseOffset = 0b0111;
    }else if( temOffset <=1150 )
    {
        valueRockerKey.phaseOffset = 0b0000;
    }else if( temOffset <=1200 )
    {
        valueRockerKey.phaseOffset = 0b0001;
    }
    else if( temOffset <=1250 )
    {
        valueRockerKey.phaseOffset = 0b0010;
    }else if( temOffset <=1300 )
    {
        valueRockerKey.phaseOffset = 0b0011;
    }
    else if( temOffset <=1350 )
    {
        valueRockerKey.phaseOffset = 0b0100;
    }else if( temOffset <=1400 )
    {
        valueRockerKey.phaseOffset = 0b0101;
    }
    else if( temOffset <=1450 )
    {
        valueRockerKey.phaseOffset = 0b0110;
    }else if( temOffset <=1550 )
    {
        valueRockerKey.phaseOffset = 0b0111;
    }else if( temOffset <=1600 )
    {
        valueRockerKey.phaseOffset = 0b1000;
    }
    else if( temOffset <=1650 )
    {
        valueRockerKey.phaseOffset = 0b1001;
    }else if( temOffset <=1700 )
    {
        valueRockerKey.phaseOffset = 0b1010;
    }
    else if( temOffset <=1750 )
    {
        valueRockerKey.phaseOffset = 0b1011;
    }else if( temOffset <=1800 )
    {
        valueRockerKey.phaseOffset = 0b1100;
    }
    else if( temOffset <=1850 )
    {
        valueRockerKey.phaseOffset = 0b1101;
    }else
    {
        valueRockerKey.phaseOffset = 0b1110;
    }

    uint16_t temFrequency = hal.rcin->read(RC_CHANEL_ROCKER_LEFT_LONGITUDINAL);
    if((temFrequency > 1100) && (temFrequency < 1450))
    {
        valueRockerKey.frequency = (double)((1500 - temFrequency) *(double)1.8) / (double)400; 
        if(valueRockerKey.resetFlag)
        {
            valueRockerKey.resetFlag = false;
        }
    }else if(hal.rcin->read(RC_CHANEL_ROCKER_RIGHT_LONGITUDINAL) > 1750)
    {
        valueRockerKey.frequency = 1; 
        valueRockerKey.phaseOffset = 0b1111;
    }else
    {
        valueRockerKey.frequency = 0; 
    }

    uint16_t temKeyA = hal.rcin->read(RC_CHANEL_KEY_A);
    if(temKeyA == 1000)
    {
        valueRockerKey.key[RC_CHANEL_KEY_A-RC_CHANEL_KEY_A] = 0;
    }else if(temKeyA == 1900)
    {
        valueRockerKey.key[RC_CHANEL_KEY_A-RC_CHANEL_KEY_A] = 1;
    }

}