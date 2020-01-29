/*
 * File:   main.c
 * Author: 27783
 *
 * Created on 13 ??????? 2015 ?., 15:09
 */

#include <stdio.h>
#include <stdlib.h>


// PIC16F819 Configuration Bit Settings

// 'C' source line config statements

#include <xc.h>

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// CONFIG
#pragma config FOSC = INTOSCIO  // Oscillator Selection bits (INTRC oscillator; port I/O function on both RA6/OSC2/CLKO pin and RA7/OSC1/CLKI pin)
#pragma config WDTE = OFF        // Watchdog Timer Enable bit (WDT enabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = ON       // RA5/MCLR/VPP Pin Function Select bit (RA5/MCLR/VPP pin function is MCLR)
#pragma config BOREN = ON       // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = OFF        // Low-Voltage Programming Enable bit (RB3/PGM pin has digital I/O function, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EE Memory Code Protection bit (Code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off)
#pragma config CCPMX = RB2      // CCP1 Pin Selection bit (CCP1 function on RB2)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)
/*
 *
 */
#define TMR0_PRELOAD    0x40// 1ms (1KHz)
#define TMR2_PRELOAD    0x4D //10ms (100Hz)

#define ADC_DELTA   10

volatile unsigned int DELAY_C_COUNT;
volatile unsigned int DELAY_COUNT;
volatile unsigned char DELAY_FLAG;
unsigned int ADRES;
signed char ButtonPressed = 0;
volatile unsigned char State = 0;
volatile unsigned char LastState = 0;
unsigned char RearStateLast;
signed char RearState = -1;
volatile unsigned int FrontTimeOut;
volatile unsigned int CM_ModeTimeOut;
volatile unsigned char CM_Mode = 0;//1 - Volume, 0 - Menu navigation
volatile signed int VolTarget;
volatile unsigned char ENC_AB_LAST;

unsigned int CAMERA_BUT, AUDIO_BUT, NO_BUT;

int i;
unsigned int u16;
//                      0  1 2 3  4 5 6  7   8 9 A B  C D  E F
//signed char states[] = {0,-1,1,0, 1,0,0,-1, -1,0,0,1, 0,1,-1,0};//Encoder directions CCW
signed char states[] = {0,1,-1,0, -1,0,0,1, 1,0,0,-1, 0,-1,1,0};//Encoder directions CW

typedef enum
{
    NO_PRESS    = 0,
    AUDIO       = 1,
    NAVI        = 2,
    PHONE       = 3,
    SETUP       = 4,
    BACK        = 5
}T_ADC_BUT;

T_ADC_BUT But1, But2;

void CM_Rotate(void);

signed char COMPARE_ADC(unsigned int adc_val, unsigned int target)
{
    if(adc_val > target + ADC_DELTA) return 1;
    if(adc_val + ADC_DELTA < target) return -1;
    return 0;
}

T_ADC_BUT ADC_But(unsigned int adc_val)
{
        if(adc_val > 460) return NO_PRESS;
        
        if( (adc_val > 70) && (adc_val < 110) ) //[0.341V...0.537V] 0.44V in == 90
            return AUDIO;
        if( (adc_val > 170) && (adc_val < 210) ) //[0.879V...1.123V] 1V in == 205//184
            return NAVI;
        if( (adc_val > 300) && (adc_val < 350) ) //[1.464V...1.708V] 1.6V in == 328
            return PHONE;
        if( (adc_val > 380) && (adc_val < 440) ) //[1.855V...2.197V] 2V in == 410
            return SETUP;
        if( (adc_val > 450) && (adc_val < 470) ) //[2.295V...2.685V] 2.5V in == 512
            return BACK;

        return NO_PRESS;//481
}

void interrupt V1(void)
{
    if(TMR2IF) //100Hz; 10ms
    {
        //PORTA ^= 0x04;
        if(FrontTimeOut)
        {
            FrontTimeOut--;
            if(!FrontTimeOut)
                State = 0;
        }

        if(DELAY_C_COUNT > 0)
            DELAY_C_COUNT--;
        
        if(CM_ModeTimeOut)
        {
            CM_ModeTimeOut--;
            if(!CM_ModeTimeOut)
                CM_Mode = 0;//Menu navigation
        }        
        
        CM_Rotate();
        
        TMR2IF = 0;
    }

    if(TMR0IF) //1000Hz; 1ms
    {
        PORTA ^= 0x04;
        
        if(DELAY_FLAG)
        {
            if(DELAY_COUNT)
                DELAY_COUNT--;
            else
                DELAY_FLAG = 0;
        }

        TMR0 -= TMR0_PRELOAD;
        TMR0IF = 0;
    }
    
    if(ADIF)
    {
        ADIF = 0;
    }

    if(RBIF)
    {
        RBIF = 0;
    }
}

void Delay_ms(unsigned int val)
{
    DELAY_COUNT = val;
    DELAY_FLAG = 1;
    while(DELAY_FLAG) ;
}

void VolUp(void)
{
    RA7 = 1;
    Delay_ms(40);
    RA7 = 0;
    CM_ModeTimeOut = 600;
}

void VolDown(void)
{
    RA6 = 1;
    Delay_ms(40);
    RA6 = 0;
    CM_ModeTimeOut = 600;
}

void CM_Rotate(void)
{
    unsigned char ENC_A,ENC_B,ENC_AB;
//    0 - error, no move
//    1 - CW
//    2 - CCW
    ENC_A = RB5;
    ENC_B = RB4;
    ENC_A += RB5;
    ENC_B += RB4;
    ENC_A += RB5;
    ENC_B += RB4;
    ENC_A += RB5;
    ENC_B += RB4;
    if(ENC_A == 4) ENC_A = 1;
    else
    if(ENC_A == 0) ENC_A = 0;
    else
        ENC_A = ((ENC_AB_LAST>>1) & 0x01);
        
    if(ENC_B == 4) ENC_B = 1;
    else
    if(ENC_B == 0) ENC_B = 0;
    else
        ENC_B = (ENC_AB_LAST & 0x01);
    
    ENC_AB = (ENC_A << 1) | ENC_B;    
    
    if( (ENC_AB_LAST != ENC_AB) )
    {
        if(CM_Mode)//Volume
        {
            VolTarget += states[((ENC_AB_LAST<<2) | ENC_AB)];
        }
        else//Menu navigation
        {
            RB6 = ~ENC_B;
            RB7 = ~ENC_A;
        }
        ENC_AB_LAST = ENC_AB;
    }
}

char Get_ADC_BUT(T_ADC_BUT* but)
{
    GO_nDONE = 1;
    DELAY_COUNT = 50;
    DELAY_FLAG = 1;
    while( GO_nDONE && DELAY_FLAG);//Wait for ADC
    if(!GO_nDONE)
    {
        ADRES = (ADRESH<<8) + ADRESL;
        *but = ADC_But(ADRES);
        return 1;
    }
    //else ADC not ready
   
    return 0;
}

unsigned int Get_ADC(void)
{
    GO_nDONE = 1;
    DELAY_COUNT = 50;
    DELAY_FLAG = 1;
    while( GO_nDONE && DELAY_FLAG);//Wait for ADC
    if(!GO_nDONE)
    {
        ADRES = (ADRESH<<8) + ADRESL;
        return ADRES;
    }
    //else ADC not ready
    return 0;
}

int GetBut()
{
    unsigned int adc_res;
    adc_res = Get_ADC();
    if(COMPARE_ADC(adc_res, NO_BUT) == 0) return 0;
    if(COMPARE_ADC(adc_res, CAMERA_BUT) == 0) return 1;
    if(COMPARE_ADC(adc_res, AUDIO_BUT) == 0) return 2;
    return -1;
}

int main(void) {

//OSCILLATOR
    OSCCON = 0x70;//8MHz

//Ports
/*=========================
 *
 * RA0 - pin17 - Button     (an in)
 * RA2 - pin 1 - Strobe     (out)
 * RA6 - pin15 - VolDown    (out)
 * RA7 - pin16 - VolUp      (out)
 * RB0 - pin 6 - RearGear   (in)
 * RB1 - pin 7 - RearEmul   (out)
 * RB2 - pin 8 - 6V_sig     (in)
 * RB3 - pin 9 - VideoSel   (out)
 * RB4 - pin10 - CM_PH2_IN  (in)
 * RB5 - pin11 - CM_PH1_IN  (in)
 * RB6 - pin12 - CM_PH2_OUT (out)
 * RB7 - pin13 - CM_PH1_OUT (out)
 *
 *=========================
 */

//States
/*=========================
 *
 *  0 - Off
 *  1 - Rear on
 *  2 - Front on
 *
 *=========================
 */
    TRISA = 0xFF; //All inputs
    TRISB = 0xFF; //All inputs

    PORTA = 0x00;
    TRISA &= ~0xC0;//RA6 & RA7 output
    TRISA &= ~0x04;//RA2 output
    PORTA = 0x00;

    PORTB = 0x00;
    TRISB &= ~0xCA;//RB1 & RB3 & RB6 & RB7 output
    PORTB = 0x00;

    ADCON1 = 0x04; //RA0, RA1, RA3 Analog

//Timer0
    T0CS = 0; //Int osc
    PSA = 0; //Prescaller to Timer0
    PS2 = 1;
    PS1 = 0;
    PS0 = 0; //1:32 (CLK=4MHz/4)
    TMR0 = TMR0_PRELOAD;
    TMR0IF = 0;
    TMR0IE = 1;

//Timer2
    TOUTPS3 = 1;
    TOUTPS2 = 1;
    TOUTPS1 = 1;
    TOUTPS0 = 1; //1:2

    T2CKPS1 = 1;
    T2CKPS0 = 1;
    PR2 = TMR2_PRELOAD;
    TMR2IF = 0;
    TMR2IE = 1;
    TMR2ON = 1;

//ADC
    ADCS2 = 1;
    ADCS1 = 0;
    ADCS0 = 1; //Fosc/16

    CHS2 = 0;
    CHS1 = 0;
    CHS0 = 0; //AN0

    ADFM = 1; //Right justifield
    //PCFG<3:0>

    ADIF = 0;
    ADIE = 0;

    ADON = 1;

    CM_Mode = 0;//Menu navigation
    CM_Rotate();

    RBIF = 0;
    RBIE = 0;//1;

    PEIE = 1;
    GIE = 1;

    State = 0;
    RearStateLast = RB0;
    FrontTimeOut = 0;
    VolTarget = 0;

    DELAY_C_COUNT = 50;
    while(DELAY_C_COUNT);//wait 500ms
    
    DELAY_C_COUNT = 500;
    while((DELAY_C_COUNT) && (Get_ADC() < 150)) ; //AUDIO == 90 
    
    if(Get_ADC() < 150)//Programming
    {
        RB1 = 1; RB3 = 1; //front camera on
        Delay_ms(2000);
        u16 = 0;
        for(i=0;i<10;i++) u16 += Get_ADC();
        eeprom_write(0x00, (unsigned char)((u16/10)>>8));
        eeprom_write(0x01, (unsigned char)((u16/10) & 0xFF));//Buttons released
        NO_BUT = u16/10;
        RB1 = 1; RB3 = 0; //back camera on
        while(GetBut() == 0);//wait pressing
        Delay_ms(500);
        u16 = 0;
        for(i=0;i<10;i++) u16 += Get_ADC();
        eeprom_write(0x02, (unsigned char)((u16/10)>>8));
        eeprom_write(0x03, (unsigned char)((u16/10) & 0xFF));//Camera But
        CAMERA_BUT = u16/10;
        RB1 = 1; RB3 = 1; //front camera on
        while(GetBut() != 0);//wait releasing
        Delay_ms(1000);
        RB1 = 1; RB3 = 0; //back camera on
        while(GetBut() == 0);//wait pressing
        Delay_ms(500);
        u16 = 0;
        for(i=0;i<10;i++) u16 += Get_ADC();
        eeprom_write(0x04, (unsigned char)((u16/10)>>8));
        eeprom_write(0x05, (unsigned char)((u16/10) & 0xFF));//Audio on
        AUDIO_BUT = u16/10;
        RB1 = 1; RB3 = 1; //front camera on
        Delay_ms(500);
        RB1 = 1; RB3 = 0; //back camera on
        Delay_ms(500);
        RB1 = 1; RB3 = 1; //front camera on
        Delay_ms(500);
        RB1 = 1; RB3 = 0; //back camera on
        Delay_ms(500);
        RB1 = 1; RB3 = 1; //front camera on
        Delay_ms(1000);
        RB1 = 0; RB3 = 0;//All off
    }
    else//Read saved
    {
        NO_BUT = ((eeprom_read(0x00)<<8) + eeprom_read(0x01));
        CAMERA_BUT = ((eeprom_read(0x02)<<8) + eeprom_read(0x03));
        AUDIO_BUT = ((eeprom_read(0x04)<<8) + eeprom_read(0x05));
    }

        
    
for(;;)
{
    RearState = -1;

//Free == 3.16V (4.85K)
//Navi == 1.0V (150 Ohm)
//Audio == 0.44V (50 Ohm)
//Phone == 1.6V (300 Ohm)
//Setup == 2.0V (480 - 560 Ohm)
//Back == 2.5V (1K)

//Free == 3.16V (4.85K)
//Up == 0.44V (50 Ohm)
//Down == 1.0V (150 Ohm)
//Left == 2.0V (480 - 560 Ohm)
//Right == 1.6V (300 Ohm)
//Press == 2.9V (2.2K)

//Free == 3.0V  (4.85K)
//VolUp == 1.0V (150 Ohm)
//VolDown == 0.48V (50 Ohm)
//<< == 2.1V (560 Ohm)
//>> == 1.6V (300 Ohm
/*    
    ButtonPressed = 0;
    if(Get_ADC_BUT(&But1))
    {
        if(But1 != NO_PRESS)//Button pressed
        {
            if(Get_ADC_BUT(&But2))
            {
                if(But2 == But1)//Button pressed
                {
                    //Button presed
                    
                    if(But2 == NAVI)
                        ButtonPressed = 1;//Navi
                    
                    if(But2 == AUDIO)
                        ButtonPressed = 2;//Audio
                    else
                    {
                        CM_Mode = 0;//Menu navigation
                        CM_ModeTimeOut = 0;
                    }
                }
            }//else ADC not ready
        }//if(But1 != NO_PRESS)
    }
*/
    ButtonPressed = GetBut();
    if(ButtonPressed < 0)
    {
        ButtonPressed = 0;
        CM_Mode = 0;//Menu navigation
        CM_ModeTimeOut = 0;
        VolTarget = 0;
    }

    if(ButtonPressed == 1)//Navi
    {
        FrontTimeOut = 0;
        State++;
        if(State>2)
            State = 0;//off

        //if(RearStateLast)//Rear on
        if(RB0)//Rear on
        {
            if(State == 0)//Off
                State = 1;//Front
        }
    }

    if(ButtonPressed == 2)//Audio
    {
        if(CM_Mode > 0)
        {
            VolTarget = 0;
            CM_Mode = 0;//Menu navigation
            CM_ModeTimeOut = 0;
            
            State = 0;
            FrontTimeOut = 0;
        }
        else
        {
            VolTarget = 0;
            CM_Mode = 1;//Volume mode
            //CM_ModeTimeOut = 6000; implemented in VolUp()/VolDown()
            VolDown();
            Delay_ms(40); //+/- for draw volume bar on display
            VolUp();
        }
    }

    if(RearStateLast != RB0)
    {
        RearStateLast = RB0;
        RearState = RearStateLast;
    }

    if(RearState>=0)
    {
        FrontTimeOut = 0;
        if(RearState)//Rear on
            State = 2;
        else        //Rear off
        {
            //Delay here for stay off while skip AT selector
            // [ (T_now - T_rear_on) > 700ms ]
            State = 1;
            FrontTimeOut = 1000;//10 sec
        }

        RearState = -1;
    }

    
    switch(State)
    {
        case 0:RB1 = 0; if(LastState == 1) Delay_ms(700); RB3 = 0; break; //All off (Delay Video_sel while AudioUnit release camera power)
        case 1:RB1 = 1; RB3 = 1; break; //front
        case 2:RB1 = 1; RB3 = 0; break; //back
        default:RB1 = 0; RB3 = 0; break;
    }
    LastState = State;

    if(VolTarget != 0)
    {
        if(VolTarget > 0)
        {
            VolUp();
            VolTarget -= 2;
            if(VolTarget < 0) VolTarget = 0;
        }
        else //VolTarget < 0
        {
            VolDown();
            VolTarget += 2;
            if(VolTarget > 0) VolTarget = 0;
        }
        Delay_ms(40);
    }

    if(ButtonPressed)
    {
        //Delay_ms(500);
        //while( Get_ADC_BUT(&But1) && (But1 != NO_PRESS) );
        while(GetBut() != 0);
    }

    continue;		// let interrupt do its job
}
    return 0;
}
