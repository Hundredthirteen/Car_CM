/*
 * File:   main.c
 * Author: 27783
 *
 * Created on 13 ??????? 2015 ?., 15:09
 */

//#include <stdio.h>
//#include <stdlib.h>


// PIC16F819 Configuration Bit Settings

// 'C' source line config statements

#include <xc.h>

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// CONFIG
#pragma config FOSC = INTOSCIO  // Oscillator Selection bits (INTRC oscillator; port I/O function on both RA6/OSC2/CLKO pin and RA7/OSC1/CLKI pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
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

//#define ADC_DELTA_CONST   100//500mV
//#define ADC_DELTA_CONST   50//250mV
//#define ADC_DELTA_CONST   35//175mV
#define ADC_DELTA_CONST   20//100mV
//#define ADC_DELTA_CONST   10//50mV

#define _3CH


unsigned int ADC_DELTA;

unsigned char PORTA_ALIAS;

unsigned char ALWAYS_VOLUME;
unsigned char VOLUME_DELAY;
volatile unsigned int DELAY_C_COUNT;
volatile unsigned int DELAY_COUNT;
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

unsigned int CAMERA_BUT, AUDIO_BUT, NO_BUT, SECOND_BUT;
unsigned char USE_SEC_BUT = 0;
#ifdef _3CH
unsigned char USE_3CH = 0;
#endif

int i;
unsigned int u16;
//                      0  1 2 3  4 5 6  7   8 9 A B  C D  E F
//signed char states[] = {0,-1,1,0, 1,0,0,-1, -1,0,0,1, 0,1,-1,0};//Encoder directions CCW
signed char states[] = {0,1,-1,0, -1,0,0,1, 1,0,0,-1, 0,-1,1,0};//Encoder directions CW


void CM_Rotate(void);

signed char COMPARE_ADC(signed int adc_val, signed int target)
{
    if(adc_val > target + ADC_DELTA) return 1;
    if(adc_val + ADC_DELTA < target) return -1;
    return 0;
}

void interrupt V1(void)
{
    if(TMR2IF) //100Hz; 10ms
    {
        if(FrontTimeOut)
        {
            FrontTimeOut--;
            if(!FrontTimeOut)
                State = 0;
        }

        if(DELAY_C_COUNT > 0)
            DELAY_C_COUNT--;
        
        if(CM_ModeTimeOut > 0)
        {
            CM_ModeTimeOut--;
            if(CM_ModeTimeOut == 0)
                CM_Mode = ALWAYS_VOLUME;

        }        
        
        CM_Rotate();
        
        TMR2IF = 0;
    }

    if(TMR0IF) //1000Hz; 1ms
    {       
        if(DELAY_COUNT)
            DELAY_COUNT--;

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
    while(DELAY_COUNT);
}

void VolUp(void)
{
    //RA7 = 1;
    //RA2 = 1;
    //PORTA |= 0x84;
    PORTA_ALIAS |= 0x80;
    PORTA = PORTA_ALIAS;
    Delay_ms(VOLUME_DELAY);
    //RA7 = 0;
    //RA2 = 1;
    PORTA_ALIAS &= 0x7F;
    PORTA = PORTA_ALIAS;
}

void VolDown(void)
{
    //RA6 = 1;
    //RA2 = 1;
    //PORTA |= 0x44;
    PORTA_ALIAS |= 0x40;
    PORTA = PORTA_ALIAS;
    Delay_ms(VOLUME_DELAY);
    //RA6 = 0;
    //RA2 = 1;
    PORTA_ALIAS &= 0xBF;
    PORTA = PORTA_ALIAS;
}

void CM_Rotate(void)
{
    unsigned char ENC_AB;
//    0 - error, no move
//    1 - CW
//    2 - CCW
    ENC_AB = ((PORTB >> 4) & 0x03);
    
    if( (ENC_AB_LAST != ENC_AB) )
    {
        if(CM_Mode)//Volume
        {
            VolTarget += states[((ENC_AB_LAST<<2) | ENC_AB)];
        }
        else//Menu navigation
        {
            RB6 = (~ENC_AB & 0x01);
            RB7 = ((~ENC_AB>>1) & 0x01);
            
            if(ALWAYS_VOLUME)
                CM_ModeTimeOut = 600;
        }
        ENC_AB_LAST = ENC_AB;
    }
}

signed int Get_ADC(void)
{
    GO_nDONE = 1;
    DELAY_COUNT = 150;
    while( GO_nDONE && DELAY_COUNT);//Wait for ADC
    if(!GO_nDONE)
        return ((ADRESH<<8) + ADRESL);

    //else ADC not ready
    return -1;
}

int GetBut()
{
    signed int adc_res;
    
    if(USE_SEC_BUT)
    {
        ADC_DELTA = 200;
        ADCON0bits.CHS = 1;//AN1
        NOP();
        NOP();
        adc_res = Get_ADC();
        if(adc_res >= 0)
        {
            if(COMPARE_ADC(adc_res, SECOND_BUT) == 0) return 1;
        }
        else
            return -1;
        
        ADCON0bits.CHS = 0;//AN0
        NOP();
        NOP();
    }
  
    ADC_DELTA = ADC_DELTA_CONST;
    adc_res = Get_ADC();
    if(adc_res >= 0)
    {
        if(COMPARE_ADC(adc_res, CAMERA_BUT) == 0) return 1;
        if(COMPARE_ADC(adc_res, AUDIO_BUT) == 0) return 2;
        if(COMPARE_ADC(adc_res, NO_BUT) == 0) return 0;
    }

    return -1;
}

void CameraFront()
{
    PORTB |= 0x0A;//Enable video + select front
#ifdef _3CH
    PORTA_ALIAS &= ~0x04;//unselect 3ch
    PORTA = PORTA_ALIAS;
#endif
}

void CameraRear()
{
    PORTB |= 0x02;//Enable video
    PORTB &= ~0x08;//select rear
#ifdef _3CH
    PORTA_ALIAS &= ~0x04;//unselect 3ch
    PORTA = PORTA_ALIAS;
#endif
}

void CameraOff()
{
    PORTB &= ~0x0A;//Disable video + select rear
#ifdef _3CH
    PORTA_ALIAS &= ~0x04;//unselect 3ch
    PORTA = PORTA_ALIAS;
#endif
}

#ifdef _3CH
void ThirdChannel()
{
    PORTB |= 0x02;//Enable video
    PORTB &= ~0x08;//select rear
    PORTA_ALIAS |= 0x04;//select 3ch
    PORTA = PORTA_ALIAS;
}
#endif

void CamBlink()
{
    CameraFront();
    Delay_ms(500);
    CameraRear();
    Delay_ms(500);
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

    PORTA_ALIAS = 0x00;
    PORTA = PORTA_ALIAS;
    TRISA &= ~0xC0;//RA6 & RA7 output

#ifdef _3CH
    TRISA &= ~0x04;//RA2 output
    //PORTA &= ~0x04;
#endif
    
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
    ADCON1bits.ADCS2 = 1;//Fosc/2
    ADCON1bits.ADFM = 1;//Right justifield
    ADCON1bits.PCFG = 0x40;
    ADCON0bits.ADCS = 3;//Fosc/16
    ADCON0bits.CHS = 0;//AN0

    ADIF = 0;
    ADIE = 0;

    ADON = 1;

    ALWAYS_VOLUME = 0;
    CM_Mode = 0;//Menu navigation
    CM_Rotate();

    RBIF = 0;
    RBIE = 0;//1;

    VOLUME_DELAY = 45;
    
    PEIE = 1;
    GIE = 1;

    USE_SEC_BUT = 0;
    State = 0;
    RearStateLast = RB0;
    FrontTimeOut = 0;
    VolTarget = 0;
    ADC_DELTA = ADC_DELTA_CONST;

    DELAY_C_COUNT = 50;
    while(DELAY_C_COUNT);//wait 500ms

    ADCON0bits.CHS = 0;//AN0
    DELAY_C_COUNT = 500;
    while((DELAY_C_COUNT) && (Get_ADC() < 150)) ; //AUDIO == 90 

    //PORTA &= ~0x04;//unselect 3ch
    
    if(Get_ADC() < 150)//Programming
    {
        CM_Mode = 1;//volume control
        CM_Rotate();
        VolTarget = 0;
        
        CameraFront();
        Delay_ms(2000);
        u16 = 0;
        for(i=0;i<10;i++) u16 += Get_ADC();
        NO_BUT = u16/10;
        CameraOff();
        Delay_ms(1000);
        
        CameraFront();
        while(GetBut() == 0);//wait pressing
        Delay_ms(100);
        u16 = 0;
        for(i=0;i<10;i++) u16 += Get_ADC();
        CAMERA_BUT = u16/10;
        CameraOff();
        while(GetBut() != 0);//wait releasing
        Delay_ms(1000);
       
        CameraFront();
        while(GetBut() == 0);//wait pressing
        Delay_ms(100);
        u16 = 0;
        for(i=0;i<10;i++) u16 += Get_ADC();
        AUDIO_BUT = u16/10;
        CameraOff();
        while(GetBut() != 0);//wait releasing
        Delay_ms(1000);
        
        CamBlink();
        CamBlink();
        
        CameraFront();
        while(GetBut() == 0);//wait pressing
        Delay_ms(100);
        u16 = 0;
        ADCON0bits.CHS = 1;//AN1
        for(i=0;i<10;i++) u16 += Get_ADC();
        ADCON0bits.CHS = 0;//AN0        
        SECOND_BUT = u16/10;
        CameraOff();
        while(GetBut() != 0);//wait releasing
        Delay_ms(1000);
        
        CameraFront();
        while(GetBut() == 0);//wait pressing
        Delay_ms(100);
        u16 = 0;
        ADCON0bits.CHS = 1;//AN1
        for(i=0;i<10;i++) u16 += Get_ADC();
        ADCON0bits.CHS = 0;//AN0
        CameraOff();
        Delay_ms(1000);
        
        if(COMPARE_ADC(SECOND_BUT, u16/10) != 0)
        {
            SECOND_BUT = u16/10;
            USE_SEC_BUT = 1;
            CamBlink();
            CamBlink();
        }
                
        VolTarget = 0;
        CameraFront();

        while(VolTarget == 0);
        if(VolTarget < 0)
            VOLUME_DELAY = 40;
        else
            VOLUME_DELAY = 120;
#ifdef _3CH
        CameraRear();
        Delay_ms(500);
        CamBlink();
            
        VolTarget = 0;
        CameraFront();

        while(VolTarget == 0);
        if(VolTarget < 0)
            USE_3CH = 0;
        else
            USE_3CH = 1;
#endif
        
        CameraRear();
        Delay_ms(500);
        CamBlink();
            
        VolTarget = 0;
        CameraFront();

        while(VolTarget == 0);
        if(VolTarget < 0)
            ALWAYS_VOLUME = 0;
        else
            ALWAYS_VOLUME = 1;
        
        eeprom_write(0x00, (unsigned char)((NO_BUT)>>8));
        eeprom_write(0x01, (unsigned char)((NO_BUT) & 0xFF));//Buttons released
        eeprom_write(0x02, (unsigned char)((CAMERA_BUT)>>8));
        eeprom_write(0x03, (unsigned char)((CAMERA_BUT) & 0xFF));//Camera But
        eeprom_write(0x04, (unsigned char)((AUDIO_BUT)>>8));
        eeprom_write(0x05, (unsigned char)((AUDIO_BUT) & 0xFF));//Audio on
        eeprom_write(0x06, (unsigned char)((u16/10)>>8));
        eeprom_write(0x07, (unsigned char)((u16/10) & 0xFF));//Second but
        eeprom_write(0x08, USE_SEC_BUT);
        eeprom_write(0x09, VOLUME_DELAY);
#ifdef _3CH
        eeprom_write(0x0A, USE_3CH);
#endif
        eeprom_write(0x0B, ALWAYS_VOLUME);
        CameraOff();
    }
    else//Read saved
    {
        NO_BUT = ((eeprom_read(0x00)<<8) + eeprom_read(0x01));
        CAMERA_BUT = ((eeprom_read(0x02)<<8) + eeprom_read(0x03));
        AUDIO_BUT = ((eeprom_read(0x04)<<8) + eeprom_read(0x05));
        SECOND_BUT = ((eeprom_read(0x06)<<8) + eeprom_read(0x07));
        USE_SEC_BUT = eeprom_read(0x08);
        VOLUME_DELAY = eeprom_read(0x09);
#ifdef _3CH
        USE_3CH = eeprom_read(0x0A);
#endif
        ALWAYS_VOLUME = eeprom_read(0x0B);
    }

    VolTarget = 0;
    CM_Mode = ALWAYS_VOLUME;//Menu navigation or Volume mode
    
for(;;)
{
    RearState = -1;

    ButtonPressed = GetBut();
    Delay_ms(1);
    if(ButtonPressed != GetBut())
        ButtonPressed = 0;
    
    if(ButtonPressed < 0)
    {
        ButtonPressed = 0;
        CM_Mode = ALWAYS_VOLUME;//Menu navigation or Volume mode
        CM_ModeTimeOut = 0;
        VolTarget = 0;
    }

    if(ButtonPressed == 1)//Navi
    {
        FrontTimeOut = 0;
        State++;
#ifdef _3CH
        if(State>3)
            State = 0;//off
        
        if((USE_3CH == 0) && (State > 2))
            State = 0;//off
#else
        if(State>2)
            State = 0;//off
#endif
        /*
        if(0)//without BAT54C, camera can turn off while R position
        {
            if(State == 0)//Off
                State = 1;//Front
        }
        */
    }

    if(ButtonPressed == 2)//Audio
    {
        VolTarget = 0;

        if(CM_Mode > 0)
        {
            CM_Mode = 0;//Menu navigation
                
            if(ALWAYS_VOLUME == 0)
            {
                CM_ModeTimeOut = 0;            
                //State = 0;
                FrontTimeOut = 0;
            }
        }
        else
        {
            CM_Mode = 1;//Volume mode
            if(ALWAYS_VOLUME)
            {
                CM_ModeTimeOut = 0;
                //State = 0;
                FrontTimeOut = 0;
            }
            else
                CM_ModeTimeOut = 600;
            
            VolDown();
            Delay_ms(VOLUME_DELAY); //+/- for draw volume bar on display
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
    
#ifdef _3CH
        case 0:RB1 = 0; if((LastState == 1) || (LastState == 3)) Delay_ms(1000); RB3 = 0; PORTA_ALIAS &= ~0x04; PORTA = PORTA_ALIAS; break; //All off (Delay Video_sel while AudioUnit release camera power)
#else
        case 0:RB1 = 0; if(LastState == 1) Delay_ms(1000); RB3 = 0; break; //All off (Delay Video_sel while AudioUnit release camera power)
#endif
        case 1:CameraFront(); break; //front
        case 2:CameraRear(); break; //back
#ifdef _3CH
        case 3:ThirdChannel(); break; //3CH
#endif
        default:CameraOff(); break;
    }
    LastState = State;

    if(VolTarget != 0)
    {   
        if(ALWAYS_VOLUME == 0)
            CM_ModeTimeOut = 600;
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
        Delay_ms(VOLUME_DELAY);
    }

    Delay_ms(1);
    if(ButtonPressed)
    {
        while(GetBut() != 0);
    }
    continue;		// let interrupt do its job
}
}
