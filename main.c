/*
 * main.c
 *
 *  Created on: 21 Aug 2018  /21-Oct-2018
 *      Author: cat
 */


#include "main.h"



char strbuff[32];

uint16_t pulsecnt[GEIGER_TIME+1];  //pulse counter //[0]-current

uint8_t gbuff[GBUFF_SIZE];

volatile uint16_t pulsecnt0=0;  //counter

volatile uint16_t bzcnt=0;  //counter
volatile uint8_t bzact=0; //flag

volatile uint8_t datupd=0;  //flag

volatile uint8_t hvact=0;    //flag

uint8_t hvfreq=INIT_FREQ;  //initial frequency of HV-converter, Hz

uint8_t radbuzz=0;  //0-click, 1-beep, 2-alarm, 3-disable  //settings

volatile uint8_t led1cnt=0;

uint8_t tmr2cnt=0;



//=============================================================================
ISR(PCINT0_vect) //pin change interrupt - wake up by button
    {
    }


//-----------------------------------------------------------------------------
void set_hvfreq(uint16_t freq)
    {
    TCCR1B=0x00; //timer stop
    TCCR1A=0x00;
    HV_CLR;
    if(freq) ICR1=((F_CPU/8)/freq);
    TCNT1=ICR1-1;
    OCR1A=0x0001;  //pwm (pulse width for HV-converter)
    TCCR1A=(1<<COM1A1)|(1<<WGM11);  //COM1A=10 (non-inverting mode)
    TCCR1B=(1<<WGM13)|(1<<WGM12)|(1<<CS11);  //WGM=1110 (FastPWM ICR1-TOP) //prescaler 8
    }


//-----------------------------------------------------------------------------
void beep_start(uint8_t mode, uint16_t ms)
    {
    if(bzact==0)
        {
        bzact=1;
        BZ_CLR;
        OCR0A=TMR0_F0;  //top  //freq. 2kHz
        if(mode==0) OCR0B=TMR0_F1;  //pwm duty cycle //pulse width 50u
        else if(mode==1)  OCR0B=TMR0_F2;  //pulse width 100u
        TCNT0=0;
        TCCR0A=(1<<WGM00)|(1<<WGM01)|(1<<COM0B1);
        TCCR0B=(1<<WGM02)|(1<<CS01);  //prescaler 8
        bzcnt=ms*2;  // /0.5ms
        TIFR0=(1<<OCF0B);
        TIMSK0=(1<<OCIE0B);
        }
    }


//-----------------------------------------------------------------------------
void led1start(uint8_t ms)
    {
    led1cnt=ms/8; //1000/125Hz=8ms
    L1_SET;
    }


//=============================================================================
ISR(INT0_vect) //external interrupt 0 - impulse from detector
    {
    pulsecnt0++;

    if(radbuzz==0) beep_start(1, 2);  //clicks
    else if(radbuzz==1) beep_start(0, 40);  //beeps
    led1start(40);  //flash
    if(hvact==0) set_hvfreq(0);  //boost
    }


//=============================================================================
ISR(TIMER2_OVF_vect) //Timer2 interrupt - 125Hz
    {
    TCNT2=TIMER2_PRELOAD;

    if(++tmr2cnt>=125)  //125/125 = 1Hz  //time counter
        {
        tmr2cnt=0;
        datupd=1;
        }

    if(led1cnt>0)
        {
        led1cnt--;
        if(led1cnt==0) L1_CLR;
        }

    button_cont();
    }


//-----------------------------------------------------------------------------
void beep_stop(void)
    {
    TCCR0B=0;  //stops timer0
    TCCR0A=0;
    TIMSK0=0;
    BZ_CLR;
    bzact=0;
    }


//=============================================================================
ISR(TIMER0_COMPB_vect)
    {
    if(--bzcnt==0) beep_stop();
    }


//-----------------------------------------------------------------------------
void sys_init(void)
    {
    BUTTON_INIT;
    PORTS_INIT;
    UPORTS_INIT;
    ADCINPUT_INIT;

    sei(); //global interrupt enable

    beep_start(0, 120);  //timer0 for buzzer
    lcd_init();
    wait_buttons();

    ACSR=0b10000000;   //comparator disable
    ADCSRB=0x00;
    ADCSRA=0b10000111; //ADC enable //prescaler 128
    DIDR0=0b00000001;  //disable digital input
    ADMUX=0b11000000;  //internal Vref //channel 0

    TIMSK1=0x00;  //timer1 for HV-converter
    TCCR1C=0x00;
    set_hvfreq(hvfreq);  //start HV-converter

    TCCR2A=0x00;  //timer2 for system time
    TCNT2=TIMER2_PRELOAD;
    TCCR2B=TIMER2_PRESCALER64;
    TIFR2=(1<<TOV2);
    TIMSK2=(1<<TOIE2);  //timer2 interrupt enable

    EIFR=(1<<INTF0);
    EICRA=(1<<ISC01);  // - - - - ISC11 ISC10 ISC01 ISC00  //10 - ext. int. 0 on falling edge
    EIMSK=(1<<INT0);   // - - - - - - INT1 INT0  //1 - ext. int.0 enable

    BL_ON;
    }


//-----------------------------------------------------------------------------
void sys_off(void)
    {
    beep_stop();
    beep_start(0, 160);

    wait_buttons();

    TIMSK2=0x00;
    TCCR2B=0x00;

    tmr2cnt=0;
    led1cnt=0;
    L1_CLR;

    BUTTON_SLEEP;

    EIMSK=0x00;
    pulsecnt0=0;

    TIMSK1=0x00;
    TCCR1B=0x00;
    TCCR1A=0x00;
    HV_CLR;

    ADCSRA=0x00;
    ADCINPUT_DEINIT;
    L2_CLR;
    BD_CLR;  //battery divider OFF

    while(bzact);  //wait of timer0 stop

    lcd_deinit();

    PCMSK0=(1<<PCINT3); //change on PIN3 will cause an interrupt
    PCICR=(1<<PCIE0); //enable PCINT0
    SMCR=(1<<SM1)|(1<<SE); //SM2 SM1 SM0 - 010 Power-down mode  //SE Sleep enable
    sleep_cpu();
    cli();
    SMCR=0x00; //disable sleep
    PCICR=0x00; //disable PCINT
    TCCR1B=0x00; //timer stop

    sys_init();
    }


uint32_t doserate=0;  //dose rate, uR/h
uint32_t maxrate=0;
uint32_t dosetot=0;

uint16_t battv=0;  //battery voltage, mV
uint8_t battlow=0;  //flag

uint16_t tday=0; //999 days max
uint8_t thrs=0;
uint8_t tmin=0;

uint8_t hvfcnt=0;   //counter

uint8_t alrmact=0;  //flag

uint8_t alrmevnt=0;  //flag
uint8_t hvfevnt=0;  //flag

uint8_t battoff=0;  //flag

uint8_t btfcnt=0;   //counter

uint8_t cntdown=0;

uint8_t nmscr=0;

uint8_t menuentr=0;  //0-main screen, 1..8-menu


uint16_t lmcnt[10];
uint8_t  lmflag=0;
uint32_t lmrad=0;

int8_t mtemp=0;


//-----------------------------------------------------------------------------
uint8_t conv_ptg (uint8_t inp)
    {
    uint8_t outp=1;
    if(inp) { outp<<=(inp+1); outp-=1; }
    return outp;
    }


//-----------------------------------------------------------------------------
uint8_t sys_mainscr(void)
    {
    static uint16_t max=0;
    static uint16_t dev=0;

    if(nmscr==0) max=0;

    switch(nmscr++)
        {
        case 0: sprintf(strbuff,"%01u.%02u", battv/1000, (battv%1000)/10);  return 1;  //battery voltage "XX.X"
        case 1: lcd_print(1, 1, strbuff);                                   return 1;
        case 2: lcd_upic(1, 35, 3);                                         return 1;  //"V"
        case 3: lcd_print(2, 3, battlow ? "--" : "  ");                     return 1;  //batt. low

        case 4: sprintf(strbuff, "%3d", (menuentr>0) ? (int8_t)hvfreq : mtemp );  return 1;  //temperature/HV-converter freq.
        case 5: lcd_print2(1, 12, strbuff);                                       return 1;
        case 6: (menuentr>0) ? lcd_print2(1, 15, "Hz") : lcd_upic(1, 57, 17);     return 1;
        case 7: lcd_upic(2, 42, (menuentr==1) ? 16 : 6);                          return 1;  //cursor for settings
        case 8: lcd_upic(2, 50, (menuentr==1) ? 15 : 6);                          return 1;

        case 9: sprintf(strbuff, "%6lu", maxrate);         return 1;  //peak rate
        case 10: lcd_print2(1, 18, strbuff);               return 1;
        case 11: lcd_upic(1, 93, 4);                       return 1;  //picture
        case 12: lcd_upic(2, 78, (menuentr==2) ? 16 : 6);  return 1;  //cursor
        case 13: lcd_upic(2, 86, (menuentr==2) ? 15 : 6);  return 1;

        case 14: lcd_upic(3, 10, (hvfcnt>GEIGER_TIME/2) ? 5 : 6);            return 1;  //HV-fail
        case 15: sprintf(strbuff, "%02u",(GEIGER_TIME+1)-hvfcnt);            return 1;
        case 16: lcd_print2(3, 1, (hvfcnt>GEIGER_TIME/2) ? strbuff : "  ");  return 1;

        case 17: sprintf(strbuff, "%6lu", doserate);       return 1;  //rate
        case 18: lcd_print(3, 5, strbuff);                 return 1;
        case 19: lcd_upic(3, 85, 1);                       return 1;
        case 20: lcd_upic(3, 93, 2);                       return 1;

        case 21: lcd_upic(5, 1, BL_IS_ON ? 8 : 11);  return 1;  //backlight

        case 22:
            switch(radbuzz)
                {
                case 0: lcd_upic(5, 13, 9);      return 1; //buzzer
                case 1: lcd_upic(5, 13, 7);      return 1;
                case 2: lcd_print2(5, 4, "50");  return 1;
                case 3: lcd_upic(5, 13, 6);      return 1;
                }
            return 1;

        case 23: lcd_upic(5, 25, alrmevnt ? 12 : 6);  return 1;  //alarm event
        case 24: lcd_upic(5, 37, hvfevnt ? 14 : 6);   return 1;  //hv-fail event
        case 25: lcd_upic(5, 49, battoff ? 13 : 6);   return 1;  //battery low poweroff

        case 26: sprintf(strbuff, "%6lu", dosetot);        return 1;  //dose
        case 27: lcd_print2(5, 17, strbuff);               return 1;
        case 28: lcd_upic(5, 93, 1);                       return 1;  //"uR"
        case 29: lcd_upic(7, 74, (menuentr==3) ? 16 : 6);  return 1;  //cursor
        case 30: lcd_upic(7, 82, (menuentr==3) ? 15 : 6);  return 1;

        case 31: sprintf(strbuff, "%02u", (BATT_FAIL+1)-btfcnt);  return 1;  //batt. off countdown
        case 32: lcd_print2(6,13,btfcnt ? strbuff : "  ");        return 1;

        case 33: sprintf(strbuff, "%3u.%02u.%02u", tday, thrs, tmin);  return 1;  //time of count
        case 34: lcd_print2(6,17,strbuff);                             return 1;

        case 35: sprintf(strbuff, "%02u",(GEIGER_TIME-1)-cntdown);     return 1;  //75sec counter
        case 36: lcd_print(7,1,strbuff);                               return 1;

        case 37: sprintf(strbuff, "%6lu.%1lu", lmrad/10, lmrad%10);    return 1;  //long measurement
        case 38: lcd_print2(7,7,strbuff);                              return 1;

        case 39: for(uint8_t i=1; i<GBUFF_SIZE+1; i++) { if(pulsecnt[i]>max) max=pulsecnt[i]; }  return 1;  //graph
        case 40: dev=(max/8)+1;                                                                  return 1;
        case 41: for(uint8_t i=0; i<GBUFF_SIZE; i++) { gbuff[i]=conv_ptg(pulsecnt[i+1]/dev); }   return 1;
        case 42: lcd_graph(8, gbuff);                                                            return 1;
        case 43: sprintf(strbuff,"%4ux", dev);                                                   return 1;
        case 44: lcd_print2(8,21,strbuff); nmscr=0;                                              return 0;
        }

    return 0;
    }


//-----------------------------------------------------------------------------
uint16_t get_adc(uint8_t admux)
    {
    uint16_t adc=0;
    ADMUX=admux;                    //channel
    REG_BIT_SET(ADCSRA,ADSC);       //start ADC
    while(BIT_IS_CLR(ADCSRA,ADIF)); //wait the end of ADC
    REG_BIT_SET(ADCSRA,ADIF);       //clear flag
    REG_BIT_SET(ADCSRA,ADSC);       //start ADC
    while(BIT_IS_CLR(ADCSRA,ADIF)); //wait the end of ADC
    adc=ADC;
    REG_BIT_SET(ADCSRA,ADIF);       //clear flag
    return adc;
    }


//-----------------------------------------------------------------------------
int main(void)
    {
    sys_init();

    uint32_t pulsetot=0;  //counter

    uint8_t setreg=0b00000000;

    #define PWROFF_CLR  (REG_BIT_CLR(setreg, 0))
    #define PWROFF_SET  (REG_BIT_SET(setreg, 0))
    #define PWROFF      (BIT_IS_SET(setreg,  0))

    #define SCRUPD_CLR  (REG_BIT_CLR(setreg, 1))
    #define SCRUPD_SET  (REG_BIT_SET(setreg, 1))
    #define SCRUPD      (BIT_IS_SET(setreg,  1))

    #define SYSUPD_CLR  (REG_BIT_CLR(setreg, 2))
    #define SYSUPD_SET  (REG_BIT_SET(setreg, 2))
    #define SYSUPD      (BIT_IS_SET(setreg,  2))

    #define RATRST_CLR  (REG_BIT_CLR(setreg, 3))
    #define RATRST_SET  (REG_BIT_SET(setreg, 3))
    #define RATRST      (BIT_IS_SET(setreg,  3))

    #define DOSRST_CLR  (REG_BIT_CLR(setreg, 4))
    #define DOSRST_SET  (REG_BIT_SET(setreg, 4))
    #define DOSRST      (BIT_IS_SET(setreg,  4))

    #define PWRRST_CLR  (REG_BIT_CLR(setreg, 5))
    #define PWRRST_SET  (REG_BIT_SET(setreg, 5))
    #define PWRRST      (BIT_IS_SET(setreg,  5))

    while(1)
        {
        if(PWROFF)
            {
            sys_off();

            hvfevnt=0;
            hvfcnt=0;
            hvact=0;
            alrmevnt=0;
            alrmact=0;
            btfcnt=0;
            battoff=0;
            SCRUPD_CLR;
            nmscr=0;
            PWROFF_CLR;
            PWRRST_SET;
            }

        if(datupd)
            {
            pulsecnt[0]=pulsecnt0;
            pulsecnt0=0;
            static uint8_t tsec=0;
            if(tday<999) { if(++tsec>59) { tsec=0; if(++tmin>59) { tmin=0; if(++thrs>23) { thrs=0; ++tday; } } } }  //time cnts
            if(++cntdown>(GEIGER_TIME-1)) { cntdown=0; lmflag=1; }
            for(uint8_t k=GEIGER_TIME; k>0; k--) pulsecnt[k]=pulsecnt[k-1];  //shift
            pulsetot+=pulsecnt[0];
            if(pulsetot>999999UL*3600/GEIGER_TIME) pulsetot=999999UL*3600/GEIGER_TIME;  //overflow 999999uR
            if(pulsecnt[0]==0) { hvfcnt++; } else { hvfcnt=0; }  //hv-fail timer
            if(hvact) { set_hvfreq(hvfreq); hvact=0; hvfcnt=0; hvfevnt=1; }  //hv normal mode
            if(hvfcnt>GEIGER_TIME) { hvact=1; set_hvfreq(BOOST_FREQ); }  //start hv emergency boost
            if(battoff==1) { if(++btfcnt>BATT_FAIL) PWROFF_SET; } else { btfcnt=0; }  //turning off when the battery low
            datupd=0;
            SCRUPD_CLR;
            nmscr=0;
            SYSUPD_CLR;
            }

        if(DOSRST)
            {
            pulsetot=0;
            tday=0;
            thrs=0;
            tmin=0;
            RATRST_SET;
            DOSRST_CLR;
            }

        if(RATRST)
            {
            maxrate=0;
            PWRRST_SET;
            RATRST_CLR;
            }

        if(PWRRST)
            {
            for(uint8_t i=0; i<GEIGER_TIME+1; i++) pulsecnt[i]=0;
            cntdown=0;
            for(uint8_t i=0; i<10; i++) lmcnt[i]=0;
            lmrad=0;
            lmflag=0;
            SYSUPD_CLR;
            PWRRST_CLR;
            }

        if(!SYSUPD)
            {
            uint32_t tmprate=0;
            for(uint8_t k=GEIGER_TIME; k>0; k--) tmprate+=pulsecnt[k]; //calc dose rate
            if(tmprate>999999) tmprate=999999; //overflow
            doserate=tmprate;
            if(tmprate>maxrate) maxrate=tmprate;   //peak
            dosetot=(pulsetot*GEIGER_TIME/3600);   //dose

            if(lmflag==1)
                {
                lmflag=0;
                lmcnt[0]=tmprate;

                uint32_t tmplmrad=0;
                for(uint8_t i=0; i<10; i++) tmplmrad+=lmcnt[i];
                if(tmplmrad>999999) tmplmrad=999999;
                lmrad=tmplmrad;

                for(uint8_t i=9; i; i--) lmcnt[i]=lmcnt[i-1];
                }

            if(radbuzz==2)
                {
                if(doserate>=50) { alrmact=1; alrmevnt=1; }
                if(alrmact==1)   { beep_stop(); beep_start(0, 400); }
                }

            uint16_t adc0=get_adc(0b11000000);
            static uint16_t oldadc0=0;
            if((oldadc0==0)||(adc0==0)||(adc0==1024)||(adc0>oldadc0+1)||(adc0<oldadc0-1)) { oldadc0 = adc0; }  //filter
            battv=(((uint32_t)oldadc0*ADC_REF/ADC_RES)*VOLT_FACT)/100;
            if(battv>9999) battv=9999;

            (battv<3300) ? (battoff=1) : (battoff=0);
            (battv<3400) ? (battlow=1) : (battlow=0);

            uint16_t temp=get_adc(0b11001000);
            static uint16_t oldtemp=0;
            if((oldtemp==0)||(temp==0)||(temp==1024)||(temp>oldtemp+1)||(temp<oldtemp-1)) { oldtemp = temp; }  //filter
            mtemp=(int16_t)oldtemp-315;
            if(mtemp>99) mtemp=99;
            else if(mtemp<-99) mtemp=-99;

            SYSUPD_SET;
            }

        if(!SCRUPD)  //----------------------  main screen ------------------------
            {
            if(sys_mainscr()==0) SCRUPD_SET;
            }

        switch(button2)  //--------------------- Button #2 ------------------------
            {
            case BUTTON_SHORT:
                button2=0;
                SCRUPD_CLR;
                nmscr=0;
                if(menuentr==0) { alrmact=0; if(++radbuzz>3) radbuzz=0; if(radbuzz==2 && doserate>=50) radbuzz=3; break; }  //buzzer/alarm
                if(menuentr==1) { if(++hvfreq>MAX_FREQ) hvfreq=MAX_FREQ; set_hvfreq(hvfreq); break; }  //boost+
                break;

            case BUTTON_LONG:
                button2=0;
                SCRUPD_CLR;
                nmscr=0;
                if(++menuentr>3) menuentr=0;
                BL_ON;
                break;
            }

        switch(button1)  //--------------------- Button #1 ------------------------
            {
            case BUTTON_SHORT:
                button1=0;
                SCRUPD_CLR;
                nmscr=0;
                if(menuentr==0) { BL_TOGGLE; break; }  //backlight
                if(menuentr==1) { if(--hvfreq<MIN_FREQ) hvfreq=MIN_FREQ; set_hvfreq(hvfreq); break; }  //boost-
                break;

            case BUTTON_LONG:
                button1=0;
                SCRUPD_CLR;
                nmscr=0;
                if(menuentr==0) { PWROFF_SET; break; }  //power OFF
                if(menuentr==1) { menuentr=0; break; }
                if(menuentr==2) { RATRST_SET; menuentr=0; break; }  //clear data
                if(menuentr==3) { DOSRST_SET; menuentr=0; break; }  //clear all
                break;
            }
        }
    return 0;
    }

