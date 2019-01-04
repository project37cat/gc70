/*
 * button.c
 *
 *  Created on: 26 Aug 2018 г.
 *      Author: cat
 */



#include "main.h"



volatile uint8_t button1; //0-not pressed, 1-short press, 2-long press
volatile uint8_t button2;



//-----------------------------------------------------------------------------
void wait_buttons(void)
    {
    while(BUTTON_ANY);
    _delay_ms(50);
    button1=0;
    button2=0;
    }


//-----------------------------------------------------------------------------
void button_cont(void)
    {
    static uint8_t tmp1=0; //flags
    static uint8_t tmp2=0;

    static uint8_t deb1=0; //counters
    static uint8_t deb2=0;
    static uint8_t hld1=0;
    static uint8_t hld2=0;

    if(button1==0)
        {
        if(BUTTON1UP)
            {
            if(deb1>0)
                {
                deb1--;
                if(deb1==0 && tmp1==1)
                    {
                    tmp1=0;
                    button1=1;
                    hld1=0;
                    }
                }
            }
        else
            {
            if(deb1<DEB_TIME)
                {
                if(++deb1==DEB_TIME) tmp1=1;
                }
            if(tmp1==1)
                {
                if(hld1<HLD_TIME)
                    {
                    hld1++;
                    if(hld1==HLD_TIME)
                        {
                        tmp1=0;
                        button1=2;
                        hld1=0;
                        }
                    }
                }
            }
        }

    if(button2==0)
        {
        if(BUTTON2UP)
            {
            if(deb2>0)
                {
                deb2--;
                if(deb2==0 && tmp2==1)
                    {
                    tmp2=0;
                    button2=1;
                    hld2=0;
                    }
                }
            }
        else
            {
            if(deb2<DEB_TIME)
                {
                if(++deb2==DEB_TIME) tmp2=1;
                }
            if(tmp2==1)
                {
                if(hld2<HLD_TIME)
                    {
                    hld2++;
                    if(hld2==HLD_TIME)
                        {
                        tmp2=0;
                        button2=2;
                        hld2=0;
                        }
                    }
                }
            }
        }
    }




