/*
 * lcd.c
 *
 *  Created on: 22 Aug 2018
 *      Author: cat
 */


#include "main.h"



//-----------------------------------------------------------------------------
void lcd_send(uint8_t data)  //send byte
    {
    for(uint8_t mask=0x80; mask; mask>>=1)
        {
        CK_CLR;
        (data & mask) ? DT_SET : DT_CLR;
        CK_SET;  //SDIN is sampled at the positive edge of SCLK
        }
    }


//-----------------------------------------------------------------------------
void lcd_goto(uint8_t y, uint8_t x)  //y 0..7 //x 100..0
    {
    COMMAND_MODE;
    CHIP_ENABLE;

    lcd_send(0b00100000);    //0x20 //Function set: standard instruction set
    lcd_send(0b01000000|y);  //0x40 //Y-address
    lcd_send(0b10000000|x);  //0x80 //X-address

    CHIP_DISABLE;
    }


//-----------------------------------------------------------------------------
void lcd_clear(void)
    {
    lcd_goto(0,0);

    DATA_MODE;
    CHIP_ENABLE;

    for(uint16_t i=0; i<((102*64)/8); i++) lcd_send(0x00);

    CHIP_DISABLE;
    }


//-----------------------------------------------------------------------------
void lcd_init(void)
    {
    BL_CLR; BL_OUT;  //backlight OFF

    RS_CLR; RS_OUT;  //lcd reset     //active - LOW
    CE_CLR; CE_OUT;  //chip enable   //active - LOW
    DC_CLR; DC_OUT;  //command mode  //command - LOW / data - HIGH
    CK_CLR; CK_OUT;  //
    DT_CLR; DT_OUT;  //

    PW_SET; PW_OUT;  //display power ON

    _delay_us(10);  //pause for power up //pause for reset

    RESET_INACTIVE;

    _delay_us(10);  //pause before first command

    lcd_send(0b00100001); //0x21 //Function set: extended instruction set

    _delay_ms(20);

    lcd_send(0b00010100); //0x14 //Bias System
    lcd_send(0b00001010); //0x0A //HV-gen voltage multiplication
    lcd_send(0b00000101); //0x05 //Temperature Control
    lcd_send(0b11001100); //0xCC //Set V OP (contrast)

    lcd_send(0b00100000); //0x20 //Function set: standard instruction set
    lcd_send(0b00010001); //0x11 //VLCD programming range: high
    lcd_send(0b00001100); //0x0C //Display control: normal (inverted = 0x0D)

    lcd_clear();
    }


//-----------------------------------------------------------------------------
void lcd_char(uint8_t y, uint8_t x, uint8_t sign)  //y 1..8  //x 1..12
    {
    if((y>0 && y<9) && (x>0 && x<13))
        {
        lcd_goto(y-1, 100-(x*8));

        if(sign<=31 || sign>=127) sign=63;  //32..126

        uint16_t pos = 8*(sign-32);

        DATA_MODE;
        CHIP_ENABLE;

        for(uint8_t i=0; i<8; i++) lcd_send(pgm_read_byte(&font8x8[pos+i]));

        CHIP_DISABLE;
        }
    }


//-----------------------------------------------------------------------------
void lcd_print(uint8_t y, uint8_t x, const char *str) //y 1..8  //x 1..12
    {
    for(; (x<=12 && *str); x++) lcd_char(y, x, *str++);
    }


//-----------------------------------------------------------------------------
void lcd_char2(uint8_t y, uint8_t x, uint8_t sign)  //y 1..8  //x 1..25
    {
    if((y>0 && y<9) && (x>0 && x<26))
        {
        lcd_goto(y-1, 100-(x*4));

        if(sign==32) sign=59;
        else if(sign==45) sign=62;
        else if(sign==65) sign=61;
        else if(sign==72) sign=63;
        else if(sign==86) sign=64;
        else if(sign==120) sign=60;
        else if(sign==122) sign=65;
        else if(sign<48 || sign>57) sign=58;  //48..57

        uint16_t pos = 4*(sign-48);

        DATA_MODE;
        CHIP_ENABLE;

        for(uint8_t i=0; i<4; i++) lcd_send(pgm_read_byte(&font4x8[pos+i]));

        CHIP_DISABLE;
        }
    }

//-----------------------------------------------------------------------------
void lcd_print2(uint8_t y, uint8_t x, const char *str) //y 1..8  //x 1..25
    {
    for(; (x<=25 && *str); x++) lcd_char2(y, x, *str++);
    }


//-----------------------------------------------------------------------------
void lcd_upic(uint8_t y, uint8_t x, const uint8_t addr)  //y 1..8  //x 1..100
    {
    if((y>0 && y<9) && (x>0 && x<94))
        {
        lcd_goto(y-1, 100-((x-1)+8));

        DATA_MODE;
        CHIP_ENABLE;

        uint16_t pos = 8*addr;

        for(uint8_t i=0; i<8; i++) lcd_send(pgm_read_byte(&upic8x8[pos+i]));

        CHIP_DISABLE;
        }
    }


//-----------------------------------------------------------------------------
void lcd_graph(uint8_t y, uint8_t *data)
    {
    lcd_goto(y-1, 100-GEIGER_TIME);

    DATA_MODE;
    CHIP_ENABLE;

    for(uint8_t i=0; i<GBUFF_SIZE; i++) lcd_send(data[i]);

    CHIP_DISABLE;
    }


//-----------------------------------------------------------------------------
void lcd_deinit(void)
    {
    COMMAND_MODE;
    CHIP_ENABLE;

    lcd_send(0b00100000);  //0x20 //Function set: standard instruction set
    lcd_send(0b00001000);  //0x08 //display blank
    lcd_send(0b00100100);  //0x24 //power-down

    CHIP_DISABLE;

    _delay_ms(20);

    BL_CLR;  //backlight OFF

    RS_CLR;  //chip reset
    CE_CLR;  //chip enable
    DC_CLR;  //command mode
    CK_CLR;  //
    DT_CLR;  //

    PW_CLR;  //display power OFF

    _delay_ms(300);  //delay for system power off
    }




