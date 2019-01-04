/*
 * main.h
 *
 *  Created on: 22 Aug 2018 /27 Oct 2018
 *      Author: cat
 *
 *  atmega328p 2MHz
 */

#ifndef MAIN_H_
#define MAIN_H_


#include <stdint.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <util/delay.h>



//bitman
#define REG_BIT_SET(reg, bit) ((reg) |= (1<<(bit)))
#define REG_BIT_CLR(reg, bit) ((reg) &= ~(1<<(bit)))
#define REG_BIT_INV(reg, bit) ((reg) ^= (1<<(bit)))

#define BIT_IS_SET(reg, bitn) ((reg)>>(bitn)&1)
#define BIT_IS_CLR(reg, bitn) (!((reg)>>(bitn)&1))


//HV converter
#define HV_BIT   1
#define HV_PORT  PORTB
#define HV_DDR   DDRB

//geiger detector
#define GD_BIT   2
#define GD_PORT  PORTD
#define GD_DDR   DDRD

//external LED
#define L1_BIT   3
#define L1_PORT  PORTC
#define L1_DDR   DDRC

//internal LED
#define L2_BIT   5
#define L2_PORT  PORTB
#define L2_DDR   DDRB

//buzzer
#define BZ_BIT   5
#define BZ_PORT  PORTD
#define BZ_DDR   DDRD

//battery divider
#define BD_BIT   1
#define BD_PORT  PORTD
#define BD_DDR   DDRD


#define HV_CLR   (REG_BIT_CLR(HV_PORT, HV_BIT))
#define HV_SET   (REG_BIT_SET(HV_PORT, HV_BIT))
#define HV_OUT   (REG_BIT_SET(HV_DDR, HV_BIT))
#define HV_INP   (REG_BIT_CLR(HV_DDR, HV_BIT))

#define GD_CLR   (REG_BIT_CLR(GD_PORT, GD_BIT))
#define GD_SET   (REG_BIT_SET(GD_PORT, GD_BIT))
#define GD_OUT   (REG_BIT_SET(GD_DDR, GD_BIT))
#define GD_INP   (REG_BIT_CLR(GD_DDR, GD_BIT))

#define L1_CLR   (REG_BIT_CLR(L1_PORT, L1_BIT))
#define L1_SET   (REG_BIT_SET(L1_PORT, L1_BIT))
#define L1_INV   (REG_BIT_INV(L1_PORT, L1_BIT))
#define L1_OUT   (REG_BIT_SET(L1_DDR, L1_BIT))
#define L1_INP   (REG_BIT_CLR(L1_DDR, L1_BIT))

#define L2_CLR   (REG_BIT_CLR(L2_PORT, L2_BIT))
#define L2_SET   (REG_BIT_SET(L2_PORT, L2_BIT))
#define L2_INV   (REG_BIT_INV(L2_PORT, L2_BIT))
#define L2_OUT   (REG_BIT_SET(L2_DDR, L2_BIT))
#define L2_INP   (REG_BIT_CLR(L2_DDR, L2_BIT))

#define BZ_CLR   (REG_BIT_CLR(BZ_PORT, BZ_BIT))
#define BZ_SET   (REG_BIT_SET(BZ_PORT, BZ_BIT))
#define BZ_OUT   (REG_BIT_SET(BZ_DDR, BZ_BIT))
#define BZ_INP   (REG_BIT_CLR(BZ_DDR, BZ_BIT))

#define BD_CLR   (REG_BIT_CLR(BD_PORT, BD_BIT))
#define BD_SET   (REG_BIT_SET(BD_PORT, BD_BIT))
#define BD_OUT   (REG_BIT_SET(BD_DDR, BD_BIT))
#define BD_INP   (REG_BIT_CLR(BD_DDR, BD_BIT))

#define HV_INIT  HV_CLR; HV_OUT
#define BZ_INIT  BZ_CLR; BZ_OUT
#define GD_INIT  GD_INP; GD_SET
#define L1_INIT  L1_CLR; L1_OUT
#define L2_INIT  L2_CLR; L2_OUT
#define BD_INIT  BD_SET; BD_OUT

#define PORTS_INIT  HV_INIT; BZ_INIT; GD_INIT; L1_INIT; L2_INIT; BD_INIT


//LCD serial data input line (SDIN)
#define DT_BIT   1
#define DT_PORT  PORTC
#define DT_DDR   DDRC

//LCD serial clock line (SCLK)
#define CK_BIT   3
#define CK_PORT  PORTD
#define CK_DDR   DDRD

//LCD mode select (D/C)
#define DC_BIT   4
#define DC_PORT  PORTD
#define DC_DDR   DDRD

//LCD reset (RES)
#define RS_BIT   0
#define RS_PORT  PORTB
#define RS_DDR   DDRB

//LCD chip enable (SCE)
#define CE_BIT   2
#define CE_PORT  PORTC
#define CE_DDR   DDRC

//LCD power
#define PW_BIT   6
#define PW_PORT  PORTD
#define PW_DDR   DDRD

//LCD backlight
#define BL_BIT   7
#define BL_PORT  PORTD
#define BL_DDR   DDRD


#define DT_CLR  (REG_BIT_CLR(DT_PORT, DT_BIT))
#define DT_SET  (REG_BIT_SET(DT_PORT, DT_BIT))
#define DT_OUT  (REG_BIT_SET(DT_DDR, DT_BIT))
#define DT_INP  (REG_BIT_CLR(DT_DDR, DT_BIT))

#define CK_CLR  (REG_BIT_CLR(CK_PORT, CK_BIT))
#define CK_SET  (REG_BIT_SET(CK_PORT, CK_BIT))
#define CK_OUT  (REG_BIT_SET(CK_DDR, CK_BIT))
#define CK_INP  (REG_BIT_CLR(CK_DDR, CK_BIT))

#define DC_CLR  (REG_BIT_CLR(DC_PORT, DC_BIT))
#define DC_SET  (REG_BIT_SET(DC_PORT, DC_BIT))
#define DC_OUT  (REG_BIT_SET(DC_DDR, DC_BIT))
#define DC_INP  (REG_BIT_CLR(DC_DDR, DC_BIT))

#define DATA_MODE     DC_SET
#define COMMAND_MODE  DC_CLR

#define RS_CLR  (REG_BIT_CLR(RS_PORT, RS_BIT))
#define RS_SET  (REG_BIT_SET(RS_PORT, RS_BIT))
#define RS_OUT  (REG_BIT_SET(RS_DDR, RS_BIT))
#define RS_INP  (REG_BIT_CLR(RS_DDR, RS_BIT))

#define RESET_ACTIVE    RS_CLR
#define RESET_INACTIVE  RS_SET

#define CE_CLR  (REG_BIT_CLR(CE_PORT, CE_BIT))
#define CE_SET  (REG_BIT_SET(CE_PORT, CE_BIT))
#define CE_OUT  (REG_BIT_SET(CE_DDR, CE_BIT))
#define CE_INP  (REG_BIT_CLR(CE_DDR, CE_BIT))

#define CHIP_ENABLE   CE_CLR
#define CHIP_DISABLE  CE_SET

#define PW_CLR  (REG_BIT_CLR(PW_PORT, PW_BIT))
#define PW_SET  (REG_BIT_SET(PW_PORT, PW_BIT))
#define PW_OUT  (REG_BIT_SET(PW_DDR, PW_BIT))
#define PW_INP  (REG_BIT_CLR(PW_DDR, PW_BIT))

#define BL_CLR  (REG_BIT_CLR(BL_PORT, BL_BIT))
#define BL_SET  (REG_BIT_SET(BL_PORT, BL_BIT))
#define BL_OUT  (REG_BIT_SET(BL_DDR, BL_BIT))
#define BL_INP  (REG_BIT_CLR(BL_DDR, BL_BIT))

#define BL_INV  (REG_BIT_INV(BL_PORT, BL_BIT))

#define BL_TOGGLE  BL_INV

#define BL_ON   BL_SET
#define BL_OFF  BL_CLR

#define BL_IS_ON  BIT_IS_SET(BL_PORT,  BL_BIT)


//button1
#define B1_BIT   PIN3
#define B1_PORT  PORTB
#define B1_DDR   DDRB
#define B1_PIN   PINB

//button2
#define B2_BIT   PIN4
#define B2_PORT  PORTB
#define B2_DDR   DDRB
#define B2_PIN   PINB

//TIMEx8ms (for 125Hz interrupt)
#define DEB_TIME 5
#define HLD_TIME 80

#define B1_CLR   (REG_BIT_CLR(B1_PORT, B1_BIT))
#define B1_SET   (REG_BIT_SET(B1_PORT, B1_BIT))
#define B1_OUT   (REG_BIT_SET(B1_DDR, B1_BIT))
#define B1_INP   (REG_BIT_CLR(B1_DDR, B1_BIT))

#define B2_CLR   (REG_BIT_CLR(B2_PORT, B2_BIT))
#define B2_SET   (REG_BIT_SET(B2_PORT, B2_BIT))
#define B2_OUT   (REG_BIT_SET(B2_DDR, B2_BIT))
#define B2_INP   (REG_BIT_CLR(B2_DDR, B2_BIT))

#define BUTTON_INIT   B1_INP; B1_SET; B2_INP; B2_SET

#define BUTTON_SLEEP  B2_CLR; B2_OUT


#define BUTTON1UP    (BIT_IS_SET(B1_PIN, B1_BIT))
#define BUTTON2UP    (BIT_IS_SET(B2_PIN, B2_BIT))
#define BUTTON1      (BIT_IS_CLR(B1_PIN, B1_BIT))
#define BUTTON2      (BIT_IS_CLR(B2_PIN, B2_BIT))
#define BUTTON_ANY   (BIT_IS_CLR(B1_PIN, B1_BIT) || BIT_IS_CLR(B2_PIN, B2_BIT))
#define BUTTON_BOTH  (BIT_IS_CLR(B1_PIN, B1_BIT) && BIT_IS_CLR(B2_PIN, B2_BIT))

#define BUTTON_SHORT 1
#define BUTTON_LONG  2



///adc input pin

#define ADCINPUT_BIT   0
#define ADCINPUT_PORT  PORTC
#define ADCINPUT_DDR   DDRC

#define ADCINPUT_CLR  (REG_BIT_CLR(ADCINPUT_PORT, ADCINPUT_BIT))
#define ADCINPUT_SET  (REG_BIT_SET(ADCINPUT_PORT, ADCINPUT_BIT))
#define ADCINPUT_OUT  (REG_BIT_SET(ADCINPUT_DDR, ADCINPUT_BIT))
#define ADCINPUT_INP  (REG_BIT_CLR(ADCINPUT_DDR, ADCINPUT_BIT))

#define ADCINPUT_INIT  ADCINPUT_CLR; ADCINPUT_INP

#define ADCINPUT_DEINIT  ADCINPUT_CLR; ADCINPUT_OUT


//unused pins: pd0 adc6/adc7 pb2 pc4 pc5

#define U1_BIT   0
#define U1_PORT  PORTD
#define U1_DDR   DDRD

#define U2_BIT   2
#define U2_PORT  PORTB
#define U2_DDR   DDRB

#define U3_BIT   4
#define U3_PORT  PORTC
#define U3_DDR   DDRC

#define U4_BIT   5
#define U4_PORT  PORTC
#define U4_DDR   DDRC


#define U1_CLR  (REG_BIT_CLR(U1_PORT, U1_BIT))
#define U1_SET  (REG_BIT_SET(U1_PORT, U1_BIT))
#define U1_OUT  (REG_BIT_SET(U1_DDR, U1_BIT))
#define U1_INP  (REG_BIT_CLR(U1_DDR, U1_BIT))

#define U2_CLR  (REG_BIT_CLR(U2_PORT, U2_BIT))
#define U2_SET  (REG_BIT_SET(U2_PORT, U2_BIT))
#define U2_OUT  (REG_BIT_SET(U2_DDR, U2_BIT))
#define U2_INP  (REG_BIT_CLR(U2_DDR, U2_BIT))

#define U3_CLR  (REG_BIT_CLR(U3_PORT, U3_BIT))
#define U3_SET  (REG_BIT_SET(U3_PORT, U3_BIT))
#define U3_OUT  (REG_BIT_SET(U3_DDR, U3_BIT))
#define U3_INP  (REG_BIT_CLR(U3_DDR, U3_BIT))

#define U4_CLR  (REG_BIT_CLR(U4_PORT, U4_BIT))
#define U4_SET  (REG_BIT_SET(U4_PORT, U4_BIT))
#define U4_OUT  (REG_BIT_SET(U4_DDR, U4_BIT))
#define U4_INP  (REG_BIT_CLR(U4_DDR, U4_BIT))

#define U1_INIT  U1_CLR; U1_OUT
#define U2_INIT  U2_CLR; U2_OUT
#define U3_INIT  U3_CLR; U3_OUT
#define U4_INIT  U4_CLR; U4_OUT

#define UPORTS_INIT  U1_INIT; U2_INIT; U3_INIT; U4_INIT



//CSn2 CSn1 CSn0
/*
#define TIMER0_STOP           0b00000000
#define TIMER0_PRESCALER1     0b00000001
#define TIMER0_PRESCALER8     0b00000010
#define TIMER0_PRESCALER64    0b00000011
#define TIMER0_PRESCALER256   0b00000100
#define TIMER0_PRESCALER1024  0b00000101
*/
#define TIMER1_STOP           0b00000000
#define TIMER1_PRESCALER1     0b00000001
#define TIMER1_PRESCALER8     0b00000010
#define TIMER1_PRESCALER64    0b00000011
#define TIMER1_PRESCALER256   0b00000100
#define TIMER1_PRESCALER1024  0b00000101

#define TIMER2_STOP           0b00000000
#define TIMER2_PRESCALER1     0b00000001
#define TIMER2_PRESCALER8     0b00000010
#define TIMER2_PRESCALER32    0b00000011
#define TIMER2_PRESCALER64    0b00000100
#define TIMER2_PRESCALER128   0b00000101
#define TIMER2_PRESCALER256   0b00000110
#define TIMER2_PRESCALER1024  0b00000111



//Clock timer frequency
#define CALC_TMR2_FREQ       125
#define FACT_TMR2_PRESCALER  64
#define TIMER2_PRELOAD       (256-(F_CPU/FACT_TMR2_PRESCALER)/CALC_TMR2_FREQ)


//Buzzer frequency
#define TMR0_F0  ((F_CPU/8)/2000)
//Buzzer pulse width  //beep
#define TMR0_F1  ((F_CPU/8)/20000)
//Buzzer pulse width  //click
#define TMR0_F2  ((F_CPU/8)/10000)


#define INIT_FREQ   16
#define MIN_FREQ    10
#define MAX_FREQ    50
#define BOOST_FREQ  500


//Vbat=((ADC*Vreference/ADCresolution)*((R1+R2)/R1)*100)/100
//VOLT_FACT=((R1+R2)/R1)*100    //_1101_
#define VOLT_FACT 1110

#define ADC_REF   1100
#define ADC_RES   1024

//sec.
#define BATT_FAIL 30


//time of measurement - 75 sec (for SI29BG Geiger Tube)
#define GEIGER_TIME 75

#define GBUFF_SIZE 75


//lcd.c
void lcd_init(void);
void lcd_clear(void);
void lcd_char(uint8_t y, uint8_t x, uint8_t sign);      //y 1..8  //x 1..12
void lcd_upic(uint8_t y, uint8_t x, uint8_t addr);  //y 1..8  //x 1..100
void lcd_print(uint8_t y, uint8_t x, const char *str);  //y 1..8  //x 1..12
void lcd_deinit(void);

void lcd_char2(uint8_t y, uint8_t x, uint8_t sign);  //y 1..8  //x 1..25
void lcd_print2(uint8_t y, uint8_t x, const char *str); //y 1..8  //x 1..25

void lcd_graph(uint8_t y, uint8_t *data);


//font.c
extern const uint8_t font8x8[];
extern const uint8_t upic8x8[];
extern const uint8_t font4x8[];


//button.c
void wait_buttons(void);
void button_cont(void);

extern volatile uint8_t button1; //0-not pressed, 1-short press, 2-long press
extern volatile uint8_t button2;


//



#endif /* MAIN_H_ */




