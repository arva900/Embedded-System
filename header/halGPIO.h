#ifndef _halGPIO_H_
#define _halGPIO_H_

#include "../header/bsp_430x2xx.h"		// private library - BSP layer

#include  "../header/app.h"    		// private library - APP layer


extern volatile char sendFRAME[14];//max send 13 bytes of adc calib
extern char sendIdx, sendLenth;
extern volatile char rcvFRAME[3];
extern char rcvIdx, rcvLenth;
extern unsigned int echo_cap;
extern char angle,prev_angle,end_angle;
#define ACK 0xFF
#define EXIT 0xFE


extern void sysConfig(void);
extern void enterLPM(unsigned char);
extern void enable_interrupts(void);
extern void disable_interrupts(void);
extern void PB_EN_interrupt(int ena);
char diff;
extern void MOVEtoANGLE(void);
extern void filesLCDhandle(void);

__interrupt void PBs_handler(void);
__interrupt void TIMER1_A0_ISR(void);
__interrupt void TIMER1_A1_ISR(void);
__interrupt void TIMER0_A0_ISR(void);
__interrupt void TIMER0_A1_ISR(void);
__interrupt void ADC10_ISR (void);
__interrupt void USCI0RX_ISR(void);
__interrupt void USCI0TX_ISR(void);

extern void lcd_count(char, char );
extern void lcd_putsINT(unsigned int);
extern void lcd_cmd(unsigned char);
extern void lcd_data(unsigned char);
extern void lcd_puts(const char * s);
extern void lcd_init();
extern void lcd_strobe();
extern void DelayMs(unsigned int);
extern void DelayUs(unsigned int);

#endif







