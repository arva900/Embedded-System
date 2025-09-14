#ifndef _bsp_H_
#define _bsp_H_

#include  <msp430g2553.h>          // MSP430x2xx


#define   debounceVal      400

//--------------------- LCD abstraction ----------------/
#ifdef CHECKBUSY
    #define LCD_WAIT lcd_check_busy()
#else
    #define LCD_WAIT DelayMs(20) //*MAYBE CHANGE TO (20) FOR THE BUG*
#endif

#define LCD_EN(a)   (!a ? (P2OUT&=~0X08) : (P2OUT|=0X08)) // P2.3 is lcd enable pin v
#define LCD_EN_DIR(a)   (!a ? (P2DIR&=~0X08) : (P2DIR|=0X08)) // P2.3 pin direction v
// RS -register select
//rs=1 - DATA, rs=0 - COMMAND
#define LCD_RS(a)   (!a ? (P2OUT&=~0X10) : (P2OUT|=0X10)) // P2.4 is lcd RS pin v
#define LCD_RS_DIR(a)   (!a ? (P2DIR&=~0X10) : (P2DIR|=0X10)) // P2.4 pin direction v

#define LCD_RW(a)   (!a ? (P2OUT&=~0X80) : (P2OUT|=0X80)) // P2.7 is lcd RW pin v
#define LCD_RW_DIR(a)   (!a ? (P2DIR&=~0X80) : (P2DIR|=0X80)) // P2.7 pin direction v

#define LCD_DATA_OFFSET 4 //data pin selection offset for 4 bit mode, variable range is 0-4, default 0 - P1.4-7, no offset
#define LCD_DATA_WRITE  P1OUT
#define LCD_DATA_DIR    P1DIR
#define LCD_DATA_SEL    P1SEL//functionality
#define LCD_CTRL_SEL    P2SEL // for control

#define FOURBIT_MODE    0x0
#define EIGHTBIT_MODE   0x1
#define LCD_MODE        FOURBIT_MODE // always
#define OUTPUT_PIN      1
#define INPUT_PIN       0
#define OUTPUT_DATA     (LCD_MODE ? 0xFF : (0x0F << LCD_DATA_OFFSET))//0xF0)//
#define INPUT_DATA      0x00


#define lcd_cursor(x)       lcd_cmd(((x)&0x7F)|0x80)
#define lcd_clear()     lcd_cmd(0x01) // clear all lcd
#define lcd_putchar(x)      lcd_data(x) // send ascii to lcd
#define lcd_goto(x)     lcd_cmd(0x80+(x)) //
#define lcd_cursor_right()  lcd_cmd(0x14) // one space to right
#define lcd_cursor_left()   lcd_cmd(0x10) // one space to left
#define lcd_display_shift() lcd_cmd(0x1C) // off curser and space left??
#define lcd_home()      lcd_cmd(0x02)  //bring to first line
#define cursor_off              lcd_cmd(0x0C)
#define cursor_on               lcd_cmd(0x0F)
#define lcd_function_set        lcd_cmd(0x3C) // 8bit,two lines,5x10 dots
#define lcd_new_line            lcd_cmd(0xC0)      // new line -second line
//--------------------- end of LCD abstraction ----------------//

// PushButtons abstraction
#define PBsArrPort        P2IN
#define PBsArrIntPend      P2IFG
#define PBsArrIntEn        P2IE
#define PBsArrIntEdgeSel   P2IES
#define PBsArrPortSel      P2SEL
#define PBsArrPortDir      P2DIR
#define PB0                0x01 //P2.0
#define PB1                0x02 //P2.1

//UART
#define  RXD    0x02    //P1.1 is rx
#define  TXD    0x04    //P1.2 is tx

//SERVO motor
#define  SERVO       0x20 //p2.5
#define  SERVOdir    P2DIR
#define  SERVOsel    P2SEL

//ULTRAsonic sensor
//TRIGER
#define  ULTRIG       0x40 //p2.6
#define  ULTRIGdir    P2DIR
#define  ULTRIGsel    P2SEL
//ECHO
#define  ULECHO       0x04 //p2.2
#define  ULECHOdir    P2DIR
#define  ULECHOsel    P2SEL

//LDR sensors
//left sensor
#define  LDR1       0x01 //p1.0 - Y=0 LDR1
//right sensor
#define  LDR2       0x08 //p1.3 - Y=3 LDR2

// FLASH - INFO
#define SEG_D ((unsigned int*)0x1000)  //pointer to Flash segment D -for ADC calib
#define SEG_C ((char*)0x1040)  //pointer to Flash segment C -for FILE MENAGE
#define INFO_SIZE  64 //each segment 64 bytes
// FLASH - MAIN
#define SEG_4 ((char*)0xF600)
#define SEG_3 ((char*)0xF800)
#define SEG_2 ((char*)0xFA00)
#define SEG_1 ((char*)0xFC00)
#define end_SEG_1 ((char *)0xFDFF)
#define MAIN_SIZE  512 //each segment 512 bytes


//      Timer0_A      ///
//  TA0CTL - TIMER0_A control
//**CORE0** for 1ms delay
//  TA0CCTL0 - control to timer 0 core 0
//  TA0CCR0  - timer 0 core 0 REG
//**CORE1** FOR sensor TRIGG - P2.6(TA1.1 - PWMOUT) trigg pulse =10usec

//          Timer1_A        ///
//   TA1CTL TIMER1_A control
//**CORE0** for 20msec min servo period:
//   TA1CCTL0 control to core 0
//   TA1CCR0  core 0 REG
#define SMCLK_DIV8          1500000       // SMCLK/8 = 1.5 MHz
#define SERVO_FREQ          50         // 20 ms period
#define SERVO_T_TICKS       (unsigned int)(SMCLK_DIV8 / SERVO_FREQ) // 1.5M / 50 = 30000

//**CORE2** FOR SERVO dutycycle - P2.5(TA1.2 - PWMOUT):
//   TA1CCTL2 control to core 2
//   TA1CCR2 core2 compare/capture REG
#define SERVO_START_TICKS  (unsigned int)(SMCLK_DIV8 * 0.0005)-1 //0.5ms for angle 0
#define SERVO_END_TICKS    (unsigned int)(SMCLK_DIV8 * 0.002)-1 //2ms for angle 180

//**CORE1** FOR sensor ECHO - P2.2(CCI1B - capture)
//   TA1CCTL1 control to core 2
//    TA1CCR1 core 2 REG

extern void GPIOconfig(void);
extern volatile unsigned int ms_counter;
extern void TIMER0config1ms(void);
extern void TIMER0config10ms(void);
extern volatile char echo_high;
extern void TIMER0configTRIG(void);
extern void TIMER1configSERVO(char ang);
extern void TIMER1configECHO(void);
extern void ADCconfig(void);
extern void UARTconfig(void);
void FLASHeraseSEG(unsigned int*);
void FLASHwriteWORD(unsigned int*, unsigned int);
void FLASHwriteBYTE(char* , char );

#endif



