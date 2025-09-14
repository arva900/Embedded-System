#include  "../header/halGPIO.h"     // private library - HAL layer

volatile enum FSMstate state;
enum SYSmode lpm_mode;
volatile char sendFRAME[14];
char sendIdx, sendLenth;
volatile char rcvFRAME[3];
char rcvIdx, rcvLenth;
unsigned int echo_cap;
char angle,prev_angle,end_angle;

//--------------------------------------------------------------------
//             System Configuration  
//--------------------------------------------------------------------
void sysConfig(void){ 
	GPIOconfig();
    lcd_init();
    lcd_clear();
	ADCconfig();
	UARTconfig();
}
//---------------------------------------------------------------------
//            Enter from LPM0 mode
//---------------------------------------------------------------------
void enterLPM(unsigned char LPM_level){
	if (LPM_level == 0x00) 
	  _BIS_SR(LPM0_bits+GIE);     /* Enter Low Power Mode 0 */
    else if(LPM_level == 0x01)
	  _BIS_SR(LPM1_bits+GIE);     /* Enter Low Power Mode 1 */
    else if(LPM_level == 0x02)
	  _BIS_SR(LPM2_bits+GIE);     /* Enter Low Power Mode 2 */
	else if(LPM_level == 0x03) 
	  _BIS_SR(LPM3_bits+GIE);     /* Enter Low Power Mode 3 */
    else if(LPM_level == 0x04)
	  _BIS_SR(LPM4_bits+GIE);     /* Enter Low Power Mode 4 */
}
//---------------------------------------------------------------------
//            Enable interrupts
//---------------------------------------------------------------------
void enable_interrupts(void){
  _BIS_SR(GIE);
}
//---------------------------------------------------------------------
//            Disable interrupts
//---------------------------------------------------------------------
void disable_interrupts(void){
  _BIC_SR(GIE);
}
//---------------------------------------------------------------------
//            enable specific interrupts
//---------------------------------------------------------------------
void PB_EN_interrupt(int ena){
    if (ena == 1) PBsArrIntEn |= PB0 +PB1;
    else PBsArrIntEn &= ~(PB0+PB1);
}
char diff;
void MOVEtoANGLE(void){
    if (angle>181) angle=0;
    diff= angle-prev_angle;
    if (diff>80) diff=100; //big angle motor change
    else if(diff>20 && diff<81) diff=40;
    else diff=4;
    TIMER1configSERVO(angle);
    enterLPM(mode0);     // wait diff SERVO period, let him rotate
    TA1CCTL2 &= ~OUTMOD_7 + ~CCIE; //when finished - stop servo PWM
    TA1CTL &= ~MC_1;
    prev_angle=angle;
}

void filesLCDhandle(void){
    if(scroll==0){//choose files
        lcd_clear();
        if(file_count==0) lcd_puts("NO Files");
        else{
            unsigned int i,c=lcd_file;
            for (i=0;i<2&&i<file_count;i++){
                while(File[c].type==2){
                    c++;
                    if (c >9) c = 0;
                }
                lcd_puts("File ");
                lcd_putchar(c+'1');
                lcd_puts("=");
                if (File[c].type=='0') lcd_puts("text");
                else lcd_puts("script");
                lcd_putchar(File[c].num);
                if (i==0) {
                    lcd_new_line;
                    if (File[c].type=='0') lcd_file_ptr = File[c].addr+2;
                }
                c++;
            }
            lcd_file=c-1;
        }
    }else if(scroll==1){//file selected-scrool in file
        if (File[lcd_file].type=='0'){//txt
            unsigned int j;
            lcd_clear();
            for(j=0;j<32;j++){
                if(j==16) lcd_new_line;
                if (*lcd_file_ptr==ACK){
                    lcd_file_ptr = File[lcd_file].addr+2;
                    break;
                }
                lcd_putchar(*lcd_file_ptr);
                if (lcd_file_ptr==end_SEG_1) lcd_file_ptr=SEG_4;
                else lcd_file_ptr++;
            }
        }
    }
    new_file=1;
}
//*****************************PORTS ISR*******************************
//            Port2 Interrupt Service Rotine - pushbutton1
//*********************************************************************
#pragma vector=PORT2_VECTOR
  __interrupt void PBs_handler(void){
    volatile unsigned int i;
    for (i = debounceVal; i > 0; i--) asm(" nop");//debounce delay (inline_opt)
    PBsArrIntEn &= ~(PB0+PB1);
    new_file=2; //dont write flash
    if(PBsArrIntPend & PB0){
        PBsArrIntPend &=~PB0;
        // START ADC - DTC STORE AUTO TO ADCldr[]
        if (state==state6) ADC10CTL0 |= ENC + ADC10SC;
        else if(state==state5) LPM0_EXIT;
    }else if(PBsArrIntPend & PB1){
        PBsArrIntPend &=~PB1;
        if(state==state5){
            if(scroll==0) scroll=1;
            else if(scroll==1){
                scroll=0;
                lcd_file=0;
            }
            LPM0_EXIT;
        }
    }
    PBsArrIntEn |= (PB0+PB1);
  }
//*********************************************************************
//  Timer1_A0 Interrupt Service Routine for core0 - servo period
//*********************************************************************
#pragma vector = TIMER1_A0_VECTOR
__interrupt void TIMER1_A0_ISR(void){}
//*********************************************************************
//  Timer1_A1 Interrupt Service Routine for echo capture(core1) + servo pwm out(core2)
//*********************************************************************
#pragma vector = TIMER1_A1_VECTOR
 __interrupt void TIMER1_A1_ISR(void)
{
  switch(__even_in_range(TA1IV, 0x0A))
  {
      case TA1IV_NONE:  break;              // Vector  0:  No interrupt
      case TA1IV_TACCR1:
          TA1CCTL1 &=~CCIFG;
          if (echo_high){ //rising edge
              TA0CCTL1 &= ~OUTMOD_7; //when ECHO goes UP - stop trigger PWM
              echo_cap = TA1CCR1;
              echo_high=0;
          }else{ //falling edge
              TA1CCTL1 &=~CCIE;
              if (TA1CCR1 >= echo_cap) echo_cap = TA1CCR1 - echo_cap;
              else echo_cap = (0xFFFF - echo_cap) + TA1CCR1; //max echo time = 43msec, MAX ECHO DIST=756cm
              if (echo_cap>0x9836 ||  echo_cap<0xAD) echo_cap=0; //2-450cm
              sendFRAME[0] = (echo_cap >> 8) & 0xFF; // MSB     //mac dist=450 ->26ms echo time ->38966 0x9836
              sendFRAME[1] = echo_cap & 0xFF;        // LSB
          }
          break;              // Vector  2:  TACCR1 CCIFG -servo PWM
      case TA1IV_TACCR2:
          TA1CCTL2 &=~CCIFG;
          diff--;
          if (diff==0) LPM0_EXIT;
          break;              // Vector  4:  TACCR2 CCIFG
      case TA1IV_TAIFG: break;               //timer overflow
      default:  break;
  }
}
//*********************************************************************
//            Timer0_A0 Interrupt Service Routine (for core0)
//*********************************************************************
#pragma vector = TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR(void)
{//run every 1ms
    TA0CCTL0 &=~CCIFG;
    ms_counter++;
    if (ms_counter==X){ //exit just after we made X times the isr
        TA0CCTL0 &= ~CCIE;
        ms_counter=0;
        LPM0_EXIT; //X delay
    }
}
//*********************************************************************
//            Timer0_A1 Interrupt Service Routine (for core1-trigg,core2)
//*********************************************************************
#pragma vector = TIMER0_A1_VECTOR
__interrupt void TIMER0_A1_ISR(void)
{
  switch(__even_in_range(TA0IV, 0x0A))
  {
      case TA0IV_NONE:  break;              // Vector  0:  No interrupt
      case TA0IV_TACCR1: break;              // Vector  2:  TACCR1 CCIFG
      case TA0IV_TACCR2:break;              // Vector  4:  TACCR2 CCIFG
      case TA0IV_TAIFG:
          TA0CTL &= ~(TAIFG +TAIE);
          LPM0_EXIT; //10ms delay
          break;               //timer overflow
      default:  break;
  }
}
//*********************************************************************
//            ADC10 Interrupt Service Routine
//*********************************************************************
#pragma vector = ADC10_VECTOR
__interrupt void ADC10_ISR (void)
{//ONCE dtc finish transfer
   ADC10CTL0 &= ~(ENC + ADC10SC);
   // THE DTC bring A3-A0 so we need only [0],[3]
   int mean_ldr = (ADCldr[0]+ADCldr[3])>>1;//-200 FOR 5V ERROR
   if (mean_ldr<200) mean_ldr=0;
   else mean_ldr-=200;
   sendFRAME[0] = (mean_ldr >> 8) & 0xFF; // MSB
   sendFRAME[1] = mean_ldr & 0xFF;        // LSB
   LPM0_EXIT;
}
//******************************************************************
//              UART Interrupt Service Routine
//******************************************************************
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
    if(state!=state5){
      if (rcvIdx < rcvLenth) rcvFRAME[rcvIdx++] = UCA0RXBUF; // read received char
      if (rcvIdx >= rcvLenth){
            rcvIdx=0;
            LPM0_EXIT;
        }
    }else{
        rcvFRAME[0] = UCA0RXBUF;
        IE2 &= ~UCA0RXIE;
        LPM0_EXIT;
    }
}
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void) {
    if (sendIdx < sendLenth){
        UCA0TXBUF = sendFRAME[sendIdx++];           // Send next char
    }
    if (sendIdx >= sendLenth){
        IE2 &= ~UCA0TXIE;            // Done: disable TX interrupt
        sendIdx = 0;
    }
}
//****************************LCD**********************************
// show int and move back courser
//******************************************************************
int ord;
void lcd_putsINT(unsigned int value) {
    unsigned int digits[3] = {0,0,0};
    unsigned int bases[3]  = {1,10,100};
    int i;
    //delete the previous digits
    for (i = ord; i >0; i--) lcd_cursor_left();
    for (i = ord; i >0; i--) lcd_putchar(' ');
    for (i = ord; i >0; i--) lcd_cursor_left();
    ord=0;
    for (i = 2; i >= 0; i--) {
        while (value >= bases[i]) {
            value -= bases[i];
            digits[i]++;
        }
        if (digits[i]==0 && i>0) asm(" nop"); // Do nothing (suppress leading zeros)
        else {
            lcd_putchar(digits[i] + '0');
            ord++;
        }
    }
}
//**************************************************************
// count up/down/rra
//******************************************************************
void lcd_count(char x, char dec) {
    // dec == 0  count up 0..x
    // dec == 1  count down x..0
    // dec == 2  rra down x..0
    unsigned int i,j;
    if (dec == 0) {
        for (i = 0; i <= x; i++) {
            lcd_clear();
            ord=0;
            lcd_putsINT(i);  // print number on LCD
            j=delay;
            while (j>0){//delay*10ms
                TIMER0config10ms();
                enterLPM(mode0);
                j--;
            }
        }
    } else if(dec == 1) {
        for (i = x+1; i > 0; i--) {
            lcd_clear();
            ord=0;
            lcd_putsINT(i-1);  // print number on LCD
            j=delay;
            while (j>0){//delay*10ms
                TIMER0config10ms();
                enterLPM(mode0);
                j--;
            }
        }
    }else if(dec == 2){
        int c;
        for (i = 0; i<32 ; i++) {
            c=0;
            lcd_clear();
            while (c!=i){
               if (c==16) lcd_new_line;
               else lcd_cursor_right();
               c++;
            }
            lcd_putchar(x);  // print number on LCD
            j=delay;
            while (j>0){//delay*10ms
                TIMER0config10ms();
                enterLPM(mode0);
                j--;
            }
        }
   }
}
//******************************************************************
// send a command to the LCD
//******************************************************************
void lcd_cmd(unsigned char c){

  LCD_WAIT; // may check LCD busy flag, or just delay a little, depending on lcd.h

  if (LCD_MODE == FOURBIT_MODE)
  {
      LCD_DATA_WRITE &= ~OUTPUT_DATA;// clear bits before new write
      LCD_DATA_WRITE |= ((c >> 4) & 0x0F) << LCD_DATA_OFFSET;
      lcd_strobe();
      LCD_DATA_WRITE &= ~OUTPUT_DATA;
      LCD_DATA_WRITE |= (c & (0x0F)) << LCD_DATA_OFFSET;
      lcd_strobe();
  }
  else
  {
      LCD_DATA_WRITE = c;
      lcd_strobe();
  }

}
//******************************************************************
// send data to the LCD
//******************************************************************
void lcd_data(unsigned char c){

  LCD_WAIT; // may check LCD busy flag, or just delay a little, depending on lcd.h
  LCD_DATA_WRITE &= ~OUTPUT_DATA;
  LCD_RS(1);
  if (LCD_MODE == FOURBIT_MODE)
  {
      LCD_DATA_WRITE &= ~OUTPUT_DATA;
      LCD_DATA_WRITE |= ((c >> 4) & 0x0F) << LCD_DATA_OFFSET;
      lcd_strobe();
//*TRY WITHOUT THIS LINE* -> //
      LCD_DATA_WRITE &= (0xF0 << LCD_DATA_OFFSET) | (0xF0 >> 8 - LCD_DATA_OFFSET);
      LCD_DATA_WRITE &= ~OUTPUT_DATA;
      LCD_DATA_WRITE |= (c & 0x0F) << LCD_DATA_OFFSET;
      lcd_strobe();
  }
  else
  {
      LCD_DATA_WRITE = c;
      lcd_strobe();
  }
  LCD_RS(0);
}
//******************************************************************
// write a string of chars to the LCD
//******************************************************************
void lcd_puts(const char * s){
  while(*s)
      lcd_data(*s++);
}
//******************************************************************
// initialize the LCD
//******************************************************************
void lcd_init(){
    char init_value;

    if (LCD_MODE == FOURBIT_MODE)
        init_value = 0x3 << LCD_DATA_OFFSET;
    else
        init_value = 0x3F;

    LCD_RS_DIR(OUTPUT_PIN); //p2.5-7dir=1 ->output pins
    LCD_EN_DIR(OUTPUT_PIN);
    LCD_RW_DIR(OUTPUT_PIN);
    LCD_DATA_DIR |= OUTPUT_DATA;
    LCD_RS(0);
    LCD_EN(0);
    LCD_RW(0);

    DelayMs(15);
    LCD_DATA_WRITE &= ~OUTPUT_DATA;
    LCD_DATA_WRITE |= init_value;
    lcd_strobe();
    DelayMs(5);
    LCD_DATA_WRITE &= ~OUTPUT_DATA;
    LCD_DATA_WRITE |= init_value;
    lcd_strobe();
    DelayUs(200);
    LCD_DATA_WRITE &= ~OUTPUT_DATA;
    LCD_DATA_WRITE |= init_value;
    lcd_strobe();


    if (LCD_MODE == FOURBIT_MODE){
      LCD_WAIT; // may check LCD busy flag, or just delay a little, depending on lcd.h
      LCD_DATA_WRITE &= ~OUTPUT_DATA;
      LCD_DATA_WRITE |= 0x2 << LCD_DATA_OFFSET; // Set 4-bit mode
      lcd_strobe();
      lcd_cmd(0x28); // Function Set
    }
    else lcd_cmd(0x3C); // 8bit,two lines,5x10 dots

    lcd_cmd(0xF); //Display On, Cursor On, Cursor Blink
    lcd_cmd(0x1); //Display Clear
    lcd_cmd(0x6); //Entry Mode
    lcd_cmd(0x80); //Initialize DDRAM address to zero
}
//******************************************************************
// lcd strobe functions
//******************************************************************
void lcd_strobe(){
  LCD_EN(1);
  asm(" nop");
  asm(" nop");
  LCD_EN(0);
}
//--------------------------------------------------------------------
//              LCD Delay usec functions
//--------------------------------------------------------------------
void DelayUs(unsigned int cnt){
    unsigned char i;
    for(i=cnt ; i>0 ; i--) asm(" nop"); // tha command asm("nop") takes raphly 1usec
}

//--------------------------------------------------------------------
//              LCD Delay msec functions
//--------------------------------------------------------------------
void DelayMs(unsigned int cnt){
    unsigned char i;
    for(i=cnt ; i>0 ; i--) DelayUs(1000); // tha command asm("nop") takes raphly 1usec
}
