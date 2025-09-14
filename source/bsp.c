#include "../header/bsp_430x2xx.h" // private library - BSP layer for 2
//-----------------------------------------------------------------------------  
//           GPIO configuration
//-----------------------------------------------------------------------------
void GPIOconfig(void){
    WDTCTL = WDTHOLD | WDTPW + WDTNMI + WDTNMIES;  // WDT off NMI hi/lo;
    IE1 |= NMIIE;                        // Enable NMI

    // LCD setup
    LCD_DATA_WRITE &= ~0xF0;    // Clear data
    LCD_DATA_SEL   &= ~0xF0;   // I\O P1.4-P1.7 - DATA bits
    LCD_DATA_DIR   |=  0xF0;    // P1.4-P1.7 To Output
    LCD_CTRL_SEL   &= ~0x98;   // I\O P2.3,4,7 - CTRL bits


    PBsArrPortSel &= ~(PB1+PB0);
    PBsArrPortDir &= ~(PB1+PB0);       //P2.0-1 input
    // PushButtons Setup
    P2REN |= PB1+PB0; // Enable pull-up/down resistors for P2.0-1
    // Select pull-up for P1.0 (PB1)
    P2OUT |= PB1+PB0 ; // P2OUT=1 means pull-up when REN=1
    PBsArrIntEdgeSel &= ~(PB1+PB0);          // pull-up mode -rising edge
    //PBsArrIntEn |= PB0+PB1;               //interrupts from PB0 enable when entering state
    PBsArrIntPend &= ~(PB1+PB0);            // clear pending interrupts

    //Important!!!
    //servo pwm out should trigger also the ldr conversion operation and the ultrasonic trigger pin
    //
    //SERVO setup
    SERVOdir |= SERVO;    //P2.4 set to PWMOUT from timera1 core2 in compare mode - TA1.2
    SERVOsel |= SERVO ;
    P2SEL2   &= ~SERVO;

    // ULTRAsonic sensor setup
    //trigger -
    ULTRIGdir |= ULTRIG;    //P2.6 set to PWMOUT from timera1 core0 in compare mode - TA1.0
    ULTRIGsel |= ULTRIG ;
    P2SEL2    &= ~ULTRIG;
    //echo -
    ULECHOdir &= ~ULECHO;    //P2.2 set to input capture from timera1 core1 in capture mode - CCI1A
    ULECHOsel |= ULECHO ;
    P2SEL2    &= ~ULECHO;

    _BIS_SR(GIE);                     // enable interrupts globally
}                             
//------------------------------------------------------------------------------------- 
//            Timers configuration - FOR 1ms delay and BUZZ PWMOUT
//-------------------------------------------------------------------------------------
volatile unsigned int ms_counter;
void TIMER0config1ms(void){
    //Timer0 setup for 1ms delay
    TA0CTL = TACLR;                   // CLR Bit
    TA0CCR0 = 12000-1;                // set TACCR0 to get 1msec on smclk=12mhz
    TA0CCTL0 = CCIE;
    TA0CTL = TASSEL_2 + MC_1;         // SMCLK, up mode
    ms_counter=0;
}
void TIMER0config10ms(void){
    //Timer0 setup for 1ms delay
    TA0CTL = TACLR;                   // CLR Bit
    TA0CCR0 = 15000;                // set TACCR0 to get ~10msec on smclk=12mhz
    TA0CTL = TASSEL_2 + MC_1+ID_3+TAIE;    // SMCLK/8, up mode
}
volatile char echo_high;
void TIMER0configTRIG(void){
    TA0CCR1 = 120-1; //for trigg 10us
    TA0CCTL1 = OUTMOD_7;
    echo_high = 1;
}

void TIMER1configSERVO(char ang){
    //timer1 setup for servo and echo
    TA1CTL = TACLR;             // CLR Bit
    TA1CCR0 = SERVO_T_TICKS;       // value for 20 msec - servo min cycle
    TA1CCR2 = SERVO_START_TICKS + (ang<<4)-(ang<<1); //749 + 12.5*angle
    TA1CCTL2 = OUTMOD_7+ CCIE; //Move to the new angle
    TA1CCTL1 = 0;  //dis echo
    TA1CTL = TASSEL_2 + ID_3 + MC_1;   // SMCLK/8, up mode
}
void TIMER1configECHO(void){
    //timer1 setup for servo and echo
    TA1CTL = TACLR;             // CLR Bit
    TA1CCR0 = 0xFFFF;
    TA1CCTL1 = CM_3 + CCIS_1 + SCS + CAP +CCIE;  //echo - both edges. CCI1B - P2.2, sync cap, cap, int ena
    TA1CTL = TASSEL_2 + ID_3 +MC_1 ;//+TAIE;   // SMCLK/8, up mode
}
//-------------------------------------------------------------------------------------
//            ADC configuration - LDR sensors
//-------------------------------------------------------------------------------------
void ADCconfig(void){
  /* Sequence-of-Channels Mode start from A3 DOWN TO A0
     *with MSC=1 - ONE TIGGER FROM PB0  WILL START the opartion
     * we take only A3 and A0*/

// 64*ADCLK, Multiple SampleConversion, set ref to Vcc and Gnd, Turn on ADC and Enable Interrupt
  ADC10CTL0 = ADC10SHT_3+ MSC  + SREF_0 + ADC10ON + ADC10IE ;
  ADC10CTL1 = INCH_3 + CONSEQ_1 + ADC10SSEL_3;     // Input start at A3, Sequence-of-Channels, ADCclk is SMCLK=12Mhz
  ADC10AE0  = LDR1 + LDR2;            // P1.0,3 ADC option select A0,3
  ///DTC USAGE
  ADC10DTC1 = 4; //4 TRANSFERS one for each
}
//-------------------------------------------------------------------------------------
//            UART0 configuration
//-------------------------------------------------------------------------------------
void UARTconfig(void){
  if (CALBC1_12MHZ==0xFF)                    // If calibration constant erased
  {
    while(1);                               // do not load, trap CPU!!
  }
  // Set DCO to 12 MHz for uart we want precise freq
  BCSCTL1 = CALBC1_12MHZ;
  DCOCTL  = CALDCO_12MHZ;
  //SET P1.1-2 to uart mode
  P1SEL = RXD + TXD;                     // P1.1 = RXD, P1.2=TXD
  P1SEL2 =RXD + TXD;                    // P1.1 = RXD, P1.2=TXD
  //UCAI register setup
  UCA0CTL1 = UCSWRST;           //RST UART CTRL REG
  // UART config
  UCA0CTL1 |= UCSSEL_2;              // Use SMCLK (12 MHz) to get baud rate of 9600
  //12Mhz/9600 = 1250.0 - no modulation
  UCA0BR0 = 1250 & 0xFF;             // Low byte of 1250 0xE2
  UCA0BR1 = 1250 >> 8;               // High byte of 1250 0x04
  UCA0MCTL = 0;                      // No modulation (UCBRSx = 0, UCBRFx = 0)
  UCA0CTL1 &= ~UCSWRST;          // start uart- **Initialize USCI state machine**
  IE2 |= UCA0RXIE;               // Enable USCI_A0 RX interrupt
  IE2 &= ~UCA0TXIE;
}
void FLASHeraseSEG(unsigned int* SEG){
    _BIC_SR(GIE);
    FCTL2 = FWKEY + FSSEL_1 + 25; //MCLK/25 ~461Khz
    FCTL3 = FWKEY;             // unlock flash
    FCTL1 = FWKEY + ERASE;     // erase mode
    *SEG = 0;                // dummy write triggers erase
    FCTL1 = FWKEY;             // cancel erase
    FCTL3 = FWKEY + LOCK;      // lock flash
    _BIS_SR(GIE);
}

void FLASHwriteWORD(unsigned int* addr, unsigned int value) {
    _BIC_SR(GIE);
    FCTL2 = FWKEY + FSSEL_1 + 25; //MCLK/26 ~461Khz
    FCTL3 = FWKEY;             // unlock flash
    FCTL1 = FWKEY + WRT;       // enable write
    *addr = value;              // write word
    FCTL1 = FWKEY;             // disable write
    FCTL3 = FWKEY + LOCK;      // lock flash
    _BIS_SR(GIE);
}

void FLASHwriteBYTE(char* addr, char value) {
    _BIC_SR(GIE);
    FCTL2 = FWKEY + FSSEL_1 + 25; //MCLK/26 ~461Khz
    FCTL3 = FWKEY;             // unlock flash
    FCTL1 = FWKEY + WRT;       // enable write
    *addr = value;              // write word
    FCTL1 = FWKEY;             // disable write
    FCTL3 = FWKEY + LOCK;      // lock flash
    _BIS_SR(GIE);
}

             
             
            
  

