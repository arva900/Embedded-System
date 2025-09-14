#include  "../header/api.h"    		// private library - API layer

int X,delay;
unsigned int ADCldr[4];
FileManagement File[10];
unsigned int file_count,file_idx,new_file;
volatile char lcd_file,scroll;
volatile char *lcd_file_ptr;

void RXbytes(char bytes){
    rcvLenth = bytes + 1;
    char checksum=1;
    int i;
    // 2-way handshake:
    rcvFRAME[0]=0xF0;
    rcvIdx=0;
    IE2 |= UCA0RXIE;
    while (checksum!=0 && bytes>0){
        _BIS_SR(LPM0_bits+GIE); // finish get all RX frame
        checksum=0;
        for (i=bytes ; i>=0 ; i--) checksum += rcvFRAME[i];
    }
    sendFRAME[0] = ACK;
    sendLenth = 1;
    IE2 |= UCA0TXIE;      // Enable TX interrupt

}

void TXbytes(char bytes){
    sendLenth = bytes+1; //plus checksum
    char checksum=0;
    unsigned int i;
    for (i=0; i<bytes ; i++) checksum += sendFRAME[i];
    sendFRAME[bytes] = -checksum;
    // 2-way handshake:
    rcvFRAME[0] =rcvIdx= 0;
    rcvLenth = 1;
    IE2 |= UCA0RXIE+UCA0TXIE;  // Enable TX interrupt - dont in loop it will junk the buffer
    i=0;
    while (rcvFRAME[0] < EXIT && i<500){ //if we dont got ack
//        if (i==100) IE2 |= UCA0TXIE;
        TIMER0config10ms();
        _BIS_SR(LPM0_bits+GIE);       // wait 10ms time_out
        i++;
    }
    if (i>=500 && rcvFRAME[0] < EXIT ){
        //if (state!= state7)
        state=state0;
    }
    TA0CTL = TACLR;
}

void state0API(void){
    X=60;
    sendIdx=rcvIdx=angle=0;
    prev_angle=end_angle=180;
    lcd_clear();
    lcd_puts("idle. wait RX");
    MOVEtoANGLE();
    TA1CTL = TA0CTL=TACLR;
    RXbytes(1);
    switch(rcvFRAME[0])
    {
     case 1: state = state1; break;
     case 2: state = state2; break;
     case 3: state = state3; break;
     case 4: state = state4; break;
     case 5:
         file_count=0;
         if (start==1) start=0; setFILES(File);//READ SEGMENT_C
         File[0].addr=SEG_4;
         state = state5; break;
     case 6: state = state6; break;
     case 7: state = state7; break;
     default:  state = state0; break;
    }
}


void state1API(void){ //object detector
    X=100;           //60ms delay for max echo time
    lcd_clear();
    lcd_puts("angle = ");
    int servo_deg=0;
    while (state == state1 || (state == state7 && angle<=end_angle)){
        MOVEtoANGLE();
        TIMER1configECHO();
        TIMER0config1ms();       //start timer0 + pwm trigg
        TIMER0configTRIG();
        lcd_putsINT(angle);
        enterLPM(mode0);         // wait Xms for echo to finish
        angle++;
        if (state == state7 && *lcd_file_ptr == 6 && servo_deg<100) angle--; servo_deg++;
        if (state == state7 && *lcd_file_ptr == 6) sendFRAME[2] = angle;
        else sendFRAME[2] = angle-1;
        TXbytes(3);
        if (rcvFRAME[0]==EXIT) state=state0;
    }
}


void state2API(void){
    X=100; //30ms delay for max echo time
    lcd_clear();
    lcd_puts("angle = ");
    while (state == state2 ){
        MOVEtoANGLE();
        TIMER1configECHO();
        TIMER0configTRIG();
        TIMER0config1ms();       //start timer0 + pwm trigg
        lcd_putsINT(angle);
        enterLPM(mode0);         // wait 30ms for echo to finish
        //sendFRAME[2] = angle;
        TXbytes(3);
        if (rcvFRAME[0]==EXIT){
            state=state0;
            break;
        }
        RXbytes(1) ;//get user angle
        angle=rcvFRAME[0];
    }
}

void state3API(void){
    sendADCcalib();
    X=30; //30ms delay for max echo time
    lcd_clear();
    lcd_puts("angle = ");
    while (state == state3){
        ADC10SA = (unsigned int)ADCldr; //give the adress ADCldr[0] = ldr2,[3]=ldr1
        TIMER0config1ms();       //start timer0 for Xms
        sendFRAME[2] = angle;
        lcd_putsINT(angle);
        ADC10CTL0 |= ENC + ADC10SC;
        enterLPM(mode0);      //  adc to finish
        angle++;
        MOVEtoANGLE();
        TXbytes(3);
        if (rcvFRAME[0]==EXIT) state=state0;
    }
}

void state4API(void){
    sendADCcalib();
    X=60; //30ms delay for max echo time
    lcd_clear();
    lcd_puts("angle = ");
    while (state == state4){
        MOVEtoANGLE();
        ADC10SA = (unsigned int)ADCldr; //give the adress ADCldr[0] = ldr2,[3]=ldr1
        TIMER0config1ms();
        sendFRAME[2] = angle;
        lcd_putsINT(angle);
        ADC10CTL0 |= ENC + ADC10SC;
        enterLPM(mode0);      //adc to finish
        TXbytes(3);           //send adc
        TIMER1configECHO();
        TIMER0config1ms();       //start timer0 + pwm trigg
        TIMER0configTRIG();
        enterLPM(mode0);         // wait Xms for echo to finish
        TXbytes(3);
        if (rcvFRAME[0]==EXIT) state=state0;
        angle++;
    }
}

void state5API(void){
    //resetFILES();//must delete segments before
    lcd_file=scroll=0;
    new_file=1;
    lcd_clear();
    while (state==state5){
        if(new_file>=1) filesLCDhandle();
        IE2 |= UCA0RXIE;
        enterLPM(mode0);  //Receive byte
        if (new_file==1 && rcvFRAME[0]==ACK) {
            state=state0;
            break; //exit state5
        }
        else if(new_file<2) writeFILEbyte();
    }
}

void writeFILEbyte(void){
    char *write_adr=File[file_idx].addr+File[file_idx].byte_len;
    if(write_adr>end_SEG_1) write_adr = SEG_4;
    if (*write_adr!=0xFF){
        unsigned int i,dist;
        for(i=0;i<10;i++){
            dist=(unsigned int)(File[i].addr-write_adr);
            if (dist<MAIN_SIZE && File[i].type != 2){
                File[i].type=2;   //to big - delete next file
                File[i].byte_len=0;
                File[i].addr=end_SEG_1;
                file_count--;
            }
        }
        FLASHeraseSEG((unsigned int*)write_adr);
    }
    FLASHwriteBYTE(write_adr,rcvFRAME[0]);
    File[file_idx].byte_len++;
    new_file=0;
    if(rcvFRAME[0]==ACK && File[file_idx].byte_len>0 ){
        File[file_idx].type= *File[file_idx].addr;//little Indian
        File[file_idx].num = *(File[file_idx].addr+1);
        if (file_idx<9){
            if (File[file_idx+1].byte_len==0) File[file_idx+1].addr=write_adr+1;
        }else{//round rubin delete first file
            File[0].type=2;
            File[0].byte_len=0;
            file_count--;
            file_idx=-1;
        }
        sendFRAME[0]=ACK; //send ACK to pc
        sendIdx=0;
        sendLenth = 1;
        IE2 |= UCA0TXIE;
        if (File[file_idx].type=='1'){
            state=state7;
            lcd_file_ptr=File[file_idx].addr+2;
        }
        file_count++;
        file_idx++;
        new_file=1;
    }
}
void setFILES(FileManagement *ram_ptr){//READ/WRITE SEGMENT_C
    char byte_count=0,i;
    for (i = 0; i < 10; i++) {
        // --- Read type ---
        ram_ptr[i].type = SEG_C[byte_count++];
        // --- Read num ---
        ram_ptr[i].num = SEG_C[byte_count++];
        // --- Read addr (2 bytes, little endian) ---
        unsigned int addr_low  = SEG_C[byte_count++];
        unsigned int addr_high = SEG_C[byte_count++];
        ram_ptr[i].addr = (char*)((addr_high << 8) | addr_low);
        // --- Read byte_len (2 bytes, little endian) ---
        unsigned int len_low  = SEG_C[byte_count++];
        unsigned int len_high = SEG_C[byte_count++];
        ram_ptr[i].byte_len = (len_high << 8) | len_low;
        // Count valid files
        if (ram_ptr[i].byte_len > 0) file_count++;
    }
    file_idx=file_count;
}
void state7API(void){
    delay = 50; //500ms
    while (state==state7){
        sendFRAME[0] = lcd_file_ptr[0];
        sendFRAME[1] = lcd_file_ptr[1];
        sendFRAME[2] = lcd_file_ptr[2];
        TXbytes(3);
        switch(*lcd_file_ptr) {
         case 1:
             lcd_count(lcd_file_ptr[1],0);//inc
             break;
         case 2:
             lcd_count(lcd_file_ptr[1],1);//dec
             break;
         case 3:
             lcd_count(lcd_file_ptr[1],2);//rra
             break;
         case 4:
             delay=lcd_file_ptr[1];
             break;
         case 5:
             lcd_clear();
             lcd_file_ptr--;
             break;
         case 6:
             angle=end_angle=lcd_file_ptr[1];
             state1API();
             break;
         case 7:
             angle = lcd_file_ptr[1];
             end_angle = lcd_file_ptr[2];
             state1API();
             lcd_file_ptr++;
             break;
         default: state = state5;  break;
        }
        lcd_file_ptr+=2;
    }
}
void state6API(void){ //light calibrate
    FLASHeraseSEG(SEG_D); //save the calib right in FLASH SEG_D
    angle=90;
    MOVEtoANGLE();
    lcd_clear();
    lcd_puts("  cm.  PB0-Save");
    lcd_new_line;
    lcd_puts("       PB1-Exit");
    lcd_home();
    unsigned int dist,i=0;
    for (dist = 0; dist <= 45; dist += 5) {
        ADC10SA = (unsigned int)ADCldr; //give the adress ADCldr[0] = ldr2,[3]=ldr1
        lcd_putsINT(dist);
        enterLPM(mode0);// wait for user press
        //after press, save the int in flash
        FLASHwriteWORD(&SEG_D[i], (sendFRAME[0]<<8) | sendFRAME[1]);
        i++;
    }
    state=state0;
}

void sendADCcalib(void){
    unsigned int i,mask= 0x0300;
    //compressing
    for (i=0;i<10;i++){//10 LSB of each dist
        sendFRAME[i] = SEG_D[i]; // give value SEG_D[i]==*(SEG_D+i*2)
    }
    char shft[4] = {2,4,6,8};
    sendFRAME[10] = sendFRAME[11] = 0;
    //2msb of each dist connected
    for (i=0;i<4;i++){
        sendFRAME[10]|=((SEG_D[i]&mask)>>shft[i]);
        sendFRAME[11]|=((SEG_D[i+4]&mask)>>shft[i]);
    }
    sendFRAME[12]=((SEG_D[8]&mask)>>2) | ((SEG_D[9]&mask)>>4) ;
    TXbytes(13);
}


void resetFILES(void){
    //seg1-4 resets every time msp resets..
//    FLASHeraseSEG(SEG_4);
//    FLASHeraseSEG(SEG_3);
//    FLASHeraseSEG(SEG_2);
//    FLASHeraseSEG(SEG_1);
    FLASHeraseSEG((unsigned int*)SEG_C);
    //upto 10 files =6*10=60bytes - SEG_C
    unsigned int file_count=0,word_count=0;
    while (file_count<10){
        FLASHwriteWORD((unsigned int*)SEG_C+word_count,2);//save type2-deleted
        word_count++;//name+type size

        FLASHwriteWORD((unsigned int*)SEG_C+word_count,(unsigned int)end_SEG_1-10+file_count);//save start address(init)
        word_count++;//address size

        FLASHwriteWORD((unsigned int*)SEG_C+word_count,0);//save length=0
        word_count++;//length size
        file_count++;
    }
}

