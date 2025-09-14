#ifndef _api_H_
#define _api_H_

#include  "../header/halGPIO.h"     // private library - HAL layer
extern int X,delay;
extern unsigned int ADCldr[4];
extern unsigned int file_count,file_idx,new_file;
typedef struct {
    char type; // 0-txt,1-script, 2-raced
    char num; //number of the file=1-10 :"script1" - typ=1,num=1
    char *addr; //start address pointer
    unsigned int byte_len; // file length in bytes
} FileManagement ; //6bytes -3words
extern FileManagement File[10];
extern volatile char lcd_file,scroll;
extern volatile char *lcd_file_ptr;

void RXbytes(char);
void TXbytes(char);
void sendADCcalib(void);
extern void state0API(void);
extern void state1API(void);
extern void state2API(void);
extern void state3API(void);
extern void state4API(void);
extern void state5API(void);
void writeFILEbyte(void);
void setFILES(FileManagement * );
extern void state6API(void);

extern void state4API(void);
extern void state7API(void);

void resetFILES(void);

#endif







