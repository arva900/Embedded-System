#ifndef _app_H_
#define _app_H_


#include  "../header/api.h"         // private library - API layer
// Define enum types here:
enum FSMstate{state0,state1,state2,state3,state4,state5,state6,state7,state8}; // global variable
enum SYSmode{mode0,mode1,mode2,mode3,mode4}; // global variable
extern volatile enum FSMstate state;   // global variable
extern enum SYSmode lpm_mode; // global variable
extern char start;
#endif







