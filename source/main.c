#include  "../header/app.h"    		// private library - APP layer
char start=1;
void main(void){
  state = state0;  // start in idle state on RESET
  sysConfig();
  while(1){
	switch(state){
        case state0:
            enable_interrupts();
            state0API();
            break;
        case state1:
            state1API();
            break;
        case state2:
            state2API();
            break;
        case state3:
            state3API();
            break;
        case state4:
            state4API();
            break;
        case state5:
            PB_EN_interrupt(1);
            state5API();
            PB_EN_interrupt(0);
            break;
        case state6:
            PB_EN_interrupt(1);
            state6API();
            PB_EN_interrupt(0);
            break;
        case state7:
            state7API();
            break;
	}
  }
}
  
  
  
  
  
  
