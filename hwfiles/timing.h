#ifndef TIMING_H
#define TIMING_H

//#include "lpc17xx_gpio.h"
#include "lpc17xx.h"
#include "lpc17xx_pinsel.h"

extern char hundredhzloopflag, onehzloopflag, fiftyhzloopflag, tenhzloopflag, twentyhzloopflag, fivehzloopflag, IMUhzloopflag;
extern uint32_t millisnow;


void TIMER0_IRQHandler(void);
void boscam_pwm();
void Timer0_init(void);
void timing_init(void);
//void trigger(int timems);
//void TIMER1_IRQHandler(void);
//void Timer1_init();
void TIMER2_IRQHandler(void);
void Timer2_init();
void Timer3_init();
void RIT_IRQHandler(void);
void Rit_init(void);
uint32_t millis();
uint64_t micros();
uint32_t millisbyten();
void timing();


#endif
