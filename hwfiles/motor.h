#ifndef MOTOR_H
#define MOTOR_H

#include "lpc17xx_pwm.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_libcfg_default.h"
#include "lpc17xx_timer.h"
#include "LED_Blink.h"
#include "lpc17xx_exti.h"
#include "control.h"
#include "config.h"

//#define RCB7X


typedef struct {
	int16_t s1;
	int16_t s2;
	int16_t s3;
	int16_t s4;
	int16_t s5;
	int16_t s6;
}motors_t;

#define NUM_CHANNELS 8

extern motors_t motors;

extern int16_t channel[NUM_CHANNELS];

void PWM_setup(void);
void Motors_set();
void PPM_setup();











#endif
