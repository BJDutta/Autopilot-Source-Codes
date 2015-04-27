

#include "lpc17xx_gpio.h"
#include "lpc17xx_clkpwr.h"

#include "LED_Blink.h"
#include "timing.h"
#include "lpc17xx_timer.h"

void LED_Setup(){

	LPC_GPIO0->FIODIR |= (1<<18);       // Status LED RED

	LPC_GPIO0->FIOSET = (1<<18);

	LPC_GPIO0->FIODIR |= (1<<17);		// GPS LED  YELLOW

	LPC_GPIO0->FIOSET = (1<<17);

}

void blink(){

	if((1<<18) & LPC_GPIO0->FIOPIN){
		LPC_GPIO0->FIOCLR = (1<<18);
	}
	else{
		LPC_GPIO0->FIOSET = (1<<18);
	}

}

void GPS_blink(){

	if((1<<17) & LPC_GPIO0->FIOPIN){
		LPC_GPIO0->FIOCLR = (1<<17);
	}
	else{
		LPC_GPIO0->FIOSET = (1<<17);
	}

}

void delay(unsigned int timems){

	unsigned int i,j;

	for(i=timems; i>0; i--)
		for(j=10000; j>0; j--) {
		}
}
