#include "motor.h"



#define _EXT_IRQH	EINT3_IRQn
#define _EXTINTH		EXTI_EINT3

#define _EXT_IRQL	EINT1_IRQn
#define _EXTINTL		EXTI_EINT1



motors_t motors;

uint32_t lowus=0, highus=0;
int16_t channel[NUM_CHANNELS];
uint8_t  channel_index=0;

void PWM_setup(void)
{
	uint8_t temp;
	PWM_TIMERCFG_Type PWMCfgDat;
	PWM_MATCHCFG_Type PWMMatchCfgDat;
	PINSEL_CFG_Type PinCfg;

	// intialize motors to zero before powering motors

	motors.s1 = motors.s2 = motors.s3 = motors.s4 = 0;

#ifdef HEXA_X
	motors.s5 = motors.s6 = 0;
#endif


	/* PWM block section -------------------------------------------- */

	/* Initialize PWM peripheral, timer mode
	 * PWM prescale value = 1 (absolute value - tick value)
	 */
	PWMCfgDat.PrescaleOption = PWM_TIMER_PRESCALE_USVAL;
	PWMCfgDat.PrescaleValue = 1;
	PWM_Init(LPC_PWM1, PWM_MODE_TIMER, (void *) &PWMCfgDat);

	/*
	 * Initialize PWM pin connect
	 */
	//	PinCfg.Funcnum = 1;		// for old board
	//	PinCfg.OpenDrain = 0;
	//	PinCfg.Pinmode = 0;
	//	PinCfg.Portnum = 2;
	//	for (temp = 2; temp <= 5; temp++)
	//	{
	//		PinCfg.Pinnum = temp;
	//		PINSEL_ConfigPin(&PinCfg);
	//	}

	PinCfg.Funcnum = 2;		// pin config for motors pwm
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 1;

	PinCfg.Pinnum = 21;
	PINSEL_ConfigPin(&PinCfg);

	PinCfg.Pinnum = 23;
	PINSEL_ConfigPin(&PinCfg);

	PinCfg.Pinnum = 24;
	PINSEL_ConfigPin(&PinCfg);

	PinCfg.Pinnum = 26;
	PINSEL_ConfigPin(&PinCfg);

#ifdef HEXA_X

	PinCfg.Funcnum = 2;		// pin config for camera servo pwm  S1 or S5 for HEXA_X,
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 1;

	PinCfg.Pinnum = 18;
	PINSEL_ConfigPin(&PinCfg);

	PinCfg.Funcnum = 3;		// pin config for camera servo pwm S2  or S6 for HEXA_X, Funcnum is 3 instead of 2
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 3;

	PinCfg.Pinnum = 25;
	PINSEL_ConfigPin(&PinCfg);

#endif




	/* Set match value for PWM match channel 0 = 100, update immediately */
	PWM_MatchUpdate(LPC_PWM1, 0, 2800, PWM_MATCH_UPDATE_NOW); // 2500 = 400 Hz, 2800 = 358 Hz, 4000 = 250 Hz, 20000= 50 Hz.
	/* PWM Timer/Counter will be reset when channel 0 matching
	 * no interrupt when match
	 * no stop when match
	 */
	PWMMatchCfgDat.IntOnMatch = DISABLE;
	PWMMatchCfgDat.MatchChannel = 0;
	PWMMatchCfgDat.ResetOnMatch = ENABLE;
	PWMMatchCfgDat.StopOnMatch = DISABLE;
	PWM_ConfigMatch(LPC_PWM1, &PWMMatchCfgDat);
	/* Configure each PWM channel: --------------------------------------------- */
	/* - Channel 2: Double Edge
	 * - Channel 4: Double Edge
	 * - Channel 5: Single Edge
	 * The Match register values are as follows:
	 * - MR0 = 100 (PWM rate)
	 * - MR1 = 41, MR2 = 78 (PWM2 output)
	 * - MR3 = 53, MR4 = 27 (PWM4 output)
	 * - MR5 = 65 (PWM5 output)
	 * PWM Duty on each PWM channel:
	 * - Channel 2: Set by match 1, Reset by match 2.
	 * - Channel 4: Set by match 3, Reset by match 4.
	 * - Channel 5: Set by match 0, Reset by match 5.
	 */

	/* Edge setting ------------------------------------ */
//	PWM_ChannelConfig(LPC_PWM1, 1, PWM_CHANNEL_SINGLE_EDGE);		// channel 1 should be disabled always, else no pwm channel will work
	PWM_ChannelConfig(LPC_PWM1, 2, PWM_CHANNEL_SINGLE_EDGE);
	PWM_ChannelConfig(LPC_PWM1, 3, PWM_CHANNEL_SINGLE_EDGE);
	PWM_ChannelConfig(LPC_PWM1, 4, PWM_CHANNEL_SINGLE_EDGE);
	PWM_ChannelConfig(LPC_PWM1, 5, PWM_CHANNEL_SINGLE_EDGE);
	PWM_ChannelConfig(LPC_PWM1, 6, PWM_CHANNEL_SINGLE_EDGE);

	/* Match value setting ------------------------------------ */

#ifdef HEXA_X
	PWM_MatchUpdate(LPC_PWM1, 1, 0, PWM_MATCH_UPDATE_NEXT_RST);		// remember to change init PWM for fixed wing
	PWM_MatchUpdate(LPC_PWM1, 2, 0, PWM_MATCH_UPDATE_NEXT_RST);
#endif
	PWM_MatchUpdate(LPC_PWM1, 3, 0, PWM_MATCH_UPDATE_NEXT_RST);
	PWM_MatchUpdate(LPC_PWM1, 4, 0, PWM_MATCH_UPDATE_NEXT_RST);
	PWM_MatchUpdate(LPC_PWM1, 5, 0, PWM_MATCH_UPDATE_NEXT_RST);
	PWM_MatchUpdate(LPC_PWM1, 6, 0, PWM_MATCH_UPDATE_NEXT_RST);
	/* Match option setting ------------------------------------ */
	PWMMatchCfgDat.IntOnMatch = DISABLE;

	PWMMatchCfgDat.ResetOnMatch = DISABLE;
	PWMMatchCfgDat.StopOnMatch = DISABLE;
	for(temp = 1; temp <= 6; temp++)
	{
		PWMMatchCfgDat.MatchChannel = temp;
		PWM_ConfigMatch(LPC_PWM1, &PWMMatchCfgDat);
	}
	/* Enable PWM Channel Output ------------------------------------ */
	/* Channel 2 */

#ifdef HEXA_X

	PWM_ChannelCmd(LPC_PWM1, 1, ENABLE);
	PWM_ChannelCmd(LPC_PWM1, 2, ENABLE);

#endif

	PWM_ChannelCmd(LPC_PWM1, 3, ENABLE);
	/* Channel 4 */
	PWM_ChannelCmd(LPC_PWM1, 4, ENABLE);
	/* Channel 5 */
	PWM_ChannelCmd(LPC_PWM1, 5, ENABLE);
	PWM_ChannelCmd(LPC_PWM1, 6, ENABLE);

	/* Reset and Start counter */
	PWM_ResetCounter(LPC_PWM1);
	PWM_CounterCmd(LPC_PWM1, ENABLE);

	/* Start PWM now */
	PWM_Cmd(LPC_PWM1, ENABLE);
}

void Motors_set(){



	if(RCmode.arm==1){

#ifdef HEXA_X
		PWM_MatchUpdate(LPC_PWM1, 1, motors.s5, PWM_MATCH_UPDATE_NEXT_RST);
		PWM_MatchUpdate(LPC_PWM1, 2, motors.s6, PWM_MATCH_UPDATE_NEXT_RST);

#endif
		PWM_MatchUpdate(LPC_PWM1, 3, motors.s1, PWM_MATCH_UPDATE_NEXT_RST);
		PWM_MatchUpdate(LPC_PWM1, 4, motors.s2, PWM_MATCH_UPDATE_NEXT_RST);
		PWM_MatchUpdate(LPC_PWM1, 5, motors.s3, PWM_MATCH_UPDATE_NEXT_RST);
		PWM_MatchUpdate(LPC_PWM1, 6, motors.s4, PWM_MATCH_UPDATE_NEXT_RST);
	}
	else {

#ifdef HEXA_X
		PWM_MatchUpdate(LPC_PWM1, 1, 1000, PWM_MATCH_UPDATE_NEXT_RST);
		PWM_MatchUpdate(LPC_PWM1, 2, 1000, PWM_MATCH_UPDATE_NEXT_RST);

#endif
		PWM_MatchUpdate(LPC_PWM1, 3, 1000, PWM_MATCH_UPDATE_NEXT_RST);
		PWM_MatchUpdate(LPC_PWM1, 4, 1000, PWM_MATCH_UPDATE_NEXT_RST);
		PWM_MatchUpdate(LPC_PWM1, 5, 1000, PWM_MATCH_UPDATE_NEXT_RST);
		PWM_MatchUpdate(LPC_PWM1, 6, 1000, PWM_MATCH_UPDATE_NEXT_RST);
	}

}

void EINT3_IRQHandler(void) // interrupt on rising edge
{
	static int16_t pulsewidth=0;
	static float width = 0;
	/* clear the EINT3 flag */
	EXTI_ClearEXTIFlag(3);

	highus = (LPC_TIM2->TC);

	if(highus<lowus)pulsewidth = (20000000 + highus) - lowus;
	else pulsewidth = highus - lowus;

	if(pulsewidth>3500) pulsewidth = 3500;

#ifdef RCB7X
	pulsewidth =  ((pulsewidth-1200)*5.0f)/4;
#else
	width =  (pulsewidth-1103)/1.022f;
#endif

	if(width>800 || width< -800) channel_index = 0;

	channel[channel_index++] = 1500 + width;
	if(channel_index==NUM_CHANNELS)channel_index=0;

	//	blink();


}

void EINT1_IRQHandler(void)
{

	/* clear the EINT1 flag */
	EXTI_ClearEXTIFlag(1);

	lowus = (LPC_TIM2->TC);




	//	blink();


}


void PPM_setup(){
	PINSEL_CFG_Type PinCfg;
	EXTI_InitTypeDef EXTICfg;

	/* Initialize EXT pin and register */
	/* P2.13 as /EINT0 */
	PinCfg.Funcnum = 1;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Pinnum = 13;
	PinCfg.Portnum = 2;
	PINSEL_ConfigPin(&PinCfg);

	/* Initialize EXT pin and register */
	/* P2.13 as /EINT0 */
	PinCfg.Funcnum = 1;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Pinnum = 11;
	PinCfg.Portnum = 2;
	PINSEL_ConfigPin(&PinCfg);


	EXTI_Init();

	EXTICfg.EXTI_Line = _EXTINTH;
	/* edge sensitive */
	EXTICfg.EXTI_Mode = EXTI_MODE_EDGE_SENSITIVE;
	EXTICfg.EXTI_polarity = EXTI_POLARITY_HIGH_ACTIVE_OR_RISING_EDGE;
	EXTI_ClearEXTIFlag(_EXTINTH);
	EXTI_Config(&EXTICfg);

	NVIC_SetPriorityGrouping(4);
	NVIC_SetPriority(_EXT_IRQH, 0);
	NVIC_EnableIRQ(_EXT_IRQH);

	EXTICfg.EXTI_Line = _EXTINTL;
	/* edge sensitive */
	EXTICfg.EXTI_Mode = EXTI_MODE_EDGE_SENSITIVE;
	EXTICfg.EXTI_polarity = EXTI_POLARITY_LOW_ACTIVE_OR_FALLING_EDGE;
	EXTI_ClearEXTIFlag(_EXTINTL);
	EXTI_Config(&EXTICfg);

	NVIC_SetPriorityGrouping(4);
	NVIC_SetPriority(_EXT_IRQL, 1);
	NVIC_EnableIRQ(_EXT_IRQL);
}

