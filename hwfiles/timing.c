
#include "timing.h"
#include "control.h"
#include "lpc17xx.h"
#include "lpc17xx_clkpwr.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_nvic.h"
#include "lpc17xx_gpio.h"

#include "lpc17xx_rit.h"
#include "lpc17xx_libcfg_default.h"

#include "mlink.h"
#include "LED_Blink.h"
#include "gps.h"
#include "waypoint.h"



char Triggerset=0;


/*
 * @brief    Timer 0 interrupt
 * @param    None.
 * @return   None.
 */

char onehzloopflag=0;
char hundredhzloopflag=0;
char fiftyhzloopflag=0;
char tenhzloopflag=0;
char twentyhzloopflag=0;
char fivehzloopflag=0;
char IMUhzloopflag=0;
uint32_t millisnow = 0;
uint32_t hundredhzlast=0;
uint32_t onehzlast = 0;
uint32_t fiftyhzlast = 0;
uint32_t tenhzlast = 0;
uint32_t twentyhzlast = 0;
uint32_t fivehzlast = 0;
uint32_t IMUhzlast = 0;

uint32_t boscam_pwmus = 1500;




//void TIMER0_IRQHandler (void)
//{
//	/*  Clear Interrupt */
//	TIM_ClearIntPending(LPC_TIM0,TIM_MR0_INT);
//	/* Code...*/
//
//	onehzloopflag = 1;
//
//	return;
//}

void timing_init(){

	//	Rit_init();
	Timer3_init();
	Timer2_init();
//	Timer1_init();	// camera trigger pwm timer
	Timer0_init();

}

void Timer3_init(){											// main timer for loop timing was for Timer0, now changed to Timer3 for Boscam integration
	TIM_TIMERCFG_Type TMR0_Cfg;
	TIM_MATCHCFG_Type TMR0_Match;

	/* On reset, Timer0/1 are enabled (PCTIM0/1 = 1), and Timer2/3 are disabled (PCTIM2/3 = 0).*/
	/* Initialize timer 3, prescale count time of 100uS */
	TMR0_Cfg.PrescaleOption = TIM_PRESCALE_USVAL;
	TMR0_Cfg.PrescaleValue = 100;
	/* Use channel 0, MR0 */
	TMR0_Match.MatchChannel = 0;
	/* DISABLE interrupt when MR0 matches the value in TC register */
	TMR0_Match.IntOnMatch = DISABLE;
	/* Enable reset on MR0: TIMER will reset if MR0 matches it */
	TMR0_Match.ResetOnMatch = TRUE;
	/* Don't stop on MR0 if MR0 matches it*/
	TMR0_Match.StopOnMatch = FALSE;
	/* Do nothing for external output pin if match (see cmsis help, there are another options) */
	TMR0_Match.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
	/* Set Match value, count value of 10000 (10000 * 100uS = 1000000us = 1s --> 1 Hz) */
	TMR0_Match.MatchValue = 10000*72000;  // 1s*72000 = 20 hrs
	/* Set configuration for Tim_config and Tim_MatchConfig */
	TIM_Init(LPC_TIM3, TIM_TIMER_MODE, &TMR0_Cfg);
	TIM_ConfigMatch(LPC_TIM3, &TMR0_Match);

	//		/* Preemption = 1, sub-priority = 1 */				// main timer for loop timing was for Timer0, now changed to Timer3 for Boscam integration
	//		NVIC_SetPriority(TIMER3_IRQn, ((0x01<<3)|0x01));
	//		/* Enable interrupt for timer 0 */
	//		NVIC_EnableIRQ(TIMER3_IRQn);
	/* Start timer 0 */
	TIM_Cmd(LPC_TIM3, ENABLE);

}

void boscam_pwm(){

	TIM_MATCHCFG_Type TIM_MatchConfigStruct;

	TIM_ResetCounter(LPC_TIM0);
	TIM_Cmd(LPC_TIM0,DISABLE);

	TIM_MatchConfigStruct.MatchChannel = 0;
	/* Disable interrupt when MR0 matches the value in TC register */
	TIM_MatchConfigStruct.IntOnMatch = FALSE;
	/* Enable reset on MR0: TIMER will reset if MR0 matches it */
	TIM_MatchConfigStruct.ResetOnMatch = TRUE;
	/* Stop on MR0 if MR0 matches it */
	TIM_MatchConfigStruct.StopOnMatch = TRUE;


	TIM_MatchConfigStruct.ExtMatchOutputType = TIM_EXTMATCH_LOW;
	/* Set Match value */
	TIM_MatchConfigStruct.MatchValue = boscam_pwmus;

	TIM_ConfigMatch(LPC_TIM0, &TIM_MatchConfigStruct);

	LPC_TIM0->EMR |=1;  // set the timer output by setting 0th bith of EMR to 1

	TIM_Cmd(LPC_TIM0,ENABLE);
}

void Timer0_init(){		// timer for servo generation for boscam (50 hz)

	//	TIM_TIMERCFG_Type TMR0_Cfg;
	//	TIM_MATCHCFG_Type TMR0_Match;
	//
	//	/* On reset, Timer0/1 are enabled (PCTIM0/1 = 1), and Timer2/3 are disabled (PCTIM2/3 = 0).*/
	//	/* Initialize timer 0, prescale count time of 1uS */
	//	TMR0_Cfg.PrescaleOption = TIM_PRESCALE_USVAL;
	//	TMR0_Cfg.PrescaleValue = 1;
	//	/* Use channel 0, MR0 */
	//	TMR0_Match.MatchChannel = 0;
	//	/* DISABLE interrupt when MR0 matches the value in TC register */
	//	TMR0_Match.IntOnMatch = DISABLE;
	//	/* Enable reset on MR0: TIMER will reset if MR0 matches it */
	//	TMR0_Match.ResetOnMatch = TRUE;
	//	/* Don't stop on MR0 if MR0 matches it*/
	//	TMR0_Match.StopOnMatch = FALSE;
	//	/* Do nothing for external output pin if match (see cmsis help, there are another options) */
	//	TMR0_Match.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
	//	/* Set Match value, count value of 10000 (10000 * 100uS = 1000000us = 1s --> 1 Hz) */
	//	TMR0_Match.MatchValue = 10000*72000;  // 1s*72000 = 20 hrs
	//	/* Set configuration for Tim_config and Tim_MatchConfig */
	//	TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &TMR0_Cfg);
	//	TIM_ConfigMatch(LPC_TIM0, &TMR0_Match);
	//
	//	//		/* Preemption = 1, sub-priority = 1 */
	//	//		NVIC_SetPriority(TIMER0_IRQn, ((0x01<<3)|0x01));
	//	//		/* Enable interrupt for timer 0 */
	//	//		NVIC_EnableIRQ(TIMER0_IRQn);
	//	/* Start timer 0 */
	//	TIM_Cmd(LPC_TIM0, ENABLE);


	PINSEL_CFG_Type PinCfg;
	TIM_TIMERCFG_Type TIM_ConfigStruct;
	TIM_MATCHCFG_Type TIM_MatchConfigStruct;

	/* Configure P3.25 as MAT0.0 */
	PinCfg.Funcnum = 2;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 3;
	PinCfg.Pinnum = 25;
	PINSEL_ConfigPin(&PinCfg);

	/* Initialize timer, prescale count time of 1uS */
	TIM_ConfigStruct.PrescaleOption = TIM_PRESCALE_USVAL;
	TIM_ConfigStruct.PrescaleValue	= 1;
	TIM_Init(LPC_TIM0, TIM_TIMER_MODE,&TIM_ConfigStruct);


//	/* use channel 0, MR0 */
//	TIM_MatchConfigStruct.MatchChannel = 0;
//	/* Disable interrupt when MR0 matches the value in TC register */
//	TIM_MatchConfigStruct.IntOnMatch = FALSE;
//	/* Enable reset on MR0: TIMER will reset if MR0 matches it */
//	TIM_MatchConfigStruct.ResetOnMatch = FALSE;
//	/* Stop on MR0 if MR0 matches it */
//	TIM_MatchConfigStruct.StopOnMatch = FALSE;
//	/* Toggle MR0.0 pin if MR0 matches it */
//	TIM_MatchConfigStruct.ExtMatchOutputType = TIM_EXTMATCH_LOW;
//	/* Set Match value */
//	TIM_MatchConfigStruct.MatchValue = 100000;
//	TIM_ConfigMatch(LPC_TIM0, &TIM_MatchConfigStruct);
//
//
//	/* use channel 3, MR3 */
//	TIM_MatchConfigStruct.MatchChannel = 3;
//	/* Disable interrupt when MR0 matches the value in TC register */
//	TIM_MatchConfigStruct.IntOnMatch = FALSE;
//	/* Enable reset on MR0: TIMER will reset if MR0 matches it */
//	TIM_MatchConfigStruct.ResetOnMatch = TRUE;
//	/* Stop on MR0 if MR0 matches it */
//	TIM_MatchConfigStruct.StopOnMatch = FALSE;
//	/* Toggle MR0.0 pin if MR0 matches it */
//	TIM_MatchConfigStruct.ExtMatchOutputType = TIM_EXTMATCH_HIGH;
//	/* Set Match value */
//	TIM_MatchConfigStruct.MatchValue = 300000;
//	TIM_ConfigMatch(LPC_TIM0, &TIM_MatchConfigStruct);
//
//	TIM_Cmd(LPC_TIM0,ENABLE);


}

//void trigger(int timems){
//	TIM_MATCHCFG_Type TMR0_Match;
//
//	if(Triggerset==1)return;
//
//	/* Use channel 0, MR0 */
//	TMR0_Match.MatchChannel = 0;
//	/* DISABLE interrupt when MR0 matches the value in TC register */
//	TMR0_Match.IntOnMatch = ENABLE;
//	/* Enable reset on MR0: TIMER will reset if MR0 matches it */
//	TMR0_Match.ResetOnMatch = TRUE;
//	/* Don't stop on MR0 if MR0 matches it*/
//	TMR0_Match.StopOnMatch = TRUE;
//	/* Do nothing for external output pin if match (see cmsis help, there are another options) */
//	TMR0_Match.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
//	/* Set Match value, count value of 10000 (10000 * 100uS = 1000000us = 1s --> 1 Hz) */
//	TMR0_Match.MatchValue = timems*10;  // 100us*mstimenumber
//
//
//	TIM_ConfigMatch(LPC_TIM1, &TMR0_Match);
//	TIM_ResetCounter(LPC_TIM1);
//	TIM_Cmd(LPC_TIM1, ENABLE);
//	LPC_GPIO1->FIOSET = (1<<19);  // set the trigger
//
//	if(isVideo || isVideoTest==1) boscam_pwmus = 1900; // video
//	else boscam_pwmus = 1100; // photo
//
//	if(isPhotoTest==1)  boscam_pwmus = 1100; // photo
//
//	Triggerset=1;
//
//
//}


//void TIMER1_IRQHandler (void)
//{
//	/*  Clear Interrupt */
//	TIM_ClearIntPending(LPC_TIM1,TIM_MR0_INT);
//	/* Code...*/
//
//	LPC_GPIO1->FIOCLR = (1<<19);  // clear the trigger
//
//
//	WPtimeout=1;
//	Triggerset=0;
//
//	boscam_pwmus = 1500; // photo
//
//	blink();
//
//
//}

//void Timer1_init(){
//
//	// camera trigger gpio setup
//
//	LPC_GPIO1->FIODIR |= (1<<19);
//
//	LPC_GPIO1->FIOCLR = (1<<19);
//
//
//	TIM_TIMERCFG_Type TMR0_Cfg;
//	TIM_MATCHCFG_Type TMR0_Match;
//
//	/* On reset, Timer0/1 are enabled (PCTIM0/1 = 1), and Timer2/3 are disabled (PCTIM2/3 = 0).*/
//	/* Initialize timer 0, prescale count time of 100uS */
//	TMR0_Cfg.PrescaleOption = TIM_PRESCALE_USVAL;
//	TMR0_Cfg.PrescaleValue = 100;
//	/* Use channel 0, MR0 */
//	TMR0_Match.MatchChannel = 0;
//	/* DISABLE interrupt when MR0 matches the value in TC register */
//	TMR0_Match.IntOnMatch = ENABLE;
//	/* Enable reset on MR0: TIMER will reset if MR0 matches it */
//	TMR0_Match.ResetOnMatch = TRUE;
//	/* Don't stop on MR0 if MR0 matches it*/
//	TMR0_Match.StopOnMatch = TRUE;
//	/* Do nothing for external output pin if match (see cmsis help, there are another options) */
//	TMR0_Match.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
//	/* Set Match value, count value of 10000 (10000 * 100uS = 1000000us = 1s --> 1 Hz) */
//	TMR0_Match.MatchValue = 10000*72000;  // 1s*72000 = 20 hrs
//	/* Set configuration for Tim_config and Tim_MatchConfig */
//	TIM_Init(LPC_TIM1, TIM_TIMER_MODE, &TMR0_Cfg);
//	TIM_ConfigMatch(LPC_TIM1, &TMR0_Match);
//
//	/* Preemption = 1, sub-priority = 1 */
//	NVIC_SetPriority(TIMER1_IRQn, ((0x01<<3)|0x01));
//	/* Enable interrupt for timer 0 */
//	NVIC_EnableIRQ(TIMER1_IRQn);
//	/* Start timer 0 */
//	TIM_Cmd(LPC_TIM1, ENABLE);
//
//}

void TIMER2_IRQHandler (void)
{
	/*  Clear Interrupt */
	TIM_ClearIntPending(LPC_TIM2,TIM_MR0_INT);
	/* Code...*/
	blink();
}

void Timer2_init(){
	TIM_TIMERCFG_Type TMR0_Cfg;
	TIM_MATCHCFG_Type TMR0_Match;

	/* On reset, Timer0/1 are enabled (PCTIM0/1 = 1), and Timer2/3 are disabled (PCTIM2/3 = 0).*/
	/* Initialize timer 0, prescale count time of 100uS */
	TMR0_Cfg.PrescaleOption = TIM_PRESCALE_USVAL;
	TMR0_Cfg.PrescaleValue = 1;
	/* Use channel 0, MR0 */
	TMR0_Match.MatchChannel = 0;
	/* DISABLE interrupt when MR0 matches the value in TC register */
	TMR0_Match.IntOnMatch = DISABLE;
	/* Enable reset on MR0: TIMER will reset if MR0 matches it */
	TMR0_Match.ResetOnMatch = TRUE;
	/* Don't stop on MR0 if MR0 matches it*/
	TMR0_Match.StopOnMatch = FALSE;
	/* Do nothing for external output pin if match (see cmsis help, there are another options) */
	TMR0_Match.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
	/* Set Match value, count value of 1000000 (1000000 * 1uS = 1000000us = 1s --> 1 Hz) */
	TMR0_Match.MatchValue = 20000000;  // 20s
	/* Set configuration for Tim_config and Tim_MatchConfig */
	TIM_Init(LPC_TIM2, TIM_TIMER_MODE, &TMR0_Cfg);
	TIM_ConfigMatch(LPC_TIM2, &TMR0_Match);

	//		/* Preemption = 1, sub-priority = 1 */
	//		NVIC_SetPriority(TIMER2_IRQn, ((0x01<<3)|0x01));
	//		/* Enable interrupt for timer 0 */
	//		NVIC_EnableIRQ(TIMER2_IRQn);
	/* Start timer 0 */
	TIM_Cmd(LPC_TIM2, ENABLE);

}

void RIT_IRQHandler(void)
{
	/* call this to clear interrupt flag */
	RIT_GetIntStatus(LPC_RIT);

	onehzloopflag=1;
}

void Rit_init(void) {
	RIT_Init(LPC_RIT);
	/* Configure time_interval for RIT
	 * In this case: time_interval = 1000 ms = 1s
	 * So, RIT will generate interrupt each 1s
	 */
	RIT_TimerConfig(LPC_RIT,1000);

	NVIC_EnableIRQ(RIT_IRQn);

}

uint32_t millis(){

	return (LPC_TIM3->TC)/10;		// main timer for loop timing was for Timer0, now changed to Timer3 for Boscam integration
}

uint64_t micros(){

	return (uint64_t)(LPC_TIM3->TC)*100; // main timer for loop timing was for Timer0, now changed to Timer3 for Boscam integration
}

uint32_t millisbyten(){

	return (LPC_TIM3->TC);			// main timer for loop timing was for Timer0, now changed to Timer3 for Boscam integration
}

void timing(){

	static uint32_t millisbytennow = 1;

	millisbytennow = millisbyten();
	millisnow = millisbytennow/10;


	if((millisnow - 1000)>onehzlast){
		onehzloopflag=1;
		onehzlast = millisnow;
	}

	if((millisnow - 10)>hundredhzlast){
		hundredhzloopflag=1;
		hundredhzlast = millisnow;
	}

	if((millisnow - 20)>fiftyhzlast){
		fiftyhzloopflag=1;
		fiftyhzlast = millisnow;
	}

	if((millisnow - 100)>tenhzlast){
		tenhzloopflag=1;
		tenhzlast = millisnow;
	}

	if((millisnow - 50)>twentyhzlast){
		twentyhzloopflag=1;
		twentyhzlast = millisnow;
	}

	if((millisnow - 200)>fivehzlast){
		fivehzloopflag=1;
		fivehzlast = millisnow;
	}


	if((millisbytennow - (10000/(*IMU_HZ)))>IMUhzlast){		// IMU timing on 100 micros
		IMUhzloopflag=1;
		IMUhzlast = millisbytennow;
	}
}
