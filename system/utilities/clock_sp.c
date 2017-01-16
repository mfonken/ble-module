/*
 * clock_sp.c
 *
 *  Created on: Jan 15, 2017
 *      Author: Matthew Fonken
 */

/* Own header */
#include "clock_sp.h"

/* Defines*/
#define LFRCO_FREQUENCY                 32768
#define WAKEUP_INTERVAL_MS              500
#define RTC_COUNT_BETWEEN_WAKEUP        (((LFRCO_FREQUENCY * WAKEUP_INTERVAL_MS) / 1000)-1)

/**************************************************************************//**
 * @brief  Start LFRCO for RTC/RTCC
 * Starts the low frequency RC oscillator (LFRCO) and routes it to the RTC/RTCC
 *****************************************************************************/
void startLfrcoForRtc(void)
{
  /* Starting LFRCO and waiting until it is stable */
  CMU_OscillatorEnable(cmuOsc_LFRCO, true, true);

  /* Enabling clock to the interface of the low energy modules */
  CMU_ClockEnable(cmuClock_CORELE, true);

#if defined(RTCC_PRESENT) && (RTCC_COUNT > 0)
  /* Routing the LFRCO clock to the RTCC */
  CMU_ClockSelectSet(cmuClock_LFE, cmuSelect_LFRCO);
  CMU_ClockEnable(cmuClock_RTCC, true);
#else
  /* Routing the LFRCO clock to the RTC */
  CMU_ClockSelectSet(cmuClock_LFA,cmuSelect_LFRCO);
  CMU_ClockEnable(cmuClock_RTC, true);
#endif
}

/**************************************************************************//**
 * @brief  Setup RTC/RTCC
 * On compare match with compare channel, clear counter and set interrupt
 *****************************************************************************/
void SYSCLK_Init( void )
{
	  /* Configuring clocks in the Clock Management Unit (CMU) */
	  startLfrcoForRtc();

	#if defined(RTCC_PRESENT) && (RTCC_COUNT > 0)
	  RTCC_Init_TypeDef rtccInit = RTCC_INIT_DEFAULT;
	  RTCC_CCChConf_TypeDef rtccInitCompareChannel = RTCC_CH_INIT_COMPARE_DEFAULT;

	  rtccInit.cntWrapOnCCV1 = true;        /* Clear counter on compare match */
	  rtccInit.presc = rtccCntPresc_1;

	  /* Setting the compare value of the RTCC */
	  RTCC_ChannelInit(SYSCLK_CHANNEL, &rtccInitCompareChannel);
	  RTCC_ChannelCCVSet(SYSCLK_CHANNEL, RTC_COUNT_BETWEEN_WAKEUP);

	  /* Enabling Interrupt from RTCC */
	  RTCC_IntEnable(RTCC_IEN_CC1);
	  NVIC_ClearPendingIRQ(RTCC_IRQn);
	  NVIC_EnableIRQ(RTCC_IRQn);

	  /* Initialize the RTCC */
	  RTCC_Init(&rtccInit);
	#else
	  RTC_Init_TypeDef rtcInit = RTC_INIT_DEFAULT;

	  /* Setting the compare value of the RTC */
	  RTC_CompareSet(0, RTC_COUNT_BETWEEN_WAKEUP);

	  /* Enabling Interrupt from RTC */
	  RTC_IntEnable(RTC_IEN_COMP0);
	  NVIC_ClearPendingIRQ(RTC_IRQn);
	  NVIC_EnableIRQ(RTC_IRQn);

	  /* Initialize the RTC */
	  RTC_Init(&rtcInit);
	#endif
}

uint32_t timestamp( void )
{
	return RTCC_TimeGet();
}
