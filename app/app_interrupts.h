/*
 * app_interrupts.h
 *
 *  Created on: Jan 10, 2017
 *      Author: Matthew Fonken
 */

#ifndef SYSTEM_APP_INTERRUPTS_H_
#define SYSTEM_APP_INTERRUPTS_H_

#define TIMER_PRESCALE		timerPrescale1
#define TICK_TO_MS			( 1 / ( 1 << TIMER_PRESCALE ) )

/*
 * app_interrupts.c
 *
 *  Created on: Jan 10, 2017
 *      Author: Matthew Fonken
 */

#include "app_interrupts.h"

/* Libraries containing Gecko configuration values */
#include "em_timer.h"
#include "em_usart.h"

/* System utilities */
#include "usart_sp.h"

void registerTimer( 	  uint32_t T );
void releaseTimer( 				void );
void registerCameraInterrupt( 	void );
void releaseCameraInterrupt( 	void );
void registerGPIOInterrupt( 	void );
void releaseGPIOInterrupt( 		void );

#endif /* SYSTEM_APP_INTERRUPTS_H_ */
