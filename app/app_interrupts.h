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

typedef struct
{
	uint8_t accel:1;
	uint8_t  gyro:1;
	uint8_t   mag:1;
	uint8_t   cam:1;
} sync_t;

void sensorSyncSet( sync_t * );

/* Interrupt Registers */
void registerTimer( TIMER_TypeDef * timer, uint32_t period );
void enableTimer(   TIMER_TypeDef * timer );
void disableTimer(  TIMER_TypeDef * timer );
void resetTimer(  	TIMER_TypeDef * timer );
void enableUARTInterrupt( 	void );
void disableUARTInterrupt( 	void );
void enableGPIOInterrupt(	void );
void disableGPIOInterrupt( 	void );

#endif /* SYSTEM_APP_INTERRUPTS_H_ */
