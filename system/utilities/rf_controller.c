/*
 * rf_controller.c
 *
 *  Created on: Jan 12, 2017
 *      Author: Matthew Fonken
 */

#include "rf_controller.h"

beacon_device_t beacon_status;

void RF_Session_Init( uint8_t i, uint32_t d )
{
	/* Enable timer interrupt vector in NVIC */
	NVIC_EnableIRQ( CRYOTIMER_IRQn );

	/* Enable overflow interrupt */
	CRYOTIMER_IntEnable( TIMER_IF_OF );

	beacon_status.session_duration = d;
	beacon_status.intensity = i;
	RF_Tx( &beacon_status );
	beacon_status.session_active = true;
}

void RF_Session_End( void )
{
	CRYOTIMER_IntDisable( TIMER_IF_OF );
	beacon_status.session_active = false;
}

void RF_Kick( void )
{
	RF_Tx( &beacon_status );
}

bool RF_Step( bool i )
{
	if( ( beacon_status.intensity < INTENSITY_STEP ) ||
		( beacon_status.intensity > INTENSITY_TOP ) )
	{
		RF_Kick();
		return false;
	}
	if( i ) beacon_status.intensity += INTENSITY_STEP;
	else 	beacon_status.intensity -= INTENSITY_STEP;

	RF_Tx( &beacon_status );
	return true;
}

void RF_Tx( beacon_device_t * b)
{
	USART_Tx( BEACON_USART, b->session_duration );
	USART_Tx( BEACON_USART, b->intensity );
}
