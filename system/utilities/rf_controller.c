/*
 * rf_controller.c
 *
 *  Created on: Jan 12, 2017
 *      Author: Matthew Fonken
 */

#include "rf_controller.h"

#include "app.h"
#include "app_interrupts.h"

beacon_t beacon_status;

void RF_Session_Init( uint8_t i, uint8_t d )
{
	beacon_status.session_duration = d;
	beacon_status.intensity = i;
	RF_Tx( &beacon_status );
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

void RF_Tx( beacon_t * b)
{
	USART_Tx( BEACON_USART, b->session_duration );
	USART_Tx( BEACON_USART, b->intensity );
}
