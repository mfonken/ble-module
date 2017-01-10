/***********************************************************************************************//**
 * \file   app.c
 * \brief  Main application.
 ***************************************************************************************************
 *      Author: Matthew Fonken
 **************************************************************************************************/

/* Own header */
#include "app.h"

/* Libraries containing Gecko configuration values */
#include "em_timer.h"

/* Peripheral/Sensor headers */
#include "cam_controller.h"

/* System utilities */
#include "usart_sp.h"

/* App mode */
app_t mode;

void appInit( void )
{
	mode._2d = false;
	mode._3d = false;
	mode._lp = false;

	appModeSet( &mode );
}

void appModeSet( app_t * m )
{
	if( m->_3d == true )
	{
		initialize3DMode();
	}

	mode = *m;
}

void initialize3DMode( void )
{
	registerTimer( CAMERA_TIMER_COUNT_TOP );
}

void deinitialize3DMode( void )
{
	releaseTimer();
}

void TIMER0_IRQHandler(void)
{
	TIMER_IntClear( TIMER0, TIMER_IF_OF );      	// Clear overflow flag
	Print_String( "\tTimer.\r\n\0", 10 );
}

void registerTimer( uint32_t t )
{
	Print_String( "Starting timer.\r\n", 17 );

	/* Select TIMER0 parameters */
	TIMER_Init_TypeDef timerInit =
	{
		.enable     = true,
		.debugRun   = true,
		.prescale   = timerPrescale1024,
		.clkSel     = timerClkSelHFPerClk,
		.fallAction = timerInputActionNone,
		.riseAction = timerInputActionNone,
		.mode       = timerModeUp,
		.dmaClrAct  = false,
		.quadModeX4 = false,
		.oneShot    = false,
		.sync       = false,
	};

	/* Enable overflow interrupt */
	TIMER_IntEnable( TIMER0, TIMER_IF_OF );

	/* Enable TIMER0 interrupt vector in NVIC */
	NVIC_EnableIRQ( TIMER0_IRQn );

	/* Set TIMER Top value */
	TIMER_TopSet( TIMER0, t );

	/* Configure TIMER */
	TIMER_Init( TIMER0, &timerInit );
}

void releaseTimer( void )
{
	TIMER_Reset( TIMER0 );
}
