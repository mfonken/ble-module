/***********************************************************************************************//**
 * \file   app.c
 * \brief  Main application.
 ***************************************************************************************************
 *      Author: Matthew Fonken
 **************************************************************************************************/

/* Own header */
#include "app.h"

#include "app_interrupts.h"

/* Libraries containing Gecko configuration values */
#include "em_emu.h"
#include "em_cmu.h"
#include "em_timer.h"
#include "em_usart.h"
#include "em_gpio.h"

/* Peripheral/Sensor headers */
#include "cam_controller.h"
#include "PCA9534A.h"
#include "LSM9DS1.h"
#include "CPT112S.h"

/* System utilities */
#include "usart_sp.h"


/* App mode */
app_t mode;

void app_init( void )
{
	mode._2d = false;
	mode._3d = true;
	mode._sl = false;
	appModeSet( &mode );

	SysCtlr_Init();
	IMU_Init();
	Touch_Init();
	Camera_Init();
}

void app( void )
{

}

void appModeSet( app_t * m )
{
	if( m->_2d )
	{
		enter2DMode();
	}
	else
	{
		exit2DMode();
	}
	if( m->_3d )
	{
		enter3DMode();
	}
	else
	{
		exit3DMode();
	}
	if( m->_sl )
	{
		enterSleepMode();
	}
	else
	{
		exitSleepMode();
	}
	mode = *m;
}

void enter2DMode( void )
{
	Enable_Force_Sensor();
}

void exit2DMode( void )
{
	Disable_Force_Sensor();
}

void enter3DMode( void )
{
	Enable_Camera();
	Enable_Magnometer();
}

void exit3DMode( void )
{
	Disable_Camera();
	Disable_Magnometer();
}

void enterSleepMode( void )
{
	releaseTimer();
	registerGPIOInterrupt();

	EMU_EnterEM3( true );
}

void exitSleepMode( void )
{
	EMU_EnterEM1();

	registerTimer( SYNC_TIMER_PERIOD );
	releaseGPIOInterrupt();
}

void USART0_RX_IRQHandler(void)
{
	USART_IntClear(CAM_UART, USART_IF_RXDATAV);
}

void TIMER0_IRQHandler(void)
{
	TIMER_IntClear( TIMER0, TIMER_IF_OF );      	// Clear overflow flag
	Print_String( "\tTimer.\r\n\0", 10 );
}

void GPIO_ODD_IRQHandler(void)
{
	GPIO_IntClear(GPIO_IntGet());
	exitSleepMode();
}
