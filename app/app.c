/***********************************************************************************************//**
 * \file   app.c
 * \brief  Main application.
 ***************************************************************************************************
 *      Author: Matthew Fonken
 **************************************************************************************************/

/* Own header */
#include "app.h"

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

/* App variables */
app_t 			mode;
//sync_t 			sync;
sensor_data_t 	sensors;

/* Interrupt variables */
uint8_t buffer_rx;

/**********************/
/* Interrupt Handlers */
/**********************/

/* Camera USART */
void USART0_RX_IRQHandler(void)
{
	USART_IntClear(CAM_UART, USART_IF_RXDATAV);
	uint8_t in = USART_Rx( CAM_UART );

	if( buffer_rx > 0)
	{
		buffer_rx--;
		Camera_Buffer( in );
	}
	else
	{
		int8_t c = Camera_Check( in );
		if( c <= MAX_CENTROIDS ) buffer_rx = c;
	}
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

/* App */
void app_init( void )
{
	SysCtlr_Init();
	IMU_Init();
	Touch_Init();
	Camera_Init();

	mode._2d = _2D_MODE_DEFAULT;
	mode._3d = _3D_MODE_DEFAULT;
	mode._sl = _SLEEP_MODE_DEFAULT;
	appModeSet( &mode );

	registerTimer( SYNC_TIMER, SYNC_TIMER_PERIOD );
	registerTimer( FORCE_TIMER, FORCE_TIMER_PERIOD );
}

void app( void )
{

}

void appModeSet( app_t * m )
{
	app_t mode = *m;
	if( mode._sl )
	{
		enterSleepMode();
		return;
	}
	if( mode._2d )	enableStylusSensors();
	else			disableStylusSensors();
	if( mode._3d ) 	enableSpatialSensors();
	else			disableSpatialSensors();
	sync_t sync;
	sync.accel 	= mode._2d;
	sync.gyro  	= mode._2d;
	sync.mag	= mode._3d;
	sync.cam	= mode._3d;
	enableSyncTimer( &sync );
}

void autoDetectMode( void )
{
	/*TODO: Scan camera and force sensor to determine appropriate mode.
	 * 		Camera has two or more beacons.
	 * 		Force sensor has force.
	 */
	mode._2d = _2D_MODE_DEFAULT;
	mode._3d = _3D_MODE_DEFAULT;
	mode._sl = _SLEEP_MODE_DEFAULT;
}

void enableSyncTimer( sync_t * s )
{
	sensorSyncSet( s );
	enableTimer( SYNC_TIMER );
}
void disableSyncTimer( void )
{
	disableTimer( SYNC_TIMER );
}

void enableStylusSensors( void )
{
	Enable_Force_Sensor();
	enableTimer( FORCE_TIMER );
}

void disableStylusSensors( void )
{
	Disable_Force_Sensor();
	disableTimer( FORCE_TIMER );
}

void enableSpatialSensors( void )
{
	Enable_Camera();
	Enable_Magnometer();
}

void disableSpatialSensors( void )
{
	Disable_Camera();
	Disable_Magnometer();
}

void enterSleepMode( void )
{
	disableTimer( SYNC_TIMER );
	disableTimer( FORCE_TIMER );
	disableStylusSensors();
	disableSpatialSensors();

	enableGPIOInterrupt();
	EMU_EnterEM3( true );
}

void exitSleepMode( void )
{
	EMU_EnterEM1();
	disableGPIOInterrupt();

	autoDetectMode();

	registerTimer( SYNC_TIMER, SYNC_TIMER_PERIOD );
	registerTimer( FORCE_TIMER, FORCE_TIMER_PERIOD );

}
