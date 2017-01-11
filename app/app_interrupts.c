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
#include "em_gpio.h"

/* System utilities */
#include "usart_sp.h"

/* Peripheral/Sensor headers */
#include "cam_controller.h"
#include "PCA9534A.h"
#include "LSM9DS1.h"
#include "CPT112S.h"

void registerTimer( uint32_t T )
{
	Print_String( "Starting timer.\r\n", 17 );

	/* Select TIMER0 parameters */
	TIMER_Init_TypeDef timerInit =
	{
		.enable     = true,
		.debugRun   = true,
		.prescale   = TIMER_PRESCALE,
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
	TIMER_TopSet( TIMER0, ( T * TICK_TO_MS ) );

	/* Configure TIMER */
	TIMER_Init( TIMER0, &timerInit );
}

void releaseTimer( void )
{
	TIMER_Reset( TIMER0 );
}

void registerCameraInterrupt( void )
{
	Print_String( "Starting camera interrupt.\r\n", 17 );

	/* Prepare UART Rx and Tx interrupts */
	USART_IntClear(CAM_UART, _USART_IF_MASK);
	USART_IntEnable(CAM_UART, USART_IF_RXDATAV);
	NVIC_ClearPendingIRQ(USART0_RX_IRQn);
	NVIC_ClearPendingIRQ(USART0_TX_IRQn);
	NVIC_EnableIRQ(USART0_RX_IRQn);
	NVIC_EnableIRQ(USART0_TX_IRQn);
}

void releaseCameraInterrupt( void )
{
	USART_IntDisable(CAM_UART, USART_IF_RXDATAV);
}

void registerGPIOInterrupt( void )
{
	/* Enable interrupt in core for odd gpio interrupts */
	NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);

	/* Configure PA0 interrupt on rising edge */
	GPIO_IntConfig(gpioPortA, 0, true, false, true);
}

void releaseGPIOInterrupt( void )
{
	NVIC_DisableIRQ(GPIO_ODD_IRQn);
}
