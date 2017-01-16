/*
 * rf_controller.h
 *
 *  Created on: Jan 12, 2017
 *      Author: Matthew Fonken
 */

#ifndef SYSTEM_UTILITIES_RF_CONTROLLER_H_
#define SYSTEM_UTILITIES_RF_CONTROLLER_H_

#include <stdint.h>
#include <stdbool.h>

/* Application */
#include "app.h"
#include "app_interrupts.h"

/* Gecko Library */
#include "em_usart.h"
#include "em_cryotimer.h"

#define BEACON_HEADER	0xbe
#define BEACON_USART	USART1

#define INTENSITY_STEP	0x10
#define INTENSITY_TOP 	0xff - INTENSITY_STEP

typedef struct
{
	uint32_t 	session_duration;
	uint8_t 	intensity;
	bool 		session_active;
} beacon_device_t;

extern beacon_device_t beacon_status;

void RF_Session_Init( uint8_t, uint32_t );
void RF_Session_End(  void );
void RF_Kick( 		  void );
void RF_Tx( 		  beacon_device_t * );

#endif /* SYSTEM_UTILITIES_RF_CONTROLLER_H_ */
