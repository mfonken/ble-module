/*
 * app.h
 *
 *  Created on: Jan 9, 2017
 *      Author: Matthew Fonken
 */

#ifndef SYSTEM_APP_H_
#define SYSTEM_APP_H_

#include <stdint.h>
#include <math.h>

/* Interrupt header */
#include "app_interrupts.h"

/* Sensor type headers */
#include "sensor_data_types.h"

#define SYNC_TIMER			TIMER0
#define SYNC_TIMER_PERIOD 	100

#define FORCE_TIMER			TIMER1
#define FORCE_TIMER_PERIOD 	100

#define _2D_MODE_DEFAULT 	false
#define _3D_MODE_DEFAULT	true
#define _SLEEP_MODE_DEFAULT	false

extern app_t 			mode;
//extern sync_t 			sync;
extern sensor_data_t 	sensors;

/* App */
void app_init( 			void );
void app( 				void );
void appModeSet( 	 app_t * );

void autoDetectMode( void );

void enableSyncTimer( 	sync_t * );
void disableSyncTimer( 		void );
void enableStylusSensors( 	void );
void disableStylusSensors( 	void );
void enableSpatialSensors( 	void );
void disableSpatialSensors( void );
void enterSleepMode( 		void );
void exitSleepMode( 		void );

#endif /* SYSTEM_APP_H_ */
