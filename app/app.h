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

#define SYNC_TIMER			TIMER0
#define SYNC_TIMER_PERIOD 	100

#define FORCE_TIMER			TIMER1
#define FORCE_TIMER_PERIOD 	100

typedef struct
{
	uint8_t _2d:1;			/**< 2D Mode 				>*/
	uint8_t _3d:1;			/**< 3D Mode 				>*/
	uint8_t _sl:1;			/**< Sleep Mode				>*/
	uint8_t RESERVED:5;
} app_t;

extern app_t mode;

/* App */
void app_init( 			void );
void app( 				void );
void appModeSet( 	 app_t * );

/* Modes */
void enter2DMode( 		void );
void exit2DMode( 		void );
void enter3DMode( 		void );
void exit3DMode( 		void );
void enterSleepMode( 	void );
void exitSleepMode( 	void );

#endif /* SYSTEM_APP_H_ */
