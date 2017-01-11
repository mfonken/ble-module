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

#define SYNC_TIMER_PERIOD 	100
#define FORCE_TIMER_PERIOD 	100

#define _3D_MODE_LOC			0
#define _2D_MODE_LOC			1
#define _LP_MODE_LOC			2

typedef struct
{
	uint8_t _2d:1;			/**< 2D Mode 				>*/
	uint8_t _3d:1;			/**< 3D Mode 				>*/
	uint8_t _sl:1;			/**< Sleep Mode				>*/
	uint8_t RESERVED:5;
} app_t;

extern app_t mode;

void app_init( 			void );
void app( 				void );
void appModeSet( 	 app_t * );
void enter2DMode( 		void );
void exit2DMode( 		void );
void enter3DMode( 		void );
void exit3DMode( 		void );
void enterSleepMode( 	void );
void exitSleepMode( 	void );
void registerTimer( uint32_t );
void releaseTimer( 		void );

#endif /* SYSTEM_APP_H_ */
