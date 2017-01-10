/*
 * app.h
 *
 *  Created on: Jan 9, 2017
 *      Author: Matthew Fonken
 */

#ifndef SYSTEM_APP_H_
#define SYSTEM_APP_H_

#include <stdint.h>

#define CAMERA_TIMER_COUNT_TOP 	100

#define _3D_MODE_LOC			0
#define _2D_MODE_LOC			1
#define _LP_MODE_LOC			2

typedef struct
{
	uint8_t _2d:1;			/**< 2D Mode 				>*/
	uint8_t _3d:1;			/**< 3D Mode 				>*/
	uint8_t _lp:1;			/**< Low Power Mode			>*/
	uint8_t RESERVED:5;
} app_t;

extern app_t mode;

void appInit( void );
void appModeSet( app_t * );
void initialize3DMode( void );
void deinitialize3DMode( void );
void registerTimer( uint32_t );
void releaseTimer( void );

#endif /* SYSTEM_APP_H_ */
