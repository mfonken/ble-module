/*
 * clock_sp.h
 *
 *  Created on: Jan 15, 2017
 *      Author: Matthew Fonken
 */

#ifndef SYSTEM_UTILITIES_CLOCK_SP_H_
#define SYSTEM_UTILITIES_CLOCK_SP_H_

#include <stdint.h>
#include "em_rtcc.h"
#include "em_cmu.h"

#define SYSCLK_CHANNEL 	1

void	 SYSCLK_Init( void );
uint32_t timestamp( void );

#endif /* SYSTEM_UTILITIES_CLOCK_SP_H_ */
