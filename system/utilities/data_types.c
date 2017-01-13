/*
 * data_types.c
 *
 *  Created on: Jan 11, 2017
 *      Author: Matthew Fonken
 */

/* Own header */
#include "data_types.h"

/* Note: These functions are designed for circular buffering.
 *		 Each has only one extra instruction. It should be fine.
 */

uint8_t bufferAdd( buffer_t * b, uint8_t v )
{
    b->buffer[( b->index++ ) & BUFF_SIZE_MASK] = v;
    return b->index;
}

uint8_t bufferRead( buffer_t * b, uint8_t i )
{
    return b->buffer[( b->index + ( ~i ) ) & BUFF_SIZE_MASK];
}
