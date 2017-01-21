/*
 * CPT112S.c
 *
 *  Created on: Jan 10, 2017
 *      Author: Matthew Fonken
 */

/* Own header */
#include "CPT112S.h"
#include "i2c_sp.h"

void Touch_Init( void )
{

}

uint8_t * Touch_Read( void )
{
    uint8_t i2c_read_data[3];
    I2C_Read( TOUCH_CTR_ADDR, i2c_read_data, 3 );
    return i2c_read_data;
}
