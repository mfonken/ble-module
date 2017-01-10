//
//  PCA9534A.c
//  
//
//  Created by Matthew Fonken on 12/17/16.
//
//

#include "PCA9534A.h"

#include "i2c_sp.h"

void PCA9534A_Init( void )
{
    uint8_t i2c_write_data[2] = { CONFIG_REGISTER, PCA9534A_PORT_DIR };
    I2C_Write( PCA9534A_ADDR, i2c_write_data, 2 );

    PCA9534A_Set( PCA9534A_OUTPUT_DEFAULT );
}
uint8_t * PCA9534A_Get(  void )
{
    return 0;
}

void PCA9534A_Set( uint8_t port )
{
    uint8_t i2c_write_data[2] = { OUTPUT_REGISTER, *( uint8_t * )&port };
    I2C_Write( PCA9534A_ADDR, i2c_write_data, 2 );
}

void PCA9534A_Toggle( uint8_t port )
{
    uint8_t i2c_write_data[2] = { TOGGLE_REGISTER, *( uint8_t * )&port };
    I2C_Write( PCA9534A_ADDR, i2c_write_data, 2 );
}
