//
//  SysCtlr.c
//  
//
//  Created by Matthew Fonken on 12/17/16.
//
//

#include "PCA9534A.h"

#include "i2c_sp.h"

uint8_t curr_port;

void SysCtlr_Init( void )
{
    uint8_t i2c_write_data[2] = { CONFIG_REGISTER, SysCtlr_PORT_DIR };
    I2C_Write( SysCtlr_ADDR, i2c_write_data, 2 );

    SysCtlr_Set( SysCtlr_OUTPUT_DEFAULT );
}
uint8_t * SysCtlr_Get(  void )
{
    return 0;
}

void SysCtlr_Set( uint8_t port )
{
    uint8_t i2c_write_data[2] = { OUTPUT_REGISTER, *( uint8_t * )&port };
    I2C_Write( SysCtlr_ADDR, i2c_write_data, 2 );
    curr_port = port;
}

void SysCtlr_Toggle( uint8_t port )
{
    uint8_t i2c_write_data[2] = { TOGGLE_REGISTER, *( uint8_t * )&port };
    I2C_Write( SysCtlr_ADDR, i2c_write_data, 2 );
}

void Enable_Magnometer( void )
{
	curr_port |=  ( 1 << IMU_CS );
	SysCtlr_Set( curr_port );
}
void Disable_Magnometer( void )
{
	curr_port &= ~( 1 << IMU_CS );
	SysCtlr_Set( curr_port );
}
void Enable_Force_Sensor( void )
{
	curr_port |=  ( 1 << FRC_EN );
	SysCtlr_Set( curr_port );
}
void Disable_Force_Sensor( void )
{
	curr_port &= ~( 1 << FRC_EN );
	SysCtlr_Set( curr_port );
}
void Enable_Camera( void )
{
	curr_port |=  ( 1 << VREG_MODE );
	SysCtlr_Set( curr_port );
}
void Disable_Camera( void )
{
	curr_port &= ~( 1 << VREG_MODE );
	SysCtlr_Set( curr_port );
}
