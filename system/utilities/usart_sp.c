/***********************************************************************************************//**
 * \file   usart_sp.c
 * \brief  USART Special Functions
 ***************************************************************************************************
 *      Author: Matthew Fonken
 **************************************************************************************************/
#include <stdio.h>
#include <math.h>
/* em headers */
#include "em_usart.h"
#include "em_i2c.h"

#include "usart_sp.h"

/***********************************************************************************************//**
 * @addtogroup Application
 * @{
 **************************************************************************************************/

/***********************************************************************************************//**
 * @addtogroup sp
 * @{
 **************************************************************************************************/

/***************************************************************************************************
 Function Declarations
 **************************************************************************************************/

/***********************************************************************************************//**
 *  \brief  Print character
 *  \param[in] c Character to print
 **************************************************************************************************/
uint8_t Read_Char( void )
{
	uint8_t c = USART_Rx( USART0 );
	return c;
}

/***********************************************************************************************//**
 *  \brief  Print character
 *  \param[in] c Character to print
 **************************************************************************************************/
void Print_Char( uint8_t c )
{
	USART_Tx( USART0, c );
}

/***********************************************************************************************//**
 *  \brief  Print string
 *  \param[in] s String to print
 *  \param[in] len Length of string
 **************************************************************************************************/
void Print_String( char s[] )
{
    uint8_t l = 0;
    for(; l < MAX_STRING_LENGTH; l++)
    {
        if( s[l] == '\0' )
        {
            break;
            l++;
        }
    }
	for( int i = 0; i < l; i++ )
	{
		USART_Tx( USART0, s[i] );
	}
}

void Print_Line( char s[] )
{
    Print_String( s );
    Print_Char('\r');
    Print_Char('\n');
}

/***********************************************************************************************//**
 *  \brief  Print two byte integer
 *  \param[in] v integer to print
 **************************************************************************************************/
void Print_Double_Ascii( double v )
{
	uint8_t output[9];
	uint8_t len = sprintf( ( char * )output,"%.3f", v);
	Print_String( output );
}

/***********************************************************************************************//**
 *  \brief  Print IMU Data
 *  \param[in] motion_data IMU data to print
 **************************************************************************************************/
void Print_IMU( double motion_data[6], bool stripped )
{
	if( !stripped )
		Print_String( "IMU: g(" );
	Print_Char( '0' );
	Print_Double_Ascii( motion_data[0] );
	Print_Char( ',' );
	Print_Double_Ascii( motion_data[1] );
	Print_Char( ',' );
	Print_Double_Ascii( motion_data[2] );
	if( !stripped )
		Print_String( ") | a(" );
	else
		Print_Char( ',' );
	Print_Double_Ascii( motion_data[3] );
	Print_Char( ',' );
	Print_Double_Ascii( motion_data[4] );
	Print_Char( ',' );
	Print_Double_Ascii( motion_data[5] );
	if( !stripped )
		Print_Char( ')' );
	Print_Char( '\n' );
	Print_Char( 0x00 );
}



/** @} (end addtogroup sp) */
/** @} (end addtogroup Application) */
