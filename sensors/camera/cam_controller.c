/*
 * camera_sp.c
 *
 *  Created on: Jan 8, 2017
 *      Author: Matthew Fonken
 */

#include "cam_controller.h"
#include "usart_sp.h"
#include "app_interrupts.h"
#include <stdio.h>

centroids_t centroids;
buffer_t camera_buffer;

void Camera_Init( void )
{
	Print_Char( CAMERA_INIT );
	uint8_t n = Read_Char();
}

void Camera_Print( uint8_t cmd )
{
	char out[10];
	int l = 0;
	uint8_t n = centroids.n;
	Print_String("Found ", 6);
	l = sprintf(out, "%d",n);
	Print_String( ( uint8_t * )out, l);
	Print_String(" centroids.\r\n", 13);
	for(int i = 0; i < n; i++)
	{
		l = sprintf(out, "%d",i);
		Print_String( ( uint8_t * )out, l);
		Print_Char('>');
		l = sprintf(out, "%d",centroids.c[i].x);
		Print_String( ( uint8_t * )out, l);
		Print_Char('|');
		l = sprintf(out, "%d",centroids.c[i].y);
		Print_String( ( uint8_t * )out, l);
		Print_Char('\r');
		Print_Char('\n');
	}
}

uint8_t Camera_Check( uint8_t in )
{
	switch( in )
	{
		case CENTROID_HEAD:
			/* Allow UART rx data to come here */
			disableUARTInterrupt();
			centroids.n = Read_Char();
			enableUARTInterrupt();
			return centroids.n;
		default:
			return CAM_NULL_CMD;
	}
}

void Camera_Buffer( uint8_t in )
{
	uint8_t index = bufferAdd( &camera_buffer, in );
	if( index == ( centroids.n * 2 ) )
	{
		for( int i = 0; i < centroids.n; i++ )
		{
			centroids.c[i].x = bufferRead( &camera_buffer, i * 2 );
			centroids.c[i].y = bufferRead( &camera_buffer, i * 2 + 1 );
		}
		camera_buffer.index = 0;
	}
}
