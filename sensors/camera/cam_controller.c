/*
 * camera_sp.c
 *
 *  Created on: Jan 8, 2017
 *      Author: Matthew Fonken
 */

#include "cam_controller.h"
#include "usart_sp.h"
#include <stdio.h>

centroids_t centroids;

void Camera_Init( void )
{
	Print_Char( CAMERA_INIT );
	uint8_t n = Read_Char();
}

void Camera_Read( void )
{
	uint8_t cmd = Read_Char();
	char out[10];
	int l = 0;
	switch( cmd )
	{
		case CENTROID_HEAD:
			readCentroidData( &centroids );
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
			break;
		default:
			Print_Char('~');
			l = sprintf(out, "0x%x", cmd);
			Print_String( ( uint8_t * )out, l);
			Print_Char('\r');
			Print_Char('\n');
	}
}

void readCentroidData( centroids_t * centroids )
{
	uint8_t n = Read_Char();
	centroids->n = n;
    for(int i = 0 ; i < n; i++ )
    {
    	centroids->c[i].x = Read_Char();
    	centroids->c[i].y = Read_Char();
    }
}
