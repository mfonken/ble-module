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

#define	diff( a, b ) ( a - b > 0 ) ? a - b : b - a

beacon_t 	beacons[MAX_CENTROIDS];
uint8_t  	num_tracked;
uint8_t	 	map[MAX_CENTROIDS];
buffer_t 	camera_buffer;
centroids_t	centroids;

uint32_t	beacon_vector[2];

/********************************************************************
 * Camera Communication Functions
 *******************************************************************/

void Camera_Init( void )
{
	Print_Char( CAMERA_INIT );
	uint8_t n = Read_Char();
	num_tracked = 0;
}

void Camera_Print( uint8_t cmd )
{
	char out[10];
	int l = 0;
	uint8_t n = num_tracked;
	Print_String("Found ", 6);
	l = sprintf(out, "%d",n);
	Print_String( ( uint8_t * )out, l);
	Print_String(" centroids.\r\n", 13);
	for(int i = 0; i < n; i++)
	{
		l = sprintf(out, "%d",i);
		Print_String( ( uint8_t * )out, l);
		Print_Char('>');
		l = sprintf(out, "%d", beacons[i].centroid.x);
		Print_String( ( uint8_t * )out, l);
		Print_Char('|');
		l = sprintf(out, "%d", beacons[i].centroid.y);
		Print_String( ( uint8_t * )out, l);
		Print_Char('\r');
		Print_Char('\n');
	}
}

void Camera_Buffer( uint8_t in )
{
	/* Add new value and check if the packet is full */
	uint8_t index = bufferAdd( &camera_buffer, in );
	if( index == ( centroids.count * 2 ) )
	{
		for( int i = 0; i < centroids.count; i++ )
		{
			centroids.centroid[i].x = bufferRead( &camera_buffer, i * 2 );
			centroids.centroid[i].y = bufferRead( &camera_buffer, i * 2 + 1 );
		}
		camera_buffer.index = 0;
	}
}

uint8_t Camera_Check( uint8_t in )
{
	switch( in )
	{
		case CENTROID_HEAD:
			/* Allow UART rx data to come here */
			disableUARTInterrupt();
			centroids.count = Read_Char();
			enableUARTInterrupt();
			return centroids.count;
		default:
			return CAM_NULL_CMD;
	}
}

/********************************************************************
 * Beacon Tracking Functions
 *******************************************************************/

void Beacon_Add( centroid_t * a )
{
	num_tracked++;
	Beacon_Copy( &beacons[num_tracked].centroid, a );
	beacons[num_tracked].persistence = 1;
	beacons[num_tracked].timestamp = timestamp();
	map[num_tracked] = num_tracked;
}


void Beacon_Check( void )
{
	for( int i = 0; i < centroids.count; i++ )
	{
		bool claimed = false;
		for( int j = 0; j < num_tracked; j++ )
		{
			if( Beacon_Compare( &beacons[map[j]].centroid, &centroids.centroid[i] ) )
			{
				claimed = true;
				Beacon_Update( j, &centroids.centroid[i]);
			}
		}
		if( !claimed )
		{
			Beacon_Add( &centroids.centroid[i] );
		}
	}
}

bool Beacon_Compare( centroid_t * a, centroid_t * b ){
	/*TODO: Complete this compare check */
	if( diff( a->x, b->x ) <= MAX_X_DIFF &&
		diff( a->y, b->y ) <= MAX_Y_DIFF  )
	{
		return true;
	}
	return false;
}

/* Call this on sync */
void Beacon_Compose( void )
{
	beacon_vector[0] = beacons[map[1]].centroid.x - beacons[map[0]].centroid.x;
	beacon_vector[1] = beacons[map[1]].centroid.y - beacons[map[0]].centroid.y;
	sensors.synced.beacon[0] = beacon_vector[0];
	sensors.synced.beacon[1] = beacon_vector[1];
}

void Beacon_Copy( centroid_t * a, centroid_t * b)
{
	a->x = b->x;
	a->y = b->y;
}

void Beacon_Sort( uint8_t starting_index )
{
	/* Sudo resort by persistence */
	if( starting_index > 1 )
		{
			for( int i = starting_index; i < num_tracked; i++ )
			{
				if( beacons[map[i]].persistence < beacons[map[i+1]].persistence )
				{
					/* Swap index in map */
					uint8_t temp = map[i];
					map[i] = map[i+1];
					map[i+1] = temp;
				}
			}
		}
}

void Beacon_Perge( void )
{
	uint8_t perged = 0;
	for( int i = 0; i < num_tracked; i++ )
	{
		if( beacons[map[i]].timestamp > MAX_TRACK_AGE )
		{
			map[i] = map[i + 1 + perged];
			perged++;
		}
	}
	num_tracked -= perged;
}

void Beacon_Update( uint8_t index, centroid_t * b )
{
	Beacon_Copy( &beacons[map[index]].centroid, b );
	beacons[map[index]].timestamp = timestamp();
	beacons[map[index]].persistence++;

	Beacon_Perge();
	Beacon_Sort( index );
}
