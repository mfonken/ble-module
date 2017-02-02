/*
 * camera_sp.c
 *
 *  Created on: Jan 8, 2017
 *      Author: Matthew Fonken
 */

#include "cam_controller.h"

#include "usart_sp.h"

#define	diff( a, b ) ( a - b > 0 ) ? a - b : b - a

beacon_t 	beacons[MAX_CENTROIDS];
uint8_t  	num_tracked;
uint8_t	 	map[MAX_CENTROIDS];
buffer_t 	camera_buffer;
centroids_t	centroids;

uint32_t	beacon_vector[2];

uint8_t		centroids_to_buffer;

uint8_t     last_index;

/****************************************************************//**
 * Camera Communication Functions
 *******************************************************************/

void Camera_Init( void )
{
    /* TODO: Needs to be HW enabled and probably init every use */
    Print_Char( CAMERA_INIT );
    uint8_t n = Read_Char();
    num_tracked = 0;
    centroids_to_buffer = 0;
    bufferReset( &camera_buffer );
}

void Camera_Enable( void )
{
    SYSCTL_Enable_Camera();
    NVIC_EnableIRQ(USART0_RX_IRQn);
    //RF_Session_Init( DEFAULT_BEACON_INTENSITY, DEFAULT_BEACON_DURATION );
}

void Camera_Disable( void )
{
    SYSCTL_Disable_Camera();
    NVIC_DisableIRQ(USART0_RX_IRQn);
    //RF_Session_End();
}

uint8_t Camera_Buffer( uint8_t in )
{
	//Print_Hex( in );
	return bufferAdd( &camera_buffer, in );
}

uint8_t Camera_Check( void )
{
	disableUARTInterrupt();
    uint8_t index = last_index;
    uint8_t end = camera_buffer,index;
	while( index != end )
	{
		if( bufferRead( &camera_buffer, index ) == CENTROID_HEAD )
		{
			uint8_t n = bufferRead( &camera_buffer, index + 1 );
            if( n < 2 )
            {
                last_index = index + 1;
                goto invalid;
            }
            if( n == B_NULL ) goto null;
			centroids.count = n;
			for( uint8_t i = 0 ; i < n; i ++ )
			{
                uint8_t index1,index2;
                index1 = index + ( i * 2 ) + 2;
                index2 = index1 + 1;
                index1 &= BUFF_SIZE_MASK;
                index2 &= BUFF_SIZE_MASK;
                if ( index1 == camera_buffer.index || index2 == camera_buffer.index)
                {
					centroids.count = 0;
                    last_index = index;
					enableUARTInterrupt();
					return CAM_NULL_CMD;
				}
				centroids.centroid[i].x = bufferRead( &camera_buffer, read_index );
				centroids.centroid[i].y = bufferRead( &camera_buffer, read_index + 1 );
			}
			Beacon_Check();
            bufferReset( &camera_buffer );
			enableUARTInterrupt();
			return centroids.count;
		}
null:
        index++;
        index &= BUFF_SIZE_MASK;
	}
invalid:
	enableUARTInterrupt();
	return CAM_NULL_CMD;
}

/********************************************************************
 * Beacon Tracking Functions
 *******************************************************************/

void Beacon_Add( centroid_t * a )
{
	Beacon_Copy( &beacons[num_tracked].centroid, a );
	beacons[num_tracked].persistence = 1;
	beacons[num_tracked].timestamp = timestamp();
	map[num_tracked] = num_tracked;
	num_tracked++;
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
                j++;
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
	uint8_t c = diff( a->x, b->x ) ;
	uint8_t d = diff( a->y, b->y ) ;
	if( c <= MAX_X_DIFF &&
		d <= MAX_Y_DIFF  )
	{
		return true;
	}
	return false;
}

/* Call this on sync */
bool Beacon_Compose( cartesian2_t vis[2] )
{
	if( centroids.count >= 2 )
	{
		vis[0].x = beacons[map[0]].centroid.x;
		vis[0].y = beacons[map[0]].centroid.y;
		vis[1].x = beacons[map[1]].centroid.x;
		vis[1].y = beacons[map[1]].centroid.y;
		centroids.count = 0;
		return true;
	}
	return false;
}

void Beacon_Copy( centroid_t * a, centroid_t * b)
{
	a->x = b->x;
	a->y = b->y;
}

void Beacon_Sort( void )
{
	/* Sudo resort by persistence */
    for( int i = 0; i < ( num_tracked - 1 ); i++ )
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

void Beacon_Perge( void )
{
	uint8_t perged = 0;
	for( int i = 0; i < num_tracked; i++ )
	{
		if( ( timestamp() - beacons[map[i]].timestamp ) > MAX_TRACK_AGE )
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
	Beacon_Sort();
}
