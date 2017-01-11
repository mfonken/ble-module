/*
 * cam_controller.h
 *
 *  Created on: Jan 3, 2017
 *      Author: Matthew Fonken
 */

#ifndef SENSORS_CAMERA_CAM_CONTROLLER_H_
#define SENSORS_CAMERA_CAM_CONTROLLER_H_

#include <stdint.h>

#define CAM_UART		USART0

#define CAMERA_INIT		0xab
#define CENTROID_HEAD	0xee

#define MAX_CENTROIDS	6

typedef struct
{
	uint8_t x;
	uint8_t y;
} centroid_t;

typedef struct
{
	uint8_t n;
	centroid_t c[MAX_CENTROIDS];
} centroids_t;

extern centroids_t centroids;

void Camera_Init( void );
void Camera_Read( void );
void readCentroidData( centroids_t * centroids );


#endif /* SENSORS_CAMERA_CAM_CONTROLLER_H_ */
