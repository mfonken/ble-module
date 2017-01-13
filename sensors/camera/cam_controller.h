/*
 * cam_controller.h
 *
 *  Created on: Jan 3, 2017
 *      Author: Matthew Fonken
 */

#ifndef SENSORS_CAMERA_CAM_CONTROLLER_H_
#define SENSORS_CAMERA_CAM_CONTROLLER_H_

#include <stdint.h>
#include "data_types.h"

#define CAM_UART		USART0
#define CAM_NULL_CMD	0xff

#define CAMERA_INIT		0xab
#define CENTROID_HEAD	0xee

#define MAX_CENTROIDS	6

typedef struct
{
	uint8_t 	x;
	uint8_t 	y;
} centroid_t;

typedef struct
{
	uint8_t 	n;
	centroid_t 	c[MAX_CENTROIDS];
} centroids_t;

extern centroids_t centroids;
extern buffer_t camera_buffer;

void 	Camera_Init( 		 void );
void 	Camera_Read( 		 void );
uint8_t Camera_Check(  uint8_t in );
void 	Camera_Buffer( uint8_t in );


#endif /* SENSORS_CAMERA_CAM_CONTROLLER_H_ */
