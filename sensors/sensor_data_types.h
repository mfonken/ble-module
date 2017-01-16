/*
 * sensor_data_types.h
 *
 *  Created on: Jan 15, 2017
 *      Author: Matthew Fonken
 */

#ifndef SENSORS_SENSOR_DATA_TYPES_H_
#define SENSORS_SENSOR_DATA_TYPES_H_

typedef struct
{
	uint8_t _2d:1;			/**< 2D Mode 				>*/
	uint8_t _3d:1;			/**< 3D Mode 				>*/
	uint8_t _sl:1;			/**< Sleep Mode				>*/
	uint8_t RESERVED:5;
} app_t;

/* 8 Bytes */
typedef struct
{
	uint8_t 	accel[3];
	uint8_t 	gyro[3];
	uint8_t 	mag[3];
	uint8_t 	beacon[2];
	uint32_t 	timestamp;
} synced_sensor_data_t;

/* 6 Bytes */
typedef struct
{
	uint16_t 	value;
	uint32_t 	timestamp;
} force_t;

/* 1 Byte */
typedef struct
{
	uint8_t		front:1;
	uint8_t		middle:1;
	uint8_t		back:1;
	uint8_t		left:1;
	uint8_t		right:1;
	uint8_t		RESERVED:3;
} touch_pad_t;

/* 10 Bytes */
typedef struct
{
	touch_pad_t	pad;
	uint32_t 	slider;
	uint32_t 	timestamp;
	uint8_t		RESERVED;
} touch_t;

/* 24 Bytes */
typedef struct
{
	synced_sensor_data_t	synced;
	force_t					force;
	touch_t					touch;
} sensor_data_t;

#endif /* SENSORS_SENSOR_DATA_TYPES_H_ */
