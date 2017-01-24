/*! \file spatial.h
    \brief Spatial Orientation Main\r\n
 
 NOTES:
 - IMU is for rotational data ONLY!
 - Positional data comes from camera triangulation module and beacons
 - Right-handed rotation
 
  Created by Matthew Fonken on 10/8/16.
 */

#ifndef kinetic_h
#define kinetic_h

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>
#include <math.h>

/* Math headers */
#include "kalman.h"
#include "matrix.h"

/* Utilities */
#include "usart_sp.h"
#include "clock_sp.h"

/* Sensors headers */
#include "LSM9DS1.h"

/* Types */
#include "sensor_data_types.h"
#include "kinetic_types.h"

    
/***********************************************************************************************//**
 * \defgroup Kinetic Motion Code
 * \brief Functions for Motion Analysis
 **************************************************************************************************/

/***********************************************************************************************//**
 * @addtogroup Application
 * @{
 **************************************************************************************************/

/***********************************************************************************************//**
 * @addtogroup kinetic
 * @{
 **************************************************************************************************/

/***************************************************************************************************
 Global Definitions
***************************************************************************************************/
    
/** Absolute value */
#define 	absl(x) x > 0 ? x:-x

/** Half PI */
#define     PI      			3.141596
#define 	RAD_TO_DEG 			180 / PI
    
/** Initial normal unit vector to beacon plane */
#define     VISION_ZSTATE_IHAT  0
#define     VISION_ZSTATE_JHAT  0
#define     VISION_ZSTATE_KHAT -1
    
/** Augmentation skew */
#define     D_AUG               0
    
/** Physical distance between beacons */
#define     BEACON_DISTANCE     1
    
/** Vision camera dimensions */
#define     VISION_WIDTH        640
#define     VISION_HEIGHT       480
    
/** Vision camera center */
#define     VISION_CENTER_X     VISION_WIDTH  / 2
#define     VISION_CENTER_Y     VISION_HEIGHT / 2
extern LSM9DS1_t this;

/***************************************************************************************************
Function Declarations
***************************************************************************************************/
    
/***********************************************************************************************//**
 *  \brief  Initialize Kinetic Sensors
 **************************************************************************************************/
void Kinetic_Init( LSM9DS1_t *, kinetic_t * );
    
/***********************************************************************************************//**
 *  \brief  Initialize Filters for Kinetic Data
 **************************************************************************************************/
void Filters_Init( LSM9DS1_t * imu, kinetic_t * kinetics );
    
/***********************************************************************************************//**
 *  \brief  Update rotation filter data
 **************************************************************************************************/
void Kinetic_Update_Rotation( LSM9DS1_t * imu, kinetic_t * kinetics );
    
/***********************************************************************************************//**
 *  \brief  Update position filter data
 **************************************************************************************************/
void Kinetic_Update_Position( LSM9DS1_t * imu, kinetic_t * kinetics, centroid_t vis[2] );
    
/***********************************************************************************************//**
 *  \brief  Initialize Filters for Kinetic Data
 *  \param[in] dvec Beacon positional vector to augment
 *  \param[in] a Tait-Bryan angles to augement by
 **************************************************************************************************/
vec3_t *dAugment( vec3_t *, ang3_t * );

/** @} (end addtogroup kinetic) */
/** @} (end addtogroup Application) */
    
#ifdef __cplusplus
};
#endif
#endif /* kinetic_h */
