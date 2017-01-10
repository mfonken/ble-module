/***********************************************************************************************//**
 * \file   matrix.h
 * \brief  Matrix Math Header
 ***************************************************************************************************
 *      Author: Matthew Fonken
 **************************************************************************************************/

#ifndef matrix_h
#define matrix_h

/* Standard headers */
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

/* Included types header */
#include "kinetic_types.h"

/***********************************************************************************************//**
 * @addtogroup Application
 * @{
 **************************************************************************************************/

/***********************************************************************************************//**
 * @addtogroup kinetic
 * @{
 **************************************************************************************************/

/***************************************************************************************************
 Static Function Declarations
 **************************************************************************************************/

/***********************************************************************************************//**
 *  \brief  Tait-Bryan Z > X' > Y" matrix transformation
 *  \param[out] Transformed matrix
 *  \param[in] x Matrix to transform
 *  \param[in] rot Tait-Bryan angles
 *  \param[in] reverse invert angles
 ***************************************************************************************************
 * NOTE: Reversing angles does not invert transformation matrix!
 **************************************************************************************************/
vec3_t * zxyTransform(  vec3_t *x, ang3_t *rot, bool reverse);

/***********************************************************************************************//**
 *  \brief  Tait-Bryan Y > X' > Z" matrix transformation
 *  \param[out] Transformed matrix
 *  \param[in] x Matrix to transform
 *  \param[in] rot Tait-Bryan angles
 *  \param[in] reverse invert angles
 ***************************************************************************************************
 * NOTE: Reversing angles does not invert transformation matrix!
 **************************************************************************************************/
vec3_t * yxzTransform( vec3_t * x, ang3_t * rot, bool reverse);

/***********************************************************************************************//**
 *  \brief  Subract two 3D vectors
 *  \param[in] x Subracted from and returned
 *  \param[in] y Values to subract
 **************************************************************************************************/
void subtractvec3_t( vec3_t * x, vec3_t * y );

/***********************************************************************************************//**
 *  \brief  Return length of 3D vector
 *  \param[out] Length of vector
 *  \param[in] x Vector measured
 **************************************************************************************************/
double lengthOfvec3_t( vec3_t * vec );

/***********************************************************************************************//**
 *  \brief  Normalize a 3D Vector
 *  \param[in] vec Vector to normalize, returned as self
 **************************************************************************************************/
void normalizevec3_t( vec3_t * vec );

/***********************************************************************************************//**
 *  \brief  Distance between two 2D coordinates
 *  \param[out] Distance between
 *  \param[in] a First coordinate
 *  \param[in] b Second coordinate
 **************************************************************************************************/
double get2dDistance( cartesian2_t *a, cartesian2_t *b );

/** @} (end addtogroup kinetic) */
/** @} (end addtogroup Application) */

#endif /* matrix_h */
