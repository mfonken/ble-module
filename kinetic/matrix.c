/***********************************************************************************************//**
 * \file   matrix.c
 * \brief  Matrix Math
 ***************************************************************************************************
 *      Author: Matthew Fonken
 **************************************************************************************************/

/* Own header */
#include "matrix.h"

/***********************************************************************************************//**
 * @addtogroup Application
 * @{
 **************************************************************************************************/

/***********************************************************************************************//**
 * @addtogroup kinetic
 * @{
 **************************************************************************************************/

/***************************************************************************************************
 Static Function Definitions
 **************************************************************************************************/

/***********************************************************************************************//**
 *  \brief  Tait-Bryan Z > X' > Y" matrix transformation
 *  \param[out] Transformed matrix
 *  \param[in] x Matrix to transform
 *  \param[in] rot Tait-Bryan angles
 *  \param[in] reverse invert angles
 ***************************************************************************************************
 * NOTE: Reversing angles does not invert transformation matrix!
 ***************************************************************************************************
 * FORMULA:
 \f{eqnarray*}{
     &\mathbf{A} = \left[
         \begin{matrix}
         \cos(a)\cos(c) - \sin(a)\sin(b)\sin(c) & -\cos(b)\sin(a) & \cos(a)\sin(c) + \cos(c)\sin(a)\sin(b)\\
         \cos(c)sin(a) + cos(a)\sin(b)\sin(c) & \cos(a)\cos(b) & \sin(a)\sin(c) - \cos(a)\cos(c)\sin(b) \\
         -\cos(b)\sin(c) & \sin(b) & \cos(b)\cos(c)\\
         \end{matrix}
         \right] \\
     &\mathbf{y} = \mathbf{A}\mathbf{x}
 \f}
 **************************************************************************************************/
vec3_t * zxyTransform(  vec3_t *x,
                        ang3_t *rot)
{
    /* Extract angles */
    double a = rot->a;
    double b = rot->b;
    double c = rot->c;

    /* Transformation Matrix */
    double A[3][3];
    A[0][0] = ( cos( a ) * cos( c ) ) - ( sin( a ) * sin( b ) * sin( c ) );
    A[0][1] = - cos( b ) * sin( a );
    A[0][2] = ( cos( a ) * sin( c ) ) + ( cos( c ) * sin( a ) * sin( b ) );
    A[1][0] = ( cos( c ) * sin( a ) ) + ( cos( a ) * sin( b ) * sin( c ) );
    A[1][1] =   cos( a ) * cos( b );
    A[1][2] = ( sin( a ) * sin( c ) ) - ( cos( a ) * cos( c ) * sin( b ) );
    A[2][0] = - cos( b ) * sin( c );
    A[2][1] =   sin( b );
    A[2][2] =   cos( b ) * cos( c );

    /* Transformed Vector */
    double y[3];
    for( uint8_t i = 0; i < 3; i++ )
    {
        y[i] = ( A[i][0] * x->i ) + ( A[i][1] * x->j ) + ( A[i][2] * x->k );
    }
    vec3_t yvec;
    yvec.i = y[0];
    yvec.j = y[1];
    yvec.k = y[2];
    return &yvec;
}


/***********************************************************************************************//**
 *  \brief  Tait-Bryan Z > X' > Y" matrix transformation
 *  \param[out] Transformed matrix
 *  \param[in] x Matrix to transform
 *  \param[in] rot Tait-Bryan angles
 *  \param[in] reverse invert angles
 ***************************************************************************************************
 * NOTE: Reversing angles does not invert transformation matrix!
 ***************************************************************************************************
 * FORMULA:
 \f{eqnarray*}{
    &\mathbf{A} = \left[
     \begin{matrix}
        \cos(a)\cos(c) + \sin(a)\sin(b)\sin(c) & \cos(c)\sin(a)\sin(b) - \cos(a)\sin(c) & \cos(b)\sin(a) \\
        \cos(b)\sin(c) & \cos(b)\cos(c) & -\sin(b) \\
        \cos(a)sin(b)sin(c) - \cos(c)\sin(a) & \cos(a)\cos(c)\cos(b) + \sin(a)\sin(c) & \cos(a)\cos(b) \\
     \end{matrix}
    \right] \\
    &\mathbf{y} = \mathbf{A}\mathbf{x}
 \f}
 ***************************************************************************************************/
vec3_t * yxzTransform( vec3_t * x,
                       ang3_t * rot,
                       bool reverse)
{
    /* Extract angles */
    double a = rot->a;
    double b = rot->b;
    double c = rot->c;
    if( reverse )
    {
        a *= -1;
        b *= -1;
        c *= -1;
    }

    /* Transformation Matrix */
    double A[3][3];
    A[0][0] = ( cos( a ) * cos( c ) ) + ( sin( a ) * sin( b ) * sin( c ) );
    A[0][1] = ( cos( c ) * sin( a ) * sin( b ) ) - ( cos( a ) * sin( c ) );
    A[0][2] =   cos( b ) * sin( a );
    A[1][0] =   cos( b ) * sin( c );
    A[1][1] =   cos( b ) * cos( c );
    A[1][2] = - sin( b );
    A[2][0] = ( cos( a ) * sin( b ) * sin( c ) ) - ( cos( c ) * sin( a ) );
    A[2][1] = ( cos( a ) * cos( c ) * cos( b ) ) + ( sin( a ) * sin( c ) );
    A[2][2] =   cos( a ) * cos( b );

    /* Transformed Vector */
    double y[3];
    for( uint8_t i = 0; i < 3; i++ )
    {
        y[i] = ( A[i][0] * x->i ) + ( A[i][1] * x->j ) + ( A[i][2] * x->k );
    }
    vec3_t *yvec;
    yvec->i = y[0];
    yvec->j = y[1];
    yvec->k = y[2];
    return yvec;
}

/***********************************************************************************************//**
 *  \brief  Subract two 3D vectors
 *  \param[in] x Subracted from and returned
 *  \param[in] y Values to subract
 ***************************************************************************************************
 * FORMULA:

 \f{eqnarray*}{
    &x_\hat{i} = x_\hat{i} - y_\hat{i} \\
    &x_\hat{j} = x_\hat{j} - y_\hat{j} \\
    &x_\hat{k} = x_\hat{k} - y_\hat{k}
 \f}
 **************************************************************************************************/
void subtractvec3_t( vec3_t * x,
                     vec3_t * y )
{
    x->i = x->i - y->i;
    x->j = x->j - y->j;
    x->k = x->k - y->k;
}
/***********************************************************************************************//**
 *  \brief  Return length of 3D vector
 *  \param[out] Length of vector
 *  \param[in] x Vector measured
 ***************************************************************************************************
 * FORMULA:
 \f{eqnarray*}{
    ||\mathbf{v}|| = \sqrt{v_\hat{i}^2 + v_\hat{j}^2 + v_\hat{k}^2}\\
 \f}
 **************************************************************************************************/
double lengthOfvec3_t( vec3_t * vec )
{
    double i_2 = vec->i * vec->i;
    double j_2 = vec->j * vec->j;
    double k_2 = vec->k * vec->k;
    return sqrt( i_2 + j_2 + k_2 );
}

double lengthOfvec2_t( vec2_t * vec )
{
    double i_2 = vec->i * vec->i;
    double j_2 = vec->j * vec->j;
    return sqrt( i_2 + j_2 );
}

/***********************************************************************************************//**
 *  \brief  Normalize a 3D Vector
 *  \param[in] vec Vector to normalize, returned as self
 ***************************************************************************************************
 * FORMULA:
 \f{eqnarray*}{
    \mathbf{v_{norm}} = \frac{\mathbf{v}}{||\mathbf{v}||}
 \f}
 **************************************************************************************************/
void normalizevec3_t( vec3_t * vec )
{
    double length = lengthOfvec3_t( vec );
    vec->i /= length;
    vec->j /= length;
    vec->k /= length;
}
/***********************************************************************************************//**
 *  \brief  Distance between two 2D coordinates
 *  \param[out] Distance between
 *  \param[in] a First coordinate
 *  \param[in] b Second coordinate
 ***************************************************************************************************
 * FORMULA:
 \f{eqnarray*}{
 d = \sqrt{(x_b^2 - x_a^2) + (y_b^2 - y_a^2)}
 \f}
 **************************************************************************************************/
double get2dDistance( cartesian2_t *a, cartesian2_t *b )
{
    double a_x = a->x;
    double a_y = a->y;
    double b_x = b->x;
    double b_y = b->y;

    return sqrt( ( ( b_x * b_x ) - ( a_x * a_x ) ) + ( ( b_y * b_y ) - ( a_y * a_y ) ) );
}

void multiplyVec3x1( double a[3][3], double b[3], double c[3] )
{
    double r[3] = {0,0,0};
    for( int i = 0; i < 3; i++ )
    {
        r[i] = a[i][0] * b[0] + a[i][1] * b[1] + a[i][2] * b[2];
    }
    for( int i = 0; i < 3; i++ )
    {
        c[i] = r[i];
    }
}

void multiplyVec3x3( double a[3][3], double b[3][3], double c[3][3] )
{
    double r[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
    for( int i = 0; i < 3; i++ )
    {
        for( int j = 0; j < 3; j++ )
        {
            for( int k = 0; k < 3; k++ )
            {
                r[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    for( int i = 0; i < 3; i++ )
    {
        for( int j = 0; j < 3; j++ )
        {
            c[i][j] = r[i][j];
        }
    }
}

void getRotationX( double v[3][3], double angle )
{
    double c = cos( angle );
    double s = sin( angle );
    double r[3][3] =
    {
        {  1,  0,  0 },
        {  0,  c, -s },
        {  0,  s,  c }
    };
    for( int i = 0; i < 3; i++ )
    {
        for( int j = 0; j < 3; j++ )
        {
            v[i][j] = r[i][j];
        }
    }
}

void getRotationY( double v[3][3], double angle )
{
    double c = cos( angle );
    double s = sin( angle );
    double r[3][3] =
    {
        {  c,  0,  s },
        {  0,  1,  0 },
        { -s,  0,  c }
    };
    for( int i = 0; i < 3; i++ )
    {
        for( int j = 0; j < 3; j++ )
        {
            v[i][j] = r[i][j];
        }
    }
}

void getRotationZ( double v[3][3], double angle )
{
    double c = cos( angle );
    double s = sin( angle );
    double r[3][3] =
    {
        {  c, -s,  0 },
        {  s,  c,  0 },
        {  0,  0,  1 }
    };
    for( int i = 0; i < 3; i++ )
    {
        for( int j = 0; j < 3; j++ )
        {
            v[i][j] = r[i][j];
        }
    }
}

/** @} (end addtogroup kinetic) */
/** @} (end addtogroup Application) */
