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

void Multiply_Vec_3x1( double a[3][3], double b[3], double c[3] )
{
    for( int i = 0; i < 3; i++ )
    {
        c[i] = a[i][0] * b[0] + a[i][1] * b[1] + a[i][2] * b[2];
    }
}

void Quaternion_To_Matrix(quaternion_t * quat, double m[3][3])
{
    double wx, wy, wz, xx, yy, yz, xy, xz, zz, x2, y2, z2;
    
    x2 = quat->x + quat->x; y2 = quat->y + quat->y;
    z2 = quat->z + quat->z;
    xx = quat->x * x2; xy = quat->x * y2; xz = quat->x * z2;
    yy = quat->y * y2; yz = quat->y * z2; zz = quat->z * z2;
    wx = quat->w * x2; wy = quat->w * y2; wz = quat->w * z2;
    
    m[0][0] = 1.0 - (yy + zz);
    m[1][0] = xy - wz;
    m[2][0] = xz + wy;
    
    m[0][1] = xy + wz;
    m[1][1] = 1.0 - (xx + zz);
    m[2][1] = yz - wx;
    
    m[0][2] = xz - wy;
    m[1][2] = yz + wx;
    m[2][2] = 1.0 - (xx + yy);
}

/* See - http://www.gamasutra.com/view/feature/131686/rotating_objects_using_quaternions.php */
/* Generic intrinsic euler angles to quaternion conversion */
void Euler_To_Quaternion( double a[3], quaternion_t * quat )
{
    double half_roll, half_pitch, half_yaw;
    half_roll   = a[0] / 2;
    half_pitch  = a[1] / 2;
    half_yaw    = a[2] / 2;
    double cr, cp, cy, sr, sp, sy, cpcy, spsy, spcy, cpsy;
    cr = cos( half_roll  );
    cp = cos( half_pitch );
    cy = cos( half_yaw   );
    sr = sin( half_roll  );
    sp = sin( half_pitch );
    sy = sin( half_yaw   );
    cpcy = cp * cy;
    spsy = sp * sy;
    spcy = sp * cy;
    cpsy = cp * sy;
    quat->w = cr * cpcy + sr * spsy;
    quat->x = sr * cpcy - cr * spsy;
    quat->y = cr * spcy + sr * cpsy;
    quat->z = cr * cpsy - sr * spcy;
}

/* See - https://blog.molecular-matters.com/2013/05/24/a-faster-quaternion-vector-multiplication/ */
void Rotate_Vector_By_Quaternion(double v[3], quaternion_t * q, double r[3])
{
    double u[3];
    u[0] = q->x;
    u[1] = q->y;
    u[2] = q->z;
    double s = q->w;
    double a[3], b[3], c[3], uv[3];
    cross3( u, v, uv );
    mul3( 2, uv, a );
    mul3( s, u, b );
    add33( v, a, b, r );
}

/* Double quaternion Hamilton multiplication (Generic) */
void Quaternion_Combine(quaternion_t * a, quaternion_t * b, quaternion_t * c, quaternion_t * d)
{
    double A, B, C, D, E, F, G, H;
    A = ( a->w + a->x ) * ( b->w + b->x );
    B = ( a->z - a->y ) * ( b->y - b->z );
    C = ( a->w - a->x ) * ( b->y + b->z );
    D = ( a->y + a->z ) * ( b->w - b->x );
    E = ( a->x + a->z ) * ( b->x + b->y );
    F = ( a->x - a->z ) * ( b->x - b->y );
    G = ( a->w + a->y ) * ( b->w - b->z );
    H = ( a->w - a->y ) * ( b->w + b->z );
    d->w = B + ( -E - F + G + H ) / 2;
    d->x = A - (  E + F + G + H ) / 2;
    d->y = C + (  E - F + G - H ) / 2;
    d->z = D + (  E - F - G + H ) / 2;
    
    A = ( d->w + d->x ) * ( c->w + c->x );
    B = ( d->z - d->y ) * ( c->y - c->z );
    C = ( d->w - d->x ) * ( c->y + c->z );
    D = ( d->y + d->z ) * ( c->w - c->x );
    E = ( d->x + d->z ) * ( c->x + c->y );
    F = ( d->x - d->z ) * ( c->x - c->y );
    G = ( d->w + d->y ) * ( c->w - c->z );
    H = ( d->w - d->y ) * ( c->w + c->z );
    d->w = B + ( -E - F + G + H ) / 2;
    d->x = A - (  E + F + G + H ) / 2;
    d->y = C + (  E - F + G - H ) / 2;
    d->z = D + (  E - F - G + H ) / 2;
}

/* Generic vector3 manipulations */
void cross3( double u[3], double v[3], double r[3] )
{
    r[0] = u[1]*v[2] - u[2]*v[1];
    r[1] = u[2]*v[0] - u[0]*v[2];
    r[2] = u[0]*v[1] - u[1]*v[0];
}

void mul3( int m, double v[3], double r[3] )
{
    r[0] = m * v[0];
    r[1] = m * v[1];
    r[2] = m * v[2];
}
/* Add three vectors */
void add33( double u[3], double v[3], double w[3], double r[3])
{
    r[0] = u[0] + v[0] + w[0];
    r[1] = u[1] + v[1] + w[1];
    r[2] = u[2] + v[2] + w[2];
}

/** @} (end addtogroup kinetic) */
/** @} (end addtogroup Application) */
