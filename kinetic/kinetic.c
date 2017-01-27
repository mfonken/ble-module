/***********************************************************************************************//**
 * \file   kinetic.c
 * \brief  Kinetic Motion code
 ***************************************************************************************************
 *  Created on: Nov 13, 2016
 *      Author: Matthew Fonken
 **************************************************************************************************/

/* Own header */
#include "kinetic.h"

/***************************************************************************************************
 Local Variables
 **************************************************************************************************/
/** Local change in time */
//static double         delta_t;

quaternion_t qp, qc, qb, qa;
double  cos_precalc, sin_precalc;

/** Local positional and rotational vectors */

/***********************************************************************************************//**
 *  \brief  Initialize Kinetic Sensors
 **************************************************************************************************/
void Kinetic_Init( LSM9DS1_t * imu, kinetic_t * kinetics )
{
    Camera_Rotation_Init();
	Filters_Init( imu, kinetics );
}

/***********************************************************************************************//**
 *  \brief  Initialize Filters for Kinetic Data
 **************************************************************************************************/
void Filters_Init( LSM9DS1_t * imu, kinetic_t * kinetics )
{
	IMU_Update_All( imu );
    Kalman_Init( &kinetics->rotationFilter[0], imu->data.roll );
    Kalman_Init( &kinetics->rotationFilter[1], imu->data.pitch );
    Kalman_Init( &kinetics->rotationFilter[2], imu->data.yaw   );
}

/***********************************************************************************************//**
 *  \brief  Update IMU data and filter
 **************************************************************************************************/
void Kinetic_Update_Rotation( LSM9DS1_t * imu, kinetic_t * kinetics )
{
    IMU_Update_All( imu );
    
    double phi      = imu->data.roll;
    double theta    = imu->data.pitch;
    double psi      = imu->data.yaw;
    
    double delta_time = 0;
    
    /* Restrict pitch */
    double v = kinetics->rotationFilter[0].value;
    if( ( phi < -HALF_PI && v >  HALF_PI ) ||
        ( phi >  HALF_PI && v < -HALF_PI ) )
    {
        kinetics->rotationFilter[0].value  = phi;
        kinetics->rotation[0]              = phi;
    }
    else
    {
        /* Calculate the true pitch using a kalman_t filter */
        delta_time = seconds_since( kinetics->rotationFilter[0].timestamp );
        Kalman_Update( &kinetics->rotationFilter[0], phi, imu->data.gyro[0] * DEG_TO_RAD, delta_time );
        kinetics->rotation[0] = kinetics->rotationFilter[0].value;
    }
    
    if ( kinetics->rotation[0] > 90 )
    {
        /* Invert rate, so it fits the restricted accelerometer reading */
        imu->data.gyro[0] = -imu->data.gyro[0];
    }
    /* Calculate the true roll using a kalman_t filter */
    delta_time = seconds_since( kinetics->rotationFilter[1].timestamp );
    Kalman_Update( &kinetics->rotationFilter[1], theta, imu->data.gyro[1] * DEG_TO_RAD, delta_time );
    kinetics->rotation[1] = kinetics->rotationFilter[1].value;
    
    /* Calculate the true yaw using a kalman_t filter */
    seconds_since( kinetics->rotationFilter[2].timestamp );
    Kalman_Update( &kinetics->rotationFilter[2], psi, imu->data.gyro[2] * DEG_TO_RAD, delta_time );
    kinetics->rotation[2] = kinetics->rotationFilter[2].value;
}

/***********************************************************************************************//**
 *  \brief  Calculates system's absolute position and places value in truePosition[]

 \f{eqnarray*}{
    &a_a = rot_{f_0}, a_b = rot_{f_1}, a_c = rot_{f_2} \\

    &\mathbf{d} =
    \begin{cases}
        &d_{\hat{i}} = vis_{1_x} - vis_{0_x} \\
        &d_{\hat{j}} = vis_{1_y} - vis_{0_y} \\
        &d_{\hat{k}} = 0 \\
    \end{cases} \\
    &\mathbf{r} = \text{dAugment}(\mathbf{d}, \mathbf{a}) \\
    &\mathbf{e} =
    \begin{cases}
        &e_{\hat{i}} = V_{center_x} - vis_{0_x} \\
        &e_{\hat{j}} = V_{center_y} - vis_{0_y} \\
        &e_{\hat{k}} = 0 \\
    \end{cases} \\
    &\mathbf{e_{true}} = \text{zxyTranform}(\mathbf{e}, \mathbf{a}, 1) \\
    &\mathbf{p_{true}} = \mathbf{r} - \mathbf{e_{true}} \\
    &\mathbf{n} = \text{getNonGravAcceleration()} \\
    &\mathbf{v} =
    \begin{cases}
        &v_x = &n_{\hat{i}}\Delta{t} \\
        &v_y = &n_{\hat{j}}\Delta{t} \\
        &v_z = &n_{\hat{k}}\Delta{t} \\
    \end{cases} \\
   &\text{Update all position kalman_ts with } \mathbf{p_{true}}, \mathbf{v}, \text{ and } \Delta{t}
 \f}
 **************************************************************************************************/
void Kinetic_Update_Position( LSM9DS1_t * imu, kinetic_t * kinetics, cartesian2_t beacons[2] )
{
    /* Tait-Bryan angles of vision */
    double p_a[3];
    p_a[0] = kinetics->rotationFilter[0].value; // phi'
    p_a[1] = kinetics->rotationFilter[1].value; // theta'
    p_a[2] = kinetics->rotationFilter[2].value; // psi'
    
    /* Calculate beacon angles */
    double b_a[3];
    b_a[1]  = 0;
    b_a[0]  = CAMERA_ALPHA_H * ( ( beacons[0].y / CAMERA_HEIGHT ) - 0.5 );
    b_a[2]  = CAMERA_ALPHA_W * ( ( beacons[0].x / CAMERA_WIDTH  ) - 0.5 );
    b_a[0] += CAMERA_ALPHA_H * ( ( beacons[1].y / CAMERA_HEIGHT ) - 0.5 );
    b_a[2] += CAMERA_ALPHA_W * ( ( beacons[1].x / CAMERA_WIDTH  ) - 0.5 );

    Euler_To_Quaternion( &qp, &p_a );
    Euler_To_Quaternion( &qb, &b_a );
    
    Quaternion_Combine( &qp, &qc, &qb, &qa );
    
    double r_a[3][3];
    Quaternion_To_Matrix( &qa, r_a );
    /* Mu - Angle between d' to X-axis of reference */
    double mu = acos( r_a[2][2] );
    
    /* Sigma - Angle between beacons */
    double sigma = acos( cos( b_a[0] ) * cos( b_a[2] ) );
    
    /* r_l - Distance to beacons */
    double r_l = cos( mu - alpha ) / sin( alpha ) * D_FIXED;
    
    /* r_vec - Vector length r on X-axis */
    double r[3] = {r_l, 0, 0};
    multiplyVec3x1( r_a, r_l, r_l );
    
    /* Get non-gravitational acceleration */
    vec3_t ngacc = *( IMU_Non_Grav_Get( imu ) );
    double delta_time = 0;

    /* Filter calculated r_vec with acceleration > velocity */
    delta_time = seconds_since( kinetics->truePositionFilter[0].timestamp );
    double x_vel = ngacc.i * delta_time;
    Kalman_Update( &kinetics->truePositionFilter[0], r[0], x_vel, delta_time );

    delta_time = seconds_since( kinetics->truePositionFilter[1].timestamp );
	double y_vel = ngacc.j * delta_time;
	Kalman_Update( &kinetics->truePositionFilter[1], r[1], y_vel, delta_time );

	delta_time = seconds_since( kinetics->truePositionFilter[2].timestamp );
	double z_vel = ngacc.k * delta_time;
	Kalman_Update( &kinetics->truePositionFilter[2], r[2], z_vel, delta_time );
}

void Camera_Rotation_Init( void )
{
    double c_a[3];
    c_a[0] = CAMERA_OFFSET_ANGLE_X;
    c_a[1] = CAMERA_OFFSET_ANGLE_Y;
    c_a[2] = CAMERA_OFFSET_ANGLE_Z;
    Euler_To_Quaternion( &qc, &c_a );
}
