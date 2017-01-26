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

/** Local positional and rotational vectors */

/***********************************************************************************************//**
 *  \brief  Initialize Kinetic Sensors
 **************************************************************************************************/
void Kinetic_Init( LSM9DS1_t * imu, kinetic_t * kinetics )
{
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
    ang3_t tba;
    tba.a = kinetics->rotationFilter[0].value; // phi'
    tba.b = kinetics->rotationFilter[1].value; // theta'
    tba.c = kinetics->rotationFilter[2].value; // psi'

    /* Adjust for rotation detected by camera */
    tba.a -= 0;
    tba.b -= CAMERA_ALPHA_H * ( ( beacons[0].y / CAMERA_HEIGHT ) - 0.5 );
    tba.c -= CAMERA_ALPHA_W * ( ( beacons[0].x / CAMERA_WIDTH  ) - 0.5 );

    /* Create detected d vector */
    vec2_t dvec;
    dvec.i = ( beacons[1].x - beacons[0].x );
    dvec.j = ( beacons[1].y - beacons[0].y );

    /* Calculate angle between camera and beacon axis and adjust length */
    double sigma = asin( cos( tba.b ) * cos( tba.c ) );
    dvec.i /= sigma;
    dvec.j /= sigma;

    /* Calculate camera's distance to beacon */
    double r = ( BEACON_DISTANCE * ( 1 + D_AUG ) ) / lengthOfvec2_t( &dvec );

    /* Rotate by rotation matric for final position vector */
    vec3_t fvec;
    fvec.i = r;
    fvec.j = 0;
    fvec.k = 0;
    vec3_t tvec = *( zxyTransform( &fvec, &tba ) );

    vec3_t ngacc = *( IMU_Non_Grav_Get( imu ) );
    double delta_time = 0;

    delta_time = seconds_since( kinetics->truePositionFilter[0].timestamp );
    double x_vel = ngacc.i * delta_time;
    Kalman_Update( &kinetics->truePositionFilter[0], tvec.i, x_vel, delta_time );

    delta_time = seconds_since( kinetics->truePositionFilter[1].timestamp );
	double y_vel = ngacc.j * delta_time;
	Kalman_Update( &kinetics->truePositionFilter[1], tvec.j, y_vel, delta_time );

	delta_time = seconds_since( kinetics->truePositionFilter[2].timestamp );
	double z_vel = ngacc.k * delta_time;
	Kalman_Update( &kinetics->truePositionFilter[2], tvec.k, z_vel, delta_time );
}
