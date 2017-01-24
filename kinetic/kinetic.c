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
    Kalman_Init( &kinetics->rotationFilter[0], imu->imu.roll );
    Kalman_Init( &kinetics->rotationFilter[1], imu->imu.pitch );
    Kalman_Init( &kinetics->rotationFilter[2], imu->imu.yaw   );
}

/***********************************************************************************************//**
 *  \brief  Update IMU data and filter
 **************************************************************************************************/
void Kinetic_Update_Rotation( LSM9DS1_t * imu, kinetic_t * kinetics )
{
    IMU_Update_All( imu );
    
    double phi      = imu->imu.roll  * RAD_TO_DEG;
    double theta    = imu->imu.pitch * RAD_TO_DEG;
    double psi      = imu->imu.yaw 	 * RAD_TO_DEG;
    
    double delta_time = 0;
    
    /* Restrict pitch */
    double v = kinetics->rotationFilter[0].value;
    if( ( phi < -90 && v >  90 ) ||
        ( phi >  90 && v < -90 ) )
    {
        kinetics->rotationFilter[0].value  = phi;
        kinetics->rotation[0]              = phi;
    }
    else
    {
        /* Calculate the true pitch using a kalman_t filter */
        delta_time = seconds_since( kinetics->rotationFilter[0].timestamp );
        Kalman_Update( &kinetics->rotationFilter[0], phi, imu->imu.gyro[0], delta_time );
        kinetics->rotation[0] = kinetics->rotationFilter[0].value;
    }
    
    if ( kinetics->rotation[0] > 90 )
    {
        /* Invert rate, so it fits the restricted accelerometer reading */
        imu->imu.gyro[0] = -imu->imu.gyro[0];
    }
    /* Calculate the true roll using a kalman_t filter */
    delta_time = seconds_since( kinetics->rotationFilter[1].timestamp );
    Kalman_Update( &kinetics->rotationFilter[1], theta, imu->imu.gyro[1], delta_time );
    kinetics->rotation[1] = kinetics->rotationFilter[1].value;
    
    /* Calculate the true yaw using a kalman_t filter */
    seconds_since( kinetics->rotationFilter[2].timestamp );
    Kalman_Update( &kinetics->rotationFilter[2], psi, imu->imu.gyro[2], delta_time );
    kinetics->rotation[2] = kinetics->rotationFilter[2].value;
}

/***********************************************************************************************//**
 *  \brief  Initialize Filters for Kinetic Data
 *  \param[in] dvec Beacon positional vector to augment
 *  \param[in] a Tait-Bryan angles to augement by

 \f{eqnarray*}{
    &\mathbf{v} =
    \begin{cases}
        &v_{\hat{i}} = VZ_{\hat{i}} \\
        &v_{\hat{j}} = VZ_{\hat{j}} \\
        &v_{\hat{k}} = VZ_{\hat{k}} \\
    \end{cases} \\
    &\mathbf{v_{true}} = \text{zxyTranform}(\mathbf{v}, \mathbf{a}, 1) \\
    &\mathbf{d_{true}} = \text{zxyTranform}(\mathbf{d}, \mathbf{a}, 1) \\
    &c_{augment} = \frac{D_{beacon}(1 + D_{augment})}{||\mathbf{d_{true}}||} \\
    &\mathbf{v_{return}} =c_{augment}\mathbf{v_{true}}
 \f}
**************************************************************************************************/
vec3_t *dAugment( vec3_t *dvec,
                  ang3_t *a)
{
    /* Create v (vision) vector for zero state - e.g. unit vector */
    vec3_t vvec;
    vvec.ihat = VISION_ZSTATE_IHAT;
    vvec.jhat = VISION_ZSTATE_JHAT;
    vvec.khat = VISION_ZSTATE_KHAT;
    //normalizevec3_t(vvec); // normalize if vvec inital isn't of length 1
    
    /* Transform and normalize v vector by given angles to get unit vector from camera */
    vec3_t *vtru = zxyTransform( &vvec, a, 0 );

    /* Transform d vector by given angles to get true vector between beacons */
    vec3_t *dtru = zxyTransform(  dvec, a, 0 );

    /* Calculate estimated augmentation of v vector
       by ratio of transformed true d estimation and real (BEACON_DISTANCE) */
    double aug = ( BEACON_DISTANCE * ( 1 + D_AUG ) ) / lengthOfvec3_t( dtru ) ;

    /* Apply augmentation and return */
    vtru->ihat *= aug;
    vtru->jhat *= aug;
    vtru->khat *= aug;
    return vtru;
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
void Kinetic_Update_Position( LSM9DS1_t * imu, kinetic_t * kinetics, vec2_t * d )
{
    /* Tait-Bryan angles of vision */
    ang3_t tba;
    tba.a = kinetics->rotationFilter[0].value;
    tba.b = kinetics->rotationFilter[1].value;
    tba.c = kinetics->rotationFilter[2].value;

    /* vector from B1 in vision -TO-> B2 in vision */
    vec3_t dvec;
    dvec.ihat = d.ihat;
    dvec.jhat = d.ihat;
    dvec.khat = 0; // d is only on XY plane (surface of beacons)

    /* Create r vector (camera -TO-> vision center) from augment generated true d and vision d */
    vec3_t rvec = *( dAugment( &dvec, &tba ) );

    /* vector between vision center -TO-> B1 
        (e is only on XY plane (surface of beacons) */
    vec3_t evec;
    evec.ihat = VISION_CENTER_X - vis[0].x;
    evec.jhat = VISION_CENTER_Y - vis[0].y;
    evec.khat = 0;

    /* vector between vision center -TO-> B2 
        (f is only on XY plane (surface of beacons) */
//    vec3_t fvec;
//    fvec.ihat = VISION_CENTER_X - vis[1].x;
//    fvec.jhat = VISION_CENTER_Y - vis[1].y;
//    fvec.khat = 0;

    /* Transform e vector (vision center -TO-> B1) 
        to true e vector (transformed vision center -TO-> true B1) */
    vec3_t etru = *( zxyTransform( &evec, &tba, 1 ) );

    /* Subract true e vector from augmented r vector */
    subtractvec3_t(&rvec, &etru);

    kinetics->truePosition[0] = rvec.ihat;
    kinetics->truePosition[1] = rvec.jhat;
    kinetics->truePosition[2] = rvec.khat;

    /* Filter calculated absolute position and with integrated acceleration (velocity) */
    ang3_t * ang;
    ang->a = kinetics->rotationFilter[0].value;
    ang->b = kinetics->rotationFilter[1].value;
    ang->c = kinetics->rotationFilter[2].value;
    
    vec3_t *ngacc = IMU_Non_Grav_Get( imu );
    
    uint32_t delta_time = 0;
    
    delta_time = timestamp() - kinetics->truePositionFilter[0].timestamp;
    double x_vel = ngacc->ihat * delta_time;
    Kalman_Update( &kinetics->truePositionFilter[0], kinetics->truePosition[0], x_vel, delta_time);
    
    delta_time = timestamp() - kinetics->truePositionFilter[1].timestamp;
    double y_vel = ngacc->jhat * delta_time;
    Kalman_Update( &kinetics->truePositionFilter[1], kinetics->truePosition[1], y_vel, delta_time);
    
    delta_time = timestamp() - kinetics->truePositionFilter[2].timestamp;
    double z_vel = ngacc->khat * delta_time;
    Kalman_Update( &kinetics->truePositionFilter[2], kinetics->truePosition[2], z_vel, delta_time);
}
