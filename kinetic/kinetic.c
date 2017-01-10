/***********************************************************************************************//**
 * \file   kinetic.c
 * \brief  Kinetic Motion code
 ***************************************************************************************************
 *  Created on: Nov 13, 2016
 *      Author: Matthew Fonken
 **************************************************************************************************/

/* Own header */
#include "kinetic.h"

/* Sensors headers */
#include "LSM9DS1.h"

/* Math headers */
#include "kalman.h"
#include "matrix.h"

/* Additional function headers */
#include "usart_sp.h"

/***************************************************************************************************
 Local Variables
 **************************************************************************************************/
/** Coordinates of beacons returned by vision */
static cartesian2_t   vis[2];

/** Local change in time */
//static double         delta_t;

/** Local positional and rotational vectors */
static kinetic_t      kinetics;

/***********************************************************************************************//**
 *  \brief  Initialize Kinetic Sensors
 **************************************************************************************************/
void initKinetics( void )
{
	IMU_Init();
	Print_String("IMU Initialized.\r\n", 18);
	initFilters();
}

/***********************************************************************************************//**
 *  \brief  Initialize Filters for Kinetic Data
 **************************************************************************************************/
void initFilters( void )
{
	LSM9DS1_t * imu_pointer = IMU_Update();
    initKalman( &kinetics.rotationFilter[0], imu_pointer->imu.roll );
    initKalman( &kinetics.rotationFilter[1], imu_pointer->imu.pitch );
    initKalman( &kinetics.rotationFilter[2], imu_pointer->imu.yaw   );
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
    /* Create v (vision) vector for zero state */
    vec3_t vvec;
    vvec.ihat = VISION_ZSTATE_IHAT;
    vvec.jhat = VISION_ZSTATE_JHAT;
    vvec.khat = VISION_ZSTATE_KHAT;
    //normalizevec3_t(vvec); // normalize if vvec inital isn't of length 1
    /* Transform and normalize v vector by given angles to get unit vector from camera */
    vec3_t *vtru = zxyTransform( &vvec, a, 0 );

    /* Transform d vector by given angles to get true vector between beacons */
    vec3_t *dtru = zxyTransform( dvec, a, 0 );

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
void getAbsolutePosition( void )
{
    /* Tait-Bryan angles of vision */
    ang3_t tba;
    tba.a = kinetics.rotationFilter[0].value;
    tba.b = kinetics.rotationFilter[1].value;
    tba.c = kinetics.rotationFilter[2].value;

    /* vector from B1 in vision -TO-> B2 in vision */
    vec3_t dvec;
    dvec.ihat = vis[1].x - vis[0].x;
    dvec.jhat = vis[1].y - vis[0].y;
    dvec.khat = 0;                             // d is only on XY plane (surface of beacons)

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

    kinetics.truePosition[0] = rvec.ihat;
    kinetics.truePosition[1] = rvec.jhat;
    kinetics.truePosition[2] = rvec.khat;

    /* Filter calculated absolute position and with integrated acceleration (velocity) */
    ang3_t * ang;
    ang->a = kinetics.rotationFilter[0].value;
    ang->b = kinetics.rotationFilter[1].value;
    ang->c = kinetics.rotationFilter[2].value;
    vec3_t *ngacc = getNonGravAcceleration( ang );
    double delta_time = 0; // Insert RTCC
    double x_vel = ngacc->ihat * delta_time;
    updateKalman( &kinetics.truePositionFilter[0], kinetics.truePosition[0], x_vel, delta_time);
    double y_vel = ngacc->jhat * delta_time;
    updateKalman( &kinetics.truePositionFilter[1], kinetics.truePosition[1], y_vel, delta_time);
    double z_vel = ngacc->khat * delta_time;
    updateKalman( &kinetics.truePositionFilter[2], kinetics.truePosition[2], z_vel, delta_time);
}

/***********************************************************************************************//**
 *  \brief  Update IMU data and filter
 **************************************************************************************************/
void Kalman_Update( void )
{
	LSM9DS1_t this;
    this = *( IMU_Update() );

    double phi      = this.imu.roll;
    double theta    = this.imu.pitch;
    double psi      = this.imu.yaw;

    /* Restrict pitch */
    double v = kinetics.rotationFilter[0].value;
    if( ( phi < -HALF_PI && v > HALF_PI ) ||
        ( phi > HALF_PI  && v < -HALF_PI ) )
    {
        kinetics.rotationFilter[0].value  = phi;
        kinetics.rotation[0]              = phi;
    }
    else
    {
        double delta_time0 = getMillis() - kinetics.rotationFilter[0].timestamp;
        /* Calculate the true pitch using a kalman_t filter */
        updateKalman( &kinetics.rotationFilter[0], phi, this.imu.gyro[0], delta_time0 );
        kinetics.rotation[0] = kinetics.rotationFilter[0].value;
    }

    if ( kinetics.rotation[0] > HALF_PI )
    {
        /* Invert rate, so it fits the restricted accelerometer reading */
        this.imu.gyro[0] = -this.imu.gyro[0];
    }
    
    double delta_time1 = getMillis() - kinetics.rotationFilter[1].timestamp;
    double delta_time2 = getMillis() - kinetics.rotationFilter[2].timestamp;
    
    /* Calculate the true roll using a kalman_t filter */
    updateKalman( &kinetics.rotationFilter[1], theta, this.imu.gyro[1], delta_time1 );
    kinetics.rotation[1] = kinetics.rotationFilter[1].value;
    /* Calculate the true yaw using a kalman_t filter */
    updateKalman( &kinetics.rotationFilter[2], psi, this.imu.gyro[2], delta_time2 );
    kinetics.rotation[2] = kinetics.rotationFilter[2].value;
}

// TODO
double getMillis( void )
{
    return 0;
}
