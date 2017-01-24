/***********************************************************************************************//**
 * \file   mpu9250.c
 * \brief  IMU Control
 ***************************************************************************************************
 *      Author: Matthew Fonken
 **************************************************************************************************/

/* Standard headers */
#include <stddef.h>
#include <math.h>

/* em headers */
#include "em_i2c.h"

/* Additional function headers */
#include "i2c_sp.h"

/* Own header */
#include "LSM9DS1.h"

/* Math headers */
#include "matrix.h"

/***********************************************************************************************//**
 * @addtogroup Application
 * @{
 **************************************************************************************************/

/***********************************************************************************************//**
 * @addtogroup imu
 * @{
 **************************************************************************************************/

/***************************************************************************************************
 Macros
 **************************************************************************************************/
#define sign(x) x > 0 ? 1:-1

/***************************************************************************************************
 Local Variables
 **************************************************************************************************/
//LSM9DS1_t this;

double magResolutions[4] = { 0.00014, 0.00029, 0.00043, 0.00058 };

/***************************************************************************************************
 Local Functions
 **************************************************************************************************/
/**************************************************************************//**
 * \brief IMU_GetRegister
 * \param[out] Return value from register
 * \param[in] reg Register to access
 *****************************************************************************/
uint8_t IMU_GetRegister( uint8_t reg )
{
	uint8_t i2c_read_data[1];
	I2C_Read_Reg( LSM9DS1_IMU_ADDR, reg, i2c_read_data, 1 );
	return i2c_read_data[0];
}

/**************************************************************************//**
 * \brief IMU_SetRegister
 * \param[in] reg Register to access
 * \param[in] val Value to set
 *****************************************************************************/
void IMU_SetRegister( uint8_t reg, uint8_t val )
{
	uint8_t i2c_write_data[2];
	i2c_write_data[0] = reg;
	i2c_write_data[0] = val;
	I2C_Write( LSM9DS1_IMU_ADDR, i2c_write_data, 2 );
}

/***************************************************************************************************
 Initialization
 **************************************************************************************************/

/**************************************************************************//**
 * \brief Initialize IMU with local settings
 * \param[out] Initialization success
 *****************************************************************************/
void IMU_Init( LSM9DS1_t * this )
{
	LSM9DS1_defaultInit( &this->settings );

    this->imu.accel_bias[0] 	= ACCEL_BIAS_X;
    this->imu.accel_bias[1] 	= ACCEL_BIAS_Y;
    this->imu.accel_bias[2] 	= ACCEL_BIAS_Z;

    this->imu.gyro_bias[0] 	= GYRO_BIAS_X;
	this->imu.gyro_bias[1] 	= GYRO_BIAS_Y;
	this->imu.gyro_bias[2]	= GYRO_BIAS_Z;

	this->imu.mag_bias[0] 	= MAG_BIAS_X;
	this->imu.mag_bias[1] 	= MAG_BIAS_Y;
	this->imu.mag_bias[2] 	= MAG_BIAS_Z;

	uint32_t accel_res;
	switch ( XL_FS_DEFAULT )
	{
		default:
		case XL_FS_2G:
			accel_res = 2;
			break;
		case XL_FS_4G:
			accel_res = 4;
			break;
		case XL_FS_8G:
			accel_res = 8;
			break;
		case XL_FS_16G:
			accel_res = 16;
			break;
	}

	uint32_t gyro_res;
	switch ( GYRO_FS_DEFAULT )
	{
		default:
		case GYRO_FS_245DPS:
			gyro_res = 245;
			break;
		case GYRO_FS_500DPS:
			gyro_res = 500;
			break;
		case GYRO_FS_2000DPS:
			gyro_res = 2000;
			break;
	}

	double mag_res;
	switch ( MAG_FS_DEFAULT )
	{
		default:
		case MAG_FS_4GAUSS:
			mag_res = magResolutions[0];
			break;
		case MAG_FS_8GAUSS:
			mag_res = magResolutions[1];
			break;
		case MAG_FS_12GAUSS:
			mag_res = magResolutions[2];
			break;
		case MAG_FS_16GAUSS:
			mag_res = magResolutions[3];
			break;
	}

	this->imu.accel_res 	= accel_res / LSM9DS1_IMU_ADC_MAX;
	this->imu.gyro_res 		= gyro_res  / LSM9DS1_IMU_ADC_MAX;
	this->imu.mag_res 		= mag_res;
}

/******************************************************************************
 * Motion Data Access
 *****************************************************************************/

/**************************************************************************//**
 * \brief Read IMU accel and gyro data
 * \param[in] read_data Array to store read data
 *****************************************************************************/
void IMU_Update_All( LSM9DS1_t * this )
{
    IMU_Update_Accel( this );
    IMU_Update_Gyro( this );
    IMU_Update_Mag( this );
    
    IMU_Update_Roll( this );
    IMU_Update_Pitch( this );
    IMU_Update_Yaw( this );
}

void IMU_Update_Angles( LSM9DS1_t * this )
{
    IMU_Update_Accel( this );
    IMU_Update_Mag( this );
    
    IMU_Update_Roll( this );
    IMU_Update_Pitch( this );
    IMU_Update_Yaw( this );
}

void IMU_Update_Accel( LSM9DS1_t * this )
{
    /* Combine low and high byte values */
    uint8_t                    i2c_read_data[6];
    I2C_Read_Reg( LSM9DS1_IMU_ADDR, XL_OUT, i2c_read_data, 6 );
    int16_t accel[3];
    accel[0] = ( i2c_read_data[1] << 8 ) | i2c_read_data[0];
    accel[1] = ( i2c_read_data[3] << 8 ) | i2c_read_data[2];
    accel[2] = ( i2c_read_data[5] << 8 ) | i2c_read_data[4];
    
    for( int i = 0; i < 3 ; i++ )
    {
        this->imu.accel[i] = accel[i] * this->imu.accel_res - this->imu.accel_bias[i];
    }
}

void IMU_Update_Gyro( LSM9DS1_t * this )
{
    /* Combine low and high byte values */
    uint8_t                    i2c_read_data[6];
    I2C_Read_Reg( LSM9DS1_IMU_ADDR, G_OUT, i2c_read_data, 6 );
    int16_t gyro[3];
    gyro[0] = ( i2c_read_data[1] << 8 ) | i2c_read_data[0];
    gyro[1] = ( i2c_read_data[3] << 8 ) | i2c_read_data[2];
    gyro[2] = ( i2c_read_data[5] << 8 ) | i2c_read_data[4];
    
    for( int i = 0; i < 3 ; i++ )
    {
        this->imu.gyro[i]  = gyro[i]  * this->imu.gyro_res  - this->imu.gyro_bias[i];
    }
}

void IMU_Update_Mag( LSM9DS1_t * this )
{
    /* Combine low and high byte values */
    uint8_t                    i2c_read_data[6];
    I2C_Read_Reg( LSM9DS1_MAG_ADDR, M_OUT, i2c_read_data, 6 );
    int16_t mag[3];
    mag[0] = ( i2c_read_data[1] << 8 ) | i2c_read_data[0];
    mag[1] = ( i2c_read_data[3] << 8 ) | i2c_read_data[2];
    mag[2] = ( i2c_read_data[5] << 8 ) | i2c_read_data[4];
    
    for( int i = 0; i < 3 ; i++ )
    {
        this->imu.mag[i]   = mag[i]   * this->imu.mag_res	  - this->imu.mag_bias[i];
    }
}

/******************************************************************************
 * Rotation Calculation
 *****************************************************************************/
/* See - http://www.nxp.com/files/sensors/doc/app_note/AN3461.pdf and
       - http://www.nxp.com/assets/documents/data/en/application-notes/AN4248.pdf
 */

/**************************************************************************//**
 * \brief Calculate roll angle (phi) from accelerometer data
 * \param[out] Return roll
 *****************************************************************************/
void IMU_Update_Roll( LSM9DS1_t * this )
{
    /* AN4248: Eq. 13 */
    this->imu.roll = atan2( this->imu.accel[0], this->imu.accel[2] );
    
    /* AN3461: Eq. 37 */
//    double den = sqrt( ( ( this->imu.accel[1] * this->imu.accel[1] ) + ( this->imu.accel[2] * this->imu.accel[2] ) ) );
//    this->imu.roll = atan2( -this->imu.accel[0], den );
}

/**************************************************************************//**
 * \brief Calculate pitch angle (theta) from accelerometer data
 * \param[out] Return pitch
 *****************************************************************************/
void IMU_Update_Pitch( LSM9DS1_t * this )
{
    /* AN4248: Eq. 14 */
    double den = ( this->imu.accel[0] * sin( this->imu.roll ) ) + ( this->imu.accel[2] * cos ( this->imu.roll ) );
    this->imu.pitch = atan2( -this->imu.accel[1], den );
    
    /* AN3461: Eq. 38 */
//    double den = sign( this->imu.accel[2] ) * sqrt( ( ( this->imu.accel[2] * this->imu.accel[2] ) + ( MU * ( this->imu.accel[0] * this->imu.accel[0] ) ) ) );
//    this->imu.pitch = atan2( this->imu.accel[1], den );
}

/**************************************************************************//**
 * \brief Calculate yaw angle (psi) from magnetometer data, pitch, and roll
 * \param[out] Return yaw
 *****************************************************************************/
 void IMU_Update_Yaw( LSM9DS1_t * this )
{
	double Bx = this->imu.mag[1];
	double By = this->imu.mag[0];
	double Bz = -this->imu.mag[2];

    /* AN4248: Eq. 22 */
    double sin_phi   = sin( this->imu.roll );
    double sin_theta = sin( this->imu.pitch );
    double cos_phi   = cos( this->imu.roll );
    double cos_theta = cos( this->imu.pitch );
    double num = ( Bz * sin_phi ) - ( By * cos_phi );
    double den = ( Bx * cos_theta ) + ( By * ( sin_theta * sin_phi ) ) + ( Bz * ( sin_theta * cos_phi ) );
    this->imu.yaw = atan2( num, den );
}

/**************************************************************************//**
 * \brief Calculate roll angle (phi) error from accelerometer data
 * \param[out] Return roll error
 *****************************************************************************/
double IMU_Update_Roll_Error( LSM9DS1_t * this )
{
    double sin_phi   = sin( this->imu.roll );
    double sin_theta = sin( this->imu.pitch );
    double cos_phi   = cos( this->imu.roll );
    double cos_theta = cos( this->imu.pitch );
    double cos_theta_cos_phi = cos_theta * cos_phi;
    double mu_sin_2_theta = MU * ( sin_theta * sin_theta );
    double factor = sqrt( ( cos_theta_cos_phi * cos_theta_cos_phi ) + mu_sin_2_theta );
    double num = sin_phi * ( cos_theta_cos_phi - factor );
    double den = ( cos_theta * ( sin_phi * sin_phi ) ) + ( cos_phi * factor );
    return atan2( num, den );
}

/**************************************************************************//**
 * \brief Get no gravitation acceleration from accelerometer data
 * \param[out] Return 3D vector of acceleration
 * \param[in] tba Tait-Bryan angles to transform by
 *****************************************************************************/
vec3_t * getNonGravAcceleration( LSM9DS1_t * this, ang3_t * tba )
{
    /* Create a vector of accelerometer values */
    vec3_t avec;
    IMU_Update_Accel( this );
    avec.ihat = this->imu.accel[0];
    avec.jhat = this->imu.accel[1];
    avec.khat = this->imu.accel[2];

    /* Transform and normalize v vector by given angles to get unit vector from camera */
    vec3_t * atru = zxyTransform( &avec, tba, 1 );

    atru->khat += 1; // Negate gravity
    return atru;
}

/******************************************************************************
 * Temperature
 *****************************************************************************/

/**************************************************************************//**
 * \brief Read temperature from register
 * \param[out] Return raw temperature data
 *****************************************************************************/
uint16_t IMU_ReadTemp( void )
{
	uint16_t tempurature;
	tempurature  = IMU_GetRegister( OUT_TEMP_H ) << 8;
	tempurature += IMU_GetRegister( OUT_TEMP_L );
	return tempurature;
}

/**************************************************************************//**
 * \brief Read temperature in Fahrenheit
 * \param[out] Return corrected temperature data as readable double
 *****************************************************************************/
double getTempF( void )
{
	return ( double )( ( IMU_ReadTemp() / 16 ) + 25 );;
}

/**************************************************************************//**
 * \brief Read temperature in Celsius
 * \param[out] Return corrected temperature data as readable double
 *****************************************************************************/
double getTempC( void )
{
    return ( double )( ( getTempF() * ( 5 / 9 ) ) - 32 );;
}

/** @} (end addtogroup imu) */
/** @} (end addtogroup Application) */
