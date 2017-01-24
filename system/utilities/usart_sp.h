/***********************************************************************************************//**
 * \file   usart_sp.h
 * \brief  USART Special Functions Header
 ***************************************************************************************************
 *      Author: Matthew Fonken
 **************************************************************************************************/

#ifndef USART_SP_H_
#define USART_SP_H_

#include <stdint.h>
#include <stdbool.h>

/***********************************************************************************************//**
 * @addtogroup Application
 * @{
 **************************************************************************************************/

/***********************************************************************************************//**
 * @addtogroup sp
 * @{
 **************************************************************************************************/

#define MAX_STRING_LENGTH 50

uint8_t Read_Char( void );

/***********************************************************************************************//**
 *  \brief  Print character
 *  \param[in] c Character to print
 **************************************************************************************************/
void Print_Char( uint8_t c );

/***********************************************************************************************//**
 *  \brief  Print character
 *  \param[in] c Character to print
 **************************************************************************************************/
void Print_String( char s[] );
void Print_Line( char s[] );

/***********************************************************************************************//**
 *  \brief  Print string
 *  \param[in] s String to print
 *  \param[in] len Length of string
 **************************************************************************************************/
void Print_Double_Ascii( double v );

/***********************************************************************************************//**
 *  \brief  Print two byte integer
 *  \param[in] v integer to print
 **************************************************************************************************/
void Print_IMU( double motion_data[6], bool stripped );

void reverse(uint8_t *str, int len);
int intToStr(int x, uint8_t str[], int d);
void dtoa(double n, uint8_t *res, int afterpoint);
/** @} (end addtogroup sp) */
/** @} (end addtogroup Application) */

#endif /* USART_SP_H_ */
