/***********************************************************************************************//**
* \file   CPT112S.h
* \brief  CPT112S Touch Controller Header
***************************************************************************************************
*      Author: Matthew Fonken
**************************************************************************************************/

#ifndef CPT112S_h
#define CPT112S_h

#include <stdio.h>
#include <stdint.h>

#include "app.h"

#define TOUCH_CTR_ADDR      0xe0

typedef struct
{
    uint32_t packet_counter :4;
    uint32_t type           :4;
    uint32_t description    :16;
    uint32_t RESERVED       :8;
} touch_ctr_event_t;

enum event_types
{
    TOUCH_EVENT     = 0,
    RELEASE_EVENT   = 1,
    SLIDER_ACTIVITY = 2,
};

void 	  Touch_Init( void );
uint8_t * Touch_Read( void );

#endif /* CPT112S_h */
