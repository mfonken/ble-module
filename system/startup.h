//
//  startup.h
//  
//
//  Created by Matthew Fonken on 1/4/17.
//
//

#ifndef startup_h
#define startup_h

/* System utilities */
#include "usart_sp.h"
#include "clock_sp.h"

/* Peripheral/Sensor headers */
#include "cam_controller.h"
#include "PCA9534A.h"
#include "LSM9DS1.h"
#include "CPT112S.h"

#define ENABLED             1
#define DISABLED            0

#define SYS_CTL_LOC         0
#define IMU_LOC             1
#define CAMERA_LOC          2
#define TOUCH_CTL_LOC       3

#define SYS_CTL_DEFAULT    ENABLED
#define IMU_DEFAULT        ENABLED
#define CAMERA_DEFAULT     DISABLED
#define TOUCH_CTL_DEFAULT  ENABLED

#define SYSTEM_DEFAULT                (\
    SYS_CTL_DEFAULT   << SYS_CTL_LOC  |\
    IMU_DEFAULT       << IMU_LOC      |\
    CAMERA_DEFAULT    << CAMERA_LOC   |\
    TOUCH_CTL_DEFAULT << TOUCH_CTL_LOC )

typedef struct
{
    /* System Utilities */
    uint8_t     system_controller:1;
    
    /* Sensors */
    uint8_t     imu:1;
    uint8_t     camera:1;
    uint8_t     touch:1;
    
    uint8_t     RESERVED:4;
} system_t;

extern system_t system;

void startup( void );

void Utilities_Init( void );
void Sensors_Init( void );

#endif /* startup_h */
