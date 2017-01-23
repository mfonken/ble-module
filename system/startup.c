//
//  startup.c
//  
//
//  Created by Matthew Fonken on 1/4/17.
//
//

#include "startup.h"

system_t system;

void startup( void )
{
    uint8_t sys_default = SYSTEM_DEFAULT;
    system = *( system_t * )&sys_default;
    
    Utilities_Init();
    Sensors_Init();
}

void Utilities_Init( void )
{
    SYSCLK_Init();
    Print_String( "System Clock Initialized.\r\n" );
    
    if( system.system_controller )
    {
        SYSCTL_Init();
        Print_String( "System Controller Initialized.\r\n" );
    }
}

void Sensors_Init( void )
{
    if( system.imu )
    {
        IMU_Init();
        Print_String( "IMU Initialized.\r\n" );
    }
    
    if( systen.camera & )
    {
        Camera_Init();
        Print_String( "Camera Initialized.\r\n" );
    }
    
    if( system.touch )
    {
        Touch_Init();
        Print_String( "Touch Controller Initialized.\r\n" );
    }
}
