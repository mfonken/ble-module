################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../system/utilities/i2c_sp.c \
../system/utilities/usart_sp.c 

OBJS += \
./system/utilities/i2c_sp.o \
./system/utilities/usart_sp.o 

C_DEPS += \
./system/utilities/i2c_sp.d \
./system/utilities/usart_sp.d 


# Each subdirectory must supply rules for building sources it contributes
system/utilities/i2c_sp.o: ../system/utilities/i2c_sp.c
	@echo 'Building file: $<'
	@echo 'Invoking: IAR C/C++ Compiler for ARM'
	iccarm "$<" -o "$@" --enum_is_int --no_wrap_diagnostics -I"C:\Users\Matthew Fonken\SimplicityStudio\v4_workspace\ble-module\inc" -I"C:\Users\Matthew Fonken\SimplicityStudio\v4_workspace\ble-module\sensors\camera" -I"C:\Users\Matthew Fonken\SimplicityStudio\v4_workspace\ble-module\kinetic" -I"C:\Users\Matthew Fonken\SimplicityStudio\v4_workspace\ble-module\sensors\touch" -IC:/Users/Matthew%2525252520Fonken/SimplicityStudio/v4_workspace/ble-module -I"C:\Users\Matthew Fonken\SimplicityStudio\v4_workspace\ble-module\sensors\imu" -I"C:\Users\Matthew Fonken\SimplicityStudio\v4_workspace\ble-module\system" -IC:/Users/Matthew%2525252520Fonken/SimplicityStudio/v4_workspace/ble-module/src -I"C:\Users\Matthew Fonken\SimplicityStudio\v4_workspace\ble-module\app" -I"C:\Users\Matthew Fonken\SimplicityStudio\v4_workspace\ble-module\system\utilities" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//platform/CMSIS/Include" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//platform/Device/SiliconLabs/EFR32BG1B/Include" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//platform/emdrv/common/inc" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//platform/emdrv/dmadrv/config" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//platform/emdrv/dmadrv/inc" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//platform/emdrv/gpiointerrupt/inc" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//platform/emdrv/nvm/config" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//platform/emdrv/nvm/inc" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//platform/emdrv/rtcdrv/config" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//platform/emdrv/rtcdrv/inc" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//platform/emdrv/sleep/inc" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//platform/emdrv/spidrv/config" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//platform/emdrv/spidrv/inc" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//platform/emdrv/tempdrv/config" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//platform/emdrv/tempdrv/inc" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//platform/emdrv/uartdrv/config" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//platform/emdrv/uartdrv/inc" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//platform/emdrv/ustimer/config" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//platform/emdrv/ustimer/inc" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//platform/emlib/inc" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//hardware/kit/common/bsp" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//hardware/kit/common/drivers" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//platform/radio/rail_lib/chip/efr32/rf/common/cortex" -I"C:/Users/Matthew Fonken/SimplicityStudio/v4_workspace/ble-module/src" -I"C:/Users/Matthew Fonken/SimplicityStudio/v4_workspace/ble-module" -I"C:\Users\Matthew Fonken\SimplicityStudio\v4_workspace\ble-module\src" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//protocol/bluetooth_2.1/ble_stack/inc/common" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//protocol/bluetooth_2.1/ble_stack/inc/soc" -e --cpu Cortex-M4F --fpu VFPv4_sp --debug --endian little --cpu_mode thumb -On --no_cse --no_unroll --no_inline --no_code_motion --no_tbaa --no_clustering --no_scheduling '-DSILABS_AF_USE_HWCONF=1' '-DGENERATION_DONE=1' '-D__NO_SYSTEM_INIT=1' '-DEFR32BG1B232F256GM32=1' --diag_suppress pa050 --diag_suppress pa050 --diag_error pe223 --diag_error pe223 --dependencies=m system/utilities/i2c_sp.d
	@echo 'Finished building: $<'
	@echo ' '

system/utilities/usart_sp.o: ../system/utilities/usart_sp.c
	@echo 'Building file: $<'
	@echo 'Invoking: IAR C/C++ Compiler for ARM'
	iccarm "$<" -o "$@" --enum_is_int --no_wrap_diagnostics -I"C:\Users\Matthew Fonken\SimplicityStudio\v4_workspace\ble-module\inc" -I"C:\Users\Matthew Fonken\SimplicityStudio\v4_workspace\ble-module\sensors\camera" -I"C:\Users\Matthew Fonken\SimplicityStudio\v4_workspace\ble-module\kinetic" -I"C:\Users\Matthew Fonken\SimplicityStudio\v4_workspace\ble-module\sensors\touch" -IC:/Users/Matthew%2525252520Fonken/SimplicityStudio/v4_workspace/ble-module -I"C:\Users\Matthew Fonken\SimplicityStudio\v4_workspace\ble-module\sensors\imu" -I"C:\Users\Matthew Fonken\SimplicityStudio\v4_workspace\ble-module\system" -IC:/Users/Matthew%2525252520Fonken/SimplicityStudio/v4_workspace/ble-module/src -I"C:\Users\Matthew Fonken\SimplicityStudio\v4_workspace\ble-module\app" -I"C:\Users\Matthew Fonken\SimplicityStudio\v4_workspace\ble-module\system\utilities" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//platform/CMSIS/Include" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//platform/Device/SiliconLabs/EFR32BG1B/Include" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//platform/emdrv/common/inc" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//platform/emdrv/dmadrv/config" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//platform/emdrv/dmadrv/inc" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//platform/emdrv/gpiointerrupt/inc" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//platform/emdrv/nvm/config" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//platform/emdrv/nvm/inc" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//platform/emdrv/rtcdrv/config" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//platform/emdrv/rtcdrv/inc" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//platform/emdrv/sleep/inc" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//platform/emdrv/spidrv/config" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//platform/emdrv/spidrv/inc" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//platform/emdrv/tempdrv/config" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//platform/emdrv/tempdrv/inc" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//platform/emdrv/uartdrv/config" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//platform/emdrv/uartdrv/inc" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//platform/emdrv/ustimer/config" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//platform/emdrv/ustimer/inc" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//platform/emlib/inc" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//hardware/kit/common/bsp" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//hardware/kit/common/drivers" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//platform/radio/rail_lib/chip/efr32/rf/common/cortex" -I"C:/Users/Matthew Fonken/SimplicityStudio/v4_workspace/ble-module/src" -I"C:/Users/Matthew Fonken/SimplicityStudio/v4_workspace/ble-module" -I"C:\Users\Matthew Fonken\SimplicityStudio\v4_workspace\ble-module\src" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//protocol/bluetooth_2.1/ble_stack/inc/common" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/stacks/ble/v2.1.1.0//protocol/bluetooth_2.1/ble_stack/inc/soc" -e --cpu Cortex-M4F --fpu VFPv4_sp --debug --endian little --cpu_mode thumb -On --no_cse --no_unroll --no_inline --no_code_motion --no_tbaa --no_clustering --no_scheduling '-DSILABS_AF_USE_HWCONF=1' '-DGENERATION_DONE=1' '-D__NO_SYSTEM_INIT=1' '-DEFR32BG1B232F256GM32=1' --diag_suppress pa050 --diag_suppress pa050 --diag_error pe223 --diag_error pe223 --dependencies=m system/utilities/usart_sp.d
	@echo 'Finished building: $<'
	@echo ' '

