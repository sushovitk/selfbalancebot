################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/pixy2.cpp 

C_SRCS += \
../Core/Src/BNO055.c \
../Core/Src/calib_flash.c \
../Core/Src/hbridge.c \
../Core/Src/main.c \
../Core/Src/pid.c \
../Core/Src/pixy.c \
../Core/Src/stm32l4xx_hal_msp.c \
../Core/Src/stm32l4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32l4xx.c \
../Core/Src/vl53l0x_api.c \
../Core/Src/vl53l0x_api_calibration.c \
../Core/Src/vl53l0x_api_core.c \
../Core/Src/vl53l0x_api_ranging.c \
../Core/Src/vl53l0x_api_strings.c \
../Core/Src/vl53l0x_platform.c 

C_DEPS += \
./Core/Src/BNO055.d \
./Core/Src/calib_flash.d \
./Core/Src/hbridge.d \
./Core/Src/main.d \
./Core/Src/pid.d \
./Core/Src/pixy.d \
./Core/Src/stm32l4xx_hal_msp.d \
./Core/Src/stm32l4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32l4xx.d \
./Core/Src/vl53l0x_api.d \
./Core/Src/vl53l0x_api_calibration.d \
./Core/Src/vl53l0x_api_core.d \
./Core/Src/vl53l0x_api_ranging.d \
./Core/Src/vl53l0x_api_strings.d \
./Core/Src/vl53l0x_platform.d 

OBJS += \
./Core/Src/BNO055.o \
./Core/Src/calib_flash.o \
./Core/Src/hbridge.o \
./Core/Src/main.o \
./Core/Src/pid.o \
./Core/Src/pixy.o \
./Core/Src/pixy2.o \
./Core/Src/stm32l4xx_hal_msp.o \
./Core/Src/stm32l4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32l4xx.o \
./Core/Src/vl53l0x_api.o \
./Core/Src/vl53l0x_api_calibration.o \
./Core/Src/vl53l0x_api_core.o \
./Core/Src/vl53l0x_api_ranging.o \
./Core/Src/vl53l0x_api_strings.o \
./Core/Src/vl53l0x_platform.o 

CPP_DEPS += \
./Core/Src/pixy2.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4R5xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.cpp Core/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4R5xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/BNO055.cyclo ./Core/Src/BNO055.d ./Core/Src/BNO055.o ./Core/Src/BNO055.su ./Core/Src/calib_flash.cyclo ./Core/Src/calib_flash.d ./Core/Src/calib_flash.o ./Core/Src/calib_flash.su ./Core/Src/hbridge.cyclo ./Core/Src/hbridge.d ./Core/Src/hbridge.o ./Core/Src/hbridge.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/pid.cyclo ./Core/Src/pid.d ./Core/Src/pid.o ./Core/Src/pid.su ./Core/Src/pixy.cyclo ./Core/Src/pixy.d ./Core/Src/pixy.o ./Core/Src/pixy.su ./Core/Src/pixy2.cyclo ./Core/Src/pixy2.d ./Core/Src/pixy2.o ./Core/Src/pixy2.su ./Core/Src/stm32l4xx_hal_msp.cyclo ./Core/Src/stm32l4xx_hal_msp.d ./Core/Src/stm32l4xx_hal_msp.o ./Core/Src/stm32l4xx_hal_msp.su ./Core/Src/stm32l4xx_it.cyclo ./Core/Src/stm32l4xx_it.d ./Core/Src/stm32l4xx_it.o ./Core/Src/stm32l4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32l4xx.cyclo ./Core/Src/system_stm32l4xx.d ./Core/Src/system_stm32l4xx.o ./Core/Src/system_stm32l4xx.su ./Core/Src/vl53l0x_api.cyclo ./Core/Src/vl53l0x_api.d ./Core/Src/vl53l0x_api.o ./Core/Src/vl53l0x_api.su ./Core/Src/vl53l0x_api_calibration.cyclo ./Core/Src/vl53l0x_api_calibration.d ./Core/Src/vl53l0x_api_calibration.o ./Core/Src/vl53l0x_api_calibration.su ./Core/Src/vl53l0x_api_core.cyclo ./Core/Src/vl53l0x_api_core.d ./Core/Src/vl53l0x_api_core.o ./Core/Src/vl53l0x_api_core.su ./Core/Src/vl53l0x_api_ranging.cyclo ./Core/Src/vl53l0x_api_ranging.d ./Core/Src/vl53l0x_api_ranging.o ./Core/Src/vl53l0x_api_ranging.su ./Core/Src/vl53l0x_api_strings.cyclo ./Core/Src/vl53l0x_api_strings.d ./Core/Src/vl53l0x_api_strings.o ./Core/Src/vl53l0x_api_strings.su ./Core/Src/vl53l0x_platform.cyclo ./Core/Src/vl53l0x_platform.d ./Core/Src/vl53l0x_platform.o ./Core/Src/vl53l0x_platform.su

.PHONY: clean-Core-2f-Src

