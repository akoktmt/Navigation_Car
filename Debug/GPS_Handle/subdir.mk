################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../GPS_Handle/GPS.c \
../GPS_Handle/GPS_Handle.c 

OBJS += \
./GPS_Handle/GPS.o \
./GPS_Handle/GPS_Handle.o 

C_DEPS += \
./GPS_Handle/GPS.d \
./GPS_Handle/GPS_Handle.d 


# Each subdirectory must supply rules for building sources it contributes
GPS_Handle/%.o GPS_Handle/%.su GPS_Handle/%.cyclo: ../GPS_Handle/%.c GPS_Handle/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../GPS_Handle -I../LKF -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-GPS_Handle

clean-GPS_Handle:
	-$(RM) ./GPS_Handle/GPS.cyclo ./GPS_Handle/GPS.d ./GPS_Handle/GPS.o ./GPS_Handle/GPS.su ./GPS_Handle/GPS_Handle.cyclo ./GPS_Handle/GPS_Handle.d ./GPS_Handle/GPS_Handle.o ./GPS_Handle/GPS_Handle.su

.PHONY: clean-GPS_Handle

