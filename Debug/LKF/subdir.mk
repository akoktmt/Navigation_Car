################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../LKF/EKF.c \
../LKF/EKFmath.c \
../LKF/GPS_Measurement.c \
../LKF/Heading_Measurement.c 

OBJS += \
./LKF/EKF.o \
./LKF/EKFmath.o \
./LKF/GPS_Measurement.o \
./LKF/Heading_Measurement.o 

C_DEPS += \
./LKF/EKF.d \
./LKF/EKFmath.d \
./LKF/GPS_Measurement.d \
./LKF/Heading_Measurement.d 


# Each subdirectory must supply rules for building sources it contributes
LKF/%.o LKF/%.su LKF/%.cyclo: ../LKF/%.c LKF/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../GPS_Handle -I../LKF -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-LKF

clean-LKF:
	-$(RM) ./LKF/EKF.cyclo ./LKF/EKF.d ./LKF/EKF.o ./LKF/EKF.su ./LKF/EKFmath.cyclo ./LKF/EKFmath.d ./LKF/EKFmath.o ./LKF/EKFmath.su ./LKF/GPS_Measurement.cyclo ./LKF/GPS_Measurement.d ./LKF/GPS_Measurement.o ./LKF/GPS_Measurement.su ./LKF/Heading_Measurement.cyclo ./LKF/Heading_Measurement.d ./LKF/Heading_Measurement.o ./LKF/Heading_Measurement.su

.PHONY: clean-LKF

