################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/hbridge.c \
../Core/Src/main.c \
../Core/Src/main_loop.c \
../Core/Src/main_preloop.c \
../Core/Src/mosfet.c \
../Core/Src/mux.c \
../Core/Src/servo.c \
../Core/Src/smbus.c \
../Core/Src/spectral.c \
../Core/Src/stm32g0xx_hal_msp.c \
../Core/Src/stm32g0xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32g0xx.c \
../Core/Src/thermistor.c \
../Core/Src/triad.c 

OBJS += \
./Core/Src/hbridge.o \
./Core/Src/main.o \
./Core/Src/main_loop.o \
./Core/Src/main_preloop.o \
./Core/Src/mosfet.o \
./Core/Src/mux.o \
./Core/Src/servo.o \
./Core/Src/smbus.o \
./Core/Src/spectral.o \
./Core/Src/stm32g0xx_hal_msp.o \
./Core/Src/stm32g0xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32g0xx.o \
./Core/Src/thermistor.o \
./Core/Src/triad.o 

C_DEPS += \
./Core/Src/hbridge.d \
./Core/Src/main.d \
./Core/Src/main_loop.d \
./Core/Src/main_preloop.d \
./Core/Src/mosfet.d \
./Core/Src/mux.d \
./Core/Src/servo.d \
./Core/Src/smbus.d \
./Core/Src/spectral.d \
./Core/Src/stm32g0xx_hal_msp.d \
./Core/Src/stm32g0xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32g0xx.d \
./Core/Src/thermistor.d \
./Core/Src/triad.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G050xx -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/hbridge.d ./Core/Src/hbridge.o ./Core/Src/hbridge.su ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/main_loop.d ./Core/Src/main_loop.o ./Core/Src/main_loop.su ./Core/Src/main_preloop.d ./Core/Src/main_preloop.o ./Core/Src/main_preloop.su ./Core/Src/mosfet.d ./Core/Src/mosfet.o ./Core/Src/mosfet.su ./Core/Src/mux.d ./Core/Src/mux.o ./Core/Src/mux.su ./Core/Src/servo.d ./Core/Src/servo.o ./Core/Src/servo.su ./Core/Src/smbus.d ./Core/Src/smbus.o ./Core/Src/smbus.su ./Core/Src/spectral.d ./Core/Src/spectral.o ./Core/Src/spectral.su ./Core/Src/stm32g0xx_hal_msp.d ./Core/Src/stm32g0xx_hal_msp.o ./Core/Src/stm32g0xx_hal_msp.su ./Core/Src/stm32g0xx_it.d ./Core/Src/stm32g0xx_it.o ./Core/Src/stm32g0xx_it.su ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32g0xx.d ./Core/Src/system_stm32g0xx.o ./Core/Src/system_stm32g0xx.su ./Core/Src/thermistor.d ./Core/Src/thermistor.o ./Core/Src/thermistor.su ./Core/Src/triad.d ./Core/Src/triad.o ./Core/Src/triad.su

.PHONY: clean-Core-2f-Src

