################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/adc_sensor.c \
../Core/Src/app_freertos.c \
../Core/Src/bridge.c \
../Core/Src/diag_curr_sensor.c \
../Core/Src/diag_temp_sensor.c \
../Core/Src/heater.c \
../Core/Src/main.c \
../Core/Src/pin_data.c \
../Core/Src/servo.c \
../Core/Src/smbus.c \
../Core/Src/spectral.c \
../Core/Src/stm32g0xx_hal_msp.c \
../Core/Src/stm32g0xx_hal_timebase_tim.c \
../Core/Src/stm32g0xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32g0xx.c \
../Core/Src/thermistor.c 

OBJS += \
./Core/Src/adc_sensor.o \
./Core/Src/app_freertos.o \
./Core/Src/bridge.o \
./Core/Src/diag_curr_sensor.o \
./Core/Src/diag_temp_sensor.o \
./Core/Src/heater.o \
./Core/Src/main.o \
./Core/Src/pin_data.o \
./Core/Src/servo.o \
./Core/Src/smbus.o \
./Core/Src/spectral.o \
./Core/Src/stm32g0xx_hal_msp.o \
./Core/Src/stm32g0xx_hal_timebase_tim.o \
./Core/Src/stm32g0xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32g0xx.o \
./Core/Src/thermistor.o 

C_DEPS += \
./Core/Src/adc_sensor.d \
./Core/Src/app_freertos.d \
./Core/Src/bridge.d \
./Core/Src/diag_curr_sensor.d \
./Core/Src/diag_temp_sensor.d \
./Core/Src/heater.d \
./Core/Src/main.d \
./Core/Src/pin_data.d \
./Core/Src/servo.d \
./Core/Src/smbus.d \
./Core/Src/spectral.d \
./Core/Src/stm32g0xx_hal_msp.d \
./Core/Src/stm32g0xx_hal_timebase_tim.d \
./Core/Src/stm32g0xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32g0xx.d \
./Core/Src/thermistor.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G0B1xx '-DCMSIS_device_header=<stm32g0xx.h>' -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/adc_sensor.d ./Core/Src/adc_sensor.o ./Core/Src/adc_sensor.su ./Core/Src/app_freertos.d ./Core/Src/app_freertos.o ./Core/Src/app_freertos.su ./Core/Src/bridge.d ./Core/Src/bridge.o ./Core/Src/bridge.su ./Core/Src/diag_curr_sensor.d ./Core/Src/diag_curr_sensor.o ./Core/Src/diag_curr_sensor.su ./Core/Src/diag_temp_sensor.d ./Core/Src/diag_temp_sensor.o ./Core/Src/diag_temp_sensor.su ./Core/Src/heater.d ./Core/Src/heater.o ./Core/Src/heater.su ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/pin_data.d ./Core/Src/pin_data.o ./Core/Src/pin_data.su ./Core/Src/servo.d ./Core/Src/servo.o ./Core/Src/servo.su ./Core/Src/smbus.d ./Core/Src/smbus.o ./Core/Src/smbus.su ./Core/Src/spectral.d ./Core/Src/spectral.o ./Core/Src/spectral.su ./Core/Src/stm32g0xx_hal_msp.d ./Core/Src/stm32g0xx_hal_msp.o ./Core/Src/stm32g0xx_hal_msp.su ./Core/Src/stm32g0xx_hal_timebase_tim.d ./Core/Src/stm32g0xx_hal_timebase_tim.o ./Core/Src/stm32g0xx_hal_timebase_tim.su ./Core/Src/stm32g0xx_it.d ./Core/Src/stm32g0xx_it.o ./Core/Src/stm32g0xx_it.su ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32g0xx.d ./Core/Src/system_stm32g0xx.o ./Core/Src/system_stm32g0xx.su ./Core/Src/thermistor.d ./Core/Src/thermistor.o ./Core/Src/thermistor.su

.PHONY: clean-Core-2f-Src

