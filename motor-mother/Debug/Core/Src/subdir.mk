################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/abs_enc_reading.c \
../Core/Src/closed_loop_control.c \
../Core/Src/hbridge.c \
../Core/Src/i2c_bridge.c \
../Core/Src/limit_switch.c \
../Core/Src/main.c \
../Core/Src/motor.c \
../Core/Src/pin.c \
../Core/Src/quad_encoder.c \
../Core/Src/stm32f1xx_hal_msp.c \
../Core/Src/stm32f1xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f1xx.c 

OBJS += \
./Core/Src/abs_enc_reading.o \
./Core/Src/closed_loop_control.o \
./Core/Src/hbridge.o \
./Core/Src/i2c_bridge.o \
./Core/Src/limit_switch.o \
./Core/Src/main.o \
./Core/Src/motor.o \
./Core/Src/pin.o \
./Core/Src/quad_encoder.o \
./Core/Src/stm32f1xx_hal_msp.o \
./Core/Src/stm32f1xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f1xx.o 

C_DEPS += \
./Core/Src/abs_enc_reading.d \
./Core/Src/closed_loop_control.d \
./Core/Src/hbridge.d \
./Core/Src/i2c_bridge.d \
./Core/Src/limit_switch.d \
./Core/Src/main.d \
./Core/Src/motor.d \
./Core/Src/pin.d \
./Core/Src/quad_encoder.d \
./Core/Src/stm32f1xx_hal_msp.d \
./Core/Src/stm32f1xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f1xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F100xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/abs_enc_reading.cyclo ./Core/Src/abs_enc_reading.d ./Core/Src/abs_enc_reading.o ./Core/Src/abs_enc_reading.su ./Core/Src/closed_loop_control.cyclo ./Core/Src/closed_loop_control.d ./Core/Src/closed_loop_control.o ./Core/Src/closed_loop_control.su ./Core/Src/hbridge.cyclo ./Core/Src/hbridge.d ./Core/Src/hbridge.o ./Core/Src/hbridge.su ./Core/Src/i2c_bridge.cyclo ./Core/Src/i2c_bridge.d ./Core/Src/i2c_bridge.o ./Core/Src/i2c_bridge.su ./Core/Src/limit_switch.cyclo ./Core/Src/limit_switch.d ./Core/Src/limit_switch.o ./Core/Src/limit_switch.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/motor.cyclo ./Core/Src/motor.d ./Core/Src/motor.o ./Core/Src/motor.su ./Core/Src/pin.cyclo ./Core/Src/pin.d ./Core/Src/pin.o ./Core/Src/pin.su ./Core/Src/quad_encoder.cyclo ./Core/Src/quad_encoder.d ./Core/Src/quad_encoder.o ./Core/Src/quad_encoder.su ./Core/Src/stm32f1xx_hal_msp.cyclo ./Core/Src/stm32f1xx_hal_msp.d ./Core/Src/stm32f1xx_hal_msp.o ./Core/Src/stm32f1xx_hal_msp.su ./Core/Src/stm32f1xx_it.cyclo ./Core/Src/stm32f1xx_it.d ./Core/Src/stm32f1xx_it.o ./Core/Src/stm32f1xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f1xx.cyclo ./Core/Src/system_stm32f1xx.d ./Core/Src/system_stm32f1xx.o ./Core/Src/system_stm32f1xx.su

.PHONY: clean-Core-2f-Src

