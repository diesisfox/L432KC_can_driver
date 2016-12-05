################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CMSIS/Device/ST/STM32L4xx/Source/Templates/system_stm32l4xx.c 

OBJS += \
./Drivers/CMSIS/Device/ST/STM32L4xx/Source/Templates/system_stm32l4xx.o 

C_DEPS += \
./Drivers/CMSIS/Device/ST/STM32L4xx/Source/Templates/system_stm32l4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/Device/ST/STM32L4xx/Source/Templates/%.o: ../Drivers/CMSIS/Device/ST/STM32L4xx/Source/Templates/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -DSTM32L432xx -I"/Users/jamesliu/Development/STM32/can_diag2/Inc" -I"/Users/jamesliu/Development/STM32/can_diag2/Drivers/STM32L4xx_HAL_Driver/Inc" -I"/Users/jamesliu/Development/STM32/can_diag2/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy" -I"/Users/jamesliu/Development/STM32/can_diag2/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"/Users/jamesliu/Development/STM32/can_diag2/Drivers/CMSIS/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


