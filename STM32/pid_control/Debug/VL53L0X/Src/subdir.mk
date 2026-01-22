################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../VL53L0X/Src/VL53L0X.c 

OBJS += \
./VL53L0X/Src/VL53L0X.o 

C_DEPS += \
./VL53L0X/Src/VL53L0X.d 


# Each subdirectory must supply rules for building sources it contributes
VL53L0X/Src/%.o VL53L0X/Src/%.su VL53L0X/Src/%.cyclo: ../VL53L0X/Src/%.c VL53L0X/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I"C:/Users/vndj0/STM32CubeIDE/workspace_1.19.0/pid_control/VL53L0X/Inc" -I"C:/Users/vndj0/STM32CubeIDE/workspace_1.19.0/pid_control/VL53L0X/Src" -I"C:/Users/vndj0/STM32CubeIDE/workspace_1.19.0/pid_control/VL53L0X" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-VL53L0X-2f-Src

clean-VL53L0X-2f-Src:
	-$(RM) ./VL53L0X/Src/VL53L0X.cyclo ./VL53L0X/Src/VL53L0X.d ./VL53L0X/Src/VL53L0X.o ./VL53L0X/Src/VL53L0X.su

.PHONY: clean-VL53L0X-2f-Src

