################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/BSP/can.c 

OBJS += \
./Core/BSP/can.o 

C_DEPS += \
./Core/BSP/can.d 


# Each subdirectory must supply rules for building sources it contributes
Core/BSP/%.o Core/BSP/%.su Core/BSP/%.cyclo: ../Core/BSP/%.c Core/BSP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../core/bsp -I../core/src -I../core/app/can -I../core/app/fbl -I"E:/Panovation_Document/LinGong/lgmg446mcudriver/lgmg446mcudriver/Drivers/Function/Include" -I"E:/Panovation_Document/LinGong/lgmg446mcudriver/lgmg446mcudriver/Drivers/BSW/Inc" -I"E:/Panovation_Document/LinGong/lgmg446mcudriver/lgmg446mcudriver/Drivers/Motor/Motor_include" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-BSP

clean-Core-2f-BSP:
	-$(RM) ./Core/BSP/can.cyclo ./Core/BSP/can.d ./Core/BSP/can.o ./Core/BSP/can.su

.PHONY: clean-Core-2f-BSP

