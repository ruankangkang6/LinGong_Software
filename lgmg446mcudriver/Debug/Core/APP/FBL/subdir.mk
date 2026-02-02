################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/APP/FBL/fbl.c 

OBJS += \
./Core/APP/FBL/fbl.o 

C_DEPS += \
./Core/APP/FBL/fbl.d 


# Each subdirectory must supply rules for building sources it contributes
Core/APP/FBL/%.o Core/APP/FBL/%.su Core/APP/FBL/%.cyclo: ../Core/APP/FBL/%.c Core/APP/FBL/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../core/bsp -I../core/src -I../core/app/can -I../core/app/fbl -I"E:/Panovation_Document/LinGong/lgmg446mcudriver/lgmg446mcudriver/Drivers/Function/Include" -I"E:/Panovation_Document/LinGong/lgmg446mcudriver/lgmg446mcudriver/Drivers/BSW/Inc" -I"E:/Panovation_Document/LinGong/lgmg446mcudriver/lgmg446mcudriver/Drivers/Motor/Motor_include" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-APP-2f-FBL

clean-Core-2f-APP-2f-FBL:
	-$(RM) ./Core/APP/FBL/fbl.cyclo ./Core/APP/FBL/fbl.d ./Core/APP/FBL/fbl.o ./Core/APP/FBL/fbl.su

.PHONY: clean-Core-2f-APP-2f-FBL

