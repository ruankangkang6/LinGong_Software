################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/APP/CAN/SDO_COB.c \
../Core/APP/CAN/TP.c \
../Core/APP/CAN/il.c \
../Core/APP/CAN/nm.c \
../Core/APP/CAN/uds.c 

OBJS += \
./Core/APP/CAN/SDO_COB.o \
./Core/APP/CAN/TP.o \
./Core/APP/CAN/il.o \
./Core/APP/CAN/nm.o \
./Core/APP/CAN/uds.o 

C_DEPS += \
./Core/APP/CAN/SDO_COB.d \
./Core/APP/CAN/TP.d \
./Core/APP/CAN/il.d \
./Core/APP/CAN/nm.d \
./Core/APP/CAN/uds.d 


# Each subdirectory must supply rules for building sources it contributes
Core/APP/CAN/%.o Core/APP/CAN/%.su Core/APP/CAN/%.cyclo: ../Core/APP/CAN/%.c Core/APP/CAN/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../core/bsp -I../core/src -I../core/app/can -I../core/app/fbl -I"E:/Panovation_Document/LinGong/lgmg446mcudriver/lgmg446mcudriver/Drivers/Function/Include" -I"E:/Panovation_Document/LinGong/lgmg446mcudriver/lgmg446mcudriver/Drivers/BSW/Inc" -I"E:/Panovation_Document/LinGong/lgmg446mcudriver/lgmg446mcudriver/Drivers/Motor/Motor_include" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-APP-2f-CAN

clean-Core-2f-APP-2f-CAN:
	-$(RM) ./Core/APP/CAN/SDO_COB.cyclo ./Core/APP/CAN/SDO_COB.d ./Core/APP/CAN/SDO_COB.o ./Core/APP/CAN/SDO_COB.su ./Core/APP/CAN/TP.cyclo ./Core/APP/CAN/TP.d ./Core/APP/CAN/TP.o ./Core/APP/CAN/TP.su ./Core/APP/CAN/il.cyclo ./Core/APP/CAN/il.d ./Core/APP/CAN/il.o ./Core/APP/CAN/il.su ./Core/APP/CAN/nm.cyclo ./Core/APP/CAN/nm.d ./Core/APP/CAN/nm.o ./Core/APP/CAN/nm.su ./Core/APP/CAN/uds.cyclo ./Core/APP/CAN/uds.d ./Core/APP/CAN/uds.o ./Core/APP/CAN/uds.su

.PHONY: clean-Core-2f-APP-2f-CAN

