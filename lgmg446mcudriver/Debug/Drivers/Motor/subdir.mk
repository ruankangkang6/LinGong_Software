################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Motor/MotorAdc.c \
../Drivers/Motor/MotorCalibration.c \
../Drivers/Motor/MotorCarrier.c \
../Drivers/Motor/MotorMain.c \
../Drivers/Motor/MotorPWM.c \
../Drivers/Motor/MotorRotorPos.c \
../Drivers/Motor/MotorVC.c \
../Drivers/Motor/VehicleCtrl.c 

OBJS += \
./Drivers/Motor/MotorAdc.o \
./Drivers/Motor/MotorCalibration.o \
./Drivers/Motor/MotorCarrier.o \
./Drivers/Motor/MotorMain.o \
./Drivers/Motor/MotorPWM.o \
./Drivers/Motor/MotorRotorPos.o \
./Drivers/Motor/MotorVC.o \
./Drivers/Motor/VehicleCtrl.o 

C_DEPS += \
./Drivers/Motor/MotorAdc.d \
./Drivers/Motor/MotorCalibration.d \
./Drivers/Motor/MotorCarrier.d \
./Drivers/Motor/MotorMain.d \
./Drivers/Motor/MotorPWM.d \
./Drivers/Motor/MotorRotorPos.d \
./Drivers/Motor/MotorVC.d \
./Drivers/Motor/VehicleCtrl.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Motor/%.o Drivers/Motor/%.su Drivers/Motor/%.cyclo: ../Drivers/Motor/%.c Drivers/Motor/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../core/bsp -I../core/src -I../core/app/can -I../core/app/fbl -I"E:/Panovation_Document/LinGong/lgmg446mcudriver/lgmg446mcudriver/Drivers/Function/Include" -I"E:/Panovation_Document/LinGong/lgmg446mcudriver/lgmg446mcudriver/Drivers/BSW/Inc" -I"E:/Panovation_Document/LinGong/lgmg446mcudriver/lgmg446mcudriver/Drivers/Motor/Motor_include" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Motor

clean-Drivers-2f-Motor:
	-$(RM) ./Drivers/Motor/MotorAdc.cyclo ./Drivers/Motor/MotorAdc.d ./Drivers/Motor/MotorAdc.o ./Drivers/Motor/MotorAdc.su ./Drivers/Motor/MotorCalibration.cyclo ./Drivers/Motor/MotorCalibration.d ./Drivers/Motor/MotorCalibration.o ./Drivers/Motor/MotorCalibration.su ./Drivers/Motor/MotorCarrier.cyclo ./Drivers/Motor/MotorCarrier.d ./Drivers/Motor/MotorCarrier.o ./Drivers/Motor/MotorCarrier.su ./Drivers/Motor/MotorMain.cyclo ./Drivers/Motor/MotorMain.d ./Drivers/Motor/MotorMain.o ./Drivers/Motor/MotorMain.su ./Drivers/Motor/MotorPWM.cyclo ./Drivers/Motor/MotorPWM.d ./Drivers/Motor/MotorPWM.o ./Drivers/Motor/MotorPWM.su ./Drivers/Motor/MotorRotorPos.cyclo ./Drivers/Motor/MotorRotorPos.d ./Drivers/Motor/MotorRotorPos.o ./Drivers/Motor/MotorRotorPos.su ./Drivers/Motor/MotorVC.cyclo ./Drivers/Motor/MotorVC.d ./Drivers/Motor/MotorVC.o ./Drivers/Motor/MotorVC.su ./Drivers/Motor/VehicleCtrl.cyclo ./Drivers/Motor/VehicleCtrl.d ./Drivers/Motor/VehicleCtrl.o ./Drivers/Motor/VehicleCtrl.su

.PHONY: clean-Drivers-2f-Motor

