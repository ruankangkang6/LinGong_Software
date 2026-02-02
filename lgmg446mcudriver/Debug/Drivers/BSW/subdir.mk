################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSW/I2C.c \
../Drivers/BSW/Interrupt.c \
../Drivers/BSW/MS5910.c \
../Drivers/BSW/Monitor.c \
../Drivers/BSW/MonitorPrm.c \
../Drivers/BSW/PWM.c \
../Drivers/BSW/PrmMgr.c \
../Drivers/BSW/Variables.c \
../Drivers/BSW/can2_cfg.c \
../Drivers/BSW/can_cfg.c \
../Drivers/BSW/f_carProtocolJR-SYNC.c \
../Drivers/BSW/f_mcuctrl.c 

OBJS += \
./Drivers/BSW/I2C.o \
./Drivers/BSW/Interrupt.o \
./Drivers/BSW/MS5910.o \
./Drivers/BSW/Monitor.o \
./Drivers/BSW/MonitorPrm.o \
./Drivers/BSW/PWM.o \
./Drivers/BSW/PrmMgr.o \
./Drivers/BSW/Variables.o \
./Drivers/BSW/can2_cfg.o \
./Drivers/BSW/can_cfg.o \
./Drivers/BSW/f_carProtocolJR-SYNC.o \
./Drivers/BSW/f_mcuctrl.o 

C_DEPS += \
./Drivers/BSW/I2C.d \
./Drivers/BSW/Interrupt.d \
./Drivers/BSW/MS5910.d \
./Drivers/BSW/Monitor.d \
./Drivers/BSW/MonitorPrm.d \
./Drivers/BSW/PWM.d \
./Drivers/BSW/PrmMgr.d \
./Drivers/BSW/Variables.d \
./Drivers/BSW/can2_cfg.d \
./Drivers/BSW/can_cfg.d \
./Drivers/BSW/f_carProtocolJR-SYNC.d \
./Drivers/BSW/f_mcuctrl.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSW/%.o Drivers/BSW/%.su Drivers/BSW/%.cyclo: ../Drivers/BSW/%.c Drivers/BSW/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../core/bsp -I../core/src -I../core/app/can -I../core/app/fbl -I"E:/Panovation_Document/LinGong/lgmg446mcudriver/lgmg446mcudriver/Drivers/Function/Include" -I"E:/Panovation_Document/LinGong/lgmg446mcudriver/lgmg446mcudriver/Drivers/BSW/Inc" -I"E:/Panovation_Document/LinGong/lgmg446mcudriver/lgmg446mcudriver/Drivers/Motor/Motor_include" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSW

clean-Drivers-2f-BSW:
	-$(RM) ./Drivers/BSW/I2C.cyclo ./Drivers/BSW/I2C.d ./Drivers/BSW/I2C.o ./Drivers/BSW/I2C.su ./Drivers/BSW/Interrupt.cyclo ./Drivers/BSW/Interrupt.d ./Drivers/BSW/Interrupt.o ./Drivers/BSW/Interrupt.su ./Drivers/BSW/MS5910.cyclo ./Drivers/BSW/MS5910.d ./Drivers/BSW/MS5910.o ./Drivers/BSW/MS5910.su ./Drivers/BSW/Monitor.cyclo ./Drivers/BSW/Monitor.d ./Drivers/BSW/Monitor.o ./Drivers/BSW/Monitor.su ./Drivers/BSW/MonitorPrm.cyclo ./Drivers/BSW/MonitorPrm.d ./Drivers/BSW/MonitorPrm.o ./Drivers/BSW/MonitorPrm.su ./Drivers/BSW/PWM.cyclo ./Drivers/BSW/PWM.d ./Drivers/BSW/PWM.o ./Drivers/BSW/PWM.su ./Drivers/BSW/PrmMgr.cyclo ./Drivers/BSW/PrmMgr.d ./Drivers/BSW/PrmMgr.o ./Drivers/BSW/PrmMgr.su ./Drivers/BSW/Variables.cyclo ./Drivers/BSW/Variables.d ./Drivers/BSW/Variables.o ./Drivers/BSW/Variables.su ./Drivers/BSW/can2_cfg.cyclo ./Drivers/BSW/can2_cfg.d ./Drivers/BSW/can2_cfg.o ./Drivers/BSW/can2_cfg.su ./Drivers/BSW/can_cfg.cyclo ./Drivers/BSW/can_cfg.d ./Drivers/BSW/can_cfg.o ./Drivers/BSW/can_cfg.su ./Drivers/BSW/f_carProtocolJR-SYNC.cyclo ./Drivers/BSW/f_carProtocolJR-SYNC.d ./Drivers/BSW/f_carProtocolJR-SYNC.o ./Drivers/BSW/f_carProtocolJR-SYNC.su ./Drivers/BSW/f_mcuctrl.cyclo ./Drivers/BSW/f_mcuctrl.d ./Drivers/BSW/f_mcuctrl.o ./Drivers/BSW/f_mcuctrl.su

.PHONY: clean-Drivers-2f-BSW

