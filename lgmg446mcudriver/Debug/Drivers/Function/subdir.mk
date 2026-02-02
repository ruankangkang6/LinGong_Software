################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Function/TempCommnn.c \
../Drivers/Function/f_asrControl.c \
../Drivers/Function/f_deadBandComp.c \
../Drivers/Function/f_disCharge.c \
../Drivers/Function/f_eeprom.c \
../Drivers/Function/f_fanControl.c \
../Drivers/Function/f_faultProtect.c \
../Drivers/Function/f_fluxWeak.c \
../Drivers/Function/f_freqDeal.c \
../Drivers/Function/f_paramEst.c \
../Drivers/Function/f_public.c \
../Drivers/Function/f_runCaseDeal.c \
../Drivers/Function/f_tempCheck.c \
../Drivers/Function/f_torModSpdlimit.c \
../Drivers/Function/f_torqueComp.c \
../Drivers/Function/f_torqueDeal.c 

OBJS += \
./Drivers/Function/TempCommnn.o \
./Drivers/Function/f_asrControl.o \
./Drivers/Function/f_deadBandComp.o \
./Drivers/Function/f_disCharge.o \
./Drivers/Function/f_eeprom.o \
./Drivers/Function/f_fanControl.o \
./Drivers/Function/f_faultProtect.o \
./Drivers/Function/f_fluxWeak.o \
./Drivers/Function/f_freqDeal.o \
./Drivers/Function/f_paramEst.o \
./Drivers/Function/f_public.o \
./Drivers/Function/f_runCaseDeal.o \
./Drivers/Function/f_tempCheck.o \
./Drivers/Function/f_torModSpdlimit.o \
./Drivers/Function/f_torqueComp.o \
./Drivers/Function/f_torqueDeal.o 

C_DEPS += \
./Drivers/Function/TempCommnn.d \
./Drivers/Function/f_asrControl.d \
./Drivers/Function/f_deadBandComp.d \
./Drivers/Function/f_disCharge.d \
./Drivers/Function/f_eeprom.d \
./Drivers/Function/f_fanControl.d \
./Drivers/Function/f_faultProtect.d \
./Drivers/Function/f_fluxWeak.d \
./Drivers/Function/f_freqDeal.d \
./Drivers/Function/f_paramEst.d \
./Drivers/Function/f_public.d \
./Drivers/Function/f_runCaseDeal.d \
./Drivers/Function/f_tempCheck.d \
./Drivers/Function/f_torModSpdlimit.d \
./Drivers/Function/f_torqueComp.d \
./Drivers/Function/f_torqueDeal.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Function/%.o Drivers/Function/%.su Drivers/Function/%.cyclo: ../Drivers/Function/%.c Drivers/Function/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../core/bsp -I../core/src -I../core/app/can -I../core/app/fbl -I"E:/Panovation_Document/LinGong/lgmg446mcudriver/lgmg446mcudriver/Drivers/Function/Include" -I"E:/Panovation_Document/LinGong/lgmg446mcudriver/lgmg446mcudriver/Drivers/BSW/Inc" -I"E:/Panovation_Document/LinGong/lgmg446mcudriver/lgmg446mcudriver/Drivers/Motor/Motor_include" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Function

clean-Drivers-2f-Function:
	-$(RM) ./Drivers/Function/TempCommnn.cyclo ./Drivers/Function/TempCommnn.d ./Drivers/Function/TempCommnn.o ./Drivers/Function/TempCommnn.su ./Drivers/Function/f_asrControl.cyclo ./Drivers/Function/f_asrControl.d ./Drivers/Function/f_asrControl.o ./Drivers/Function/f_asrControl.su ./Drivers/Function/f_deadBandComp.cyclo ./Drivers/Function/f_deadBandComp.d ./Drivers/Function/f_deadBandComp.o ./Drivers/Function/f_deadBandComp.su ./Drivers/Function/f_disCharge.cyclo ./Drivers/Function/f_disCharge.d ./Drivers/Function/f_disCharge.o ./Drivers/Function/f_disCharge.su ./Drivers/Function/f_eeprom.cyclo ./Drivers/Function/f_eeprom.d ./Drivers/Function/f_eeprom.o ./Drivers/Function/f_eeprom.su ./Drivers/Function/f_fanControl.cyclo ./Drivers/Function/f_fanControl.d ./Drivers/Function/f_fanControl.o ./Drivers/Function/f_fanControl.su ./Drivers/Function/f_faultProtect.cyclo ./Drivers/Function/f_faultProtect.d ./Drivers/Function/f_faultProtect.o ./Drivers/Function/f_faultProtect.su ./Drivers/Function/f_fluxWeak.cyclo ./Drivers/Function/f_fluxWeak.d ./Drivers/Function/f_fluxWeak.o ./Drivers/Function/f_fluxWeak.su ./Drivers/Function/f_freqDeal.cyclo ./Drivers/Function/f_freqDeal.d ./Drivers/Function/f_freqDeal.o ./Drivers/Function/f_freqDeal.su ./Drivers/Function/f_paramEst.cyclo ./Drivers/Function/f_paramEst.d ./Drivers/Function/f_paramEst.o ./Drivers/Function/f_paramEst.su ./Drivers/Function/f_public.cyclo ./Drivers/Function/f_public.d ./Drivers/Function/f_public.o ./Drivers/Function/f_public.su ./Drivers/Function/f_runCaseDeal.cyclo ./Drivers/Function/f_runCaseDeal.d ./Drivers/Function/f_runCaseDeal.o ./Drivers/Function/f_runCaseDeal.su ./Drivers/Function/f_tempCheck.cyclo ./Drivers/Function/f_tempCheck.d ./Drivers/Function/f_tempCheck.o ./Drivers/Function/f_tempCheck.su ./Drivers/Function/f_torModSpdlimit.cyclo ./Drivers/Function/f_torModSpdlimit.d ./Drivers/Function/f_torModSpdlimit.o ./Drivers/Function/f_torModSpdlimit.su ./Drivers/Function/f_torqueComp.cyclo ./Drivers/Function/f_torqueComp.d ./Drivers/Function/f_torqueComp.o ./Drivers/Function/f_torqueComp.su ./Drivers/Function/f_torqueDeal.cyclo ./Drivers/Function/f_torqueDeal.d ./Drivers/Function/f_torqueDeal.o ./Drivers/Function/f_torqueDeal.su

.PHONY: clean-Drivers-2f-Function

