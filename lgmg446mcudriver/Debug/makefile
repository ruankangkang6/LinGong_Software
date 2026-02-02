################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Drivers/Motor/subdir.mk
-include Drivers/Function/subdir.mk
-include Drivers/BSW/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include Core/BSP/subdir.mk
-include Core/APP/FBL/subdir.mk
-include Core/APP/CAN/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := PM2in1_DriverMCU
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
PM2in1_DriverMCU.elf \

MAP_FILES += \
PM2in1_DriverMCU.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
PM2in1_DriverMCU.list \

OBJCOPY_HEX += \
PM2in1_DriverMCU.hex \


# All Target
all: main-build

# Main-build Target
main-build: PM2in1_DriverMCU.elf secondary-outputs

# Tool invocations
PM2in1_DriverMCU.elf PM2in1_DriverMCU.map: $(OBJS) $(USER_OBJS) E:\Panovation_Document\LinGong\lgmg446mcudriver\lgmg446mcudriver\STM32F446RCTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "PM2in1_DriverMCU.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"E:\Panovation_Document\LinGong\lgmg446mcudriver\lgmg446mcudriver\STM32F446RCTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="PM2in1_DriverMCU.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

PM2in1_DriverMCU.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "PM2in1_DriverMCU.list"
	@echo 'Finished building: $@'
	@echo ' '

PM2in1_DriverMCU.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "PM2in1_DriverMCU.hex"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) PM2in1_DriverMCU.elf PM2in1_DriverMCU.hex PM2in1_DriverMCU.list PM2in1_DriverMCU.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
