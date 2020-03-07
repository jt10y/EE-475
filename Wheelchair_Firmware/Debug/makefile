################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Wheelchair_Firmware.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Wheelchair_Firmware.list \

OBJCOPY_BIN += \
Wheelchair_Firmware.bin \


# All Target
all: Wheelchair_Firmware.elf secondary-outputs

# Tool invocations
Wheelchair_Firmware.elf: $(OBJS) $(USER_OBJS) /Users/frank/Documents/UW/Academics/Courses/20\ Q1/CSE\ 475/STM32/workspace/Wheelchair_Firmware/STM32F407VGTX_FLASH.ld
	arm-none-eabi-gcc -o "Wheelchair_Firmware.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/Users/frank/Documents/UW/Academics/Courses/20 Q1/CSE 475/STM32/workspace/Wheelchair_Firmware/STM32F407VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Wheelchair_Firmware.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Wheelchair_Firmware.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Wheelchair_Firmware.list"
	@echo 'Finished building: $@'
	@echo ' '

Wheelchair_Firmware.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Wheelchair_Firmware.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
