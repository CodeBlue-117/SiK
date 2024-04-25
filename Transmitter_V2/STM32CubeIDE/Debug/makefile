################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Utilities/subdir.mk
-include Middlewares/SubGHz_Phy/subdir.mk
-include Drivers/STM32WLxx_HAL_Driver/subdir.mk
-include Drivers/CMSIS/subdir.mk
-include Drivers/BSP/STM32WLxx_Nucleo/subdir.mk
-include Application/User/SubGHz_Phy/Target/subdir.mk
-include Application/User/SubGHz_Phy/App/subdir.mk
-include Application/User/Startup/subdir.mk
-include Application/User/Core/subdir.mk
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


BUILD_ARTIFACT_NAME := CAB_BOX_PCB_CODE_V2
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
CAB_BOX_PCB_CODE_V2.elf \

MAP_FILES += \
CAB_BOX_PCB_CODE_V2.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
CAB_BOX_PCB_CODE_V2.list \


# All Target
all: main-build

# Main-build Target
main-build: CAB_BOX_PCB_CODE_V2.elf secondary-outputs

# Tool invocations
CAB_BOX_PCB_CODE_V2.elf CAB_BOX_PCB_CODE_V2.map: $(OBJS) $(USER_OBJS) C:\Users\Jake\ P\Documents\GitHub\StormEq\CAB_BOX_PCB_CODE_V2\STM32CubeIDE\STM32WL55CCUX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "CAB_BOX_PCB_CODE_V2.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\Jake P\Documents\GitHub\StormEq\CAB_BOX_PCB_CODE_V2\STM32CubeIDE\STM32WL55CCUX_FLASH.ld" --specs=nosys.specs -Wl,-Map="CAB_BOX_PCB_CODE_V2.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -u _printf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

CAB_BOX_PCB_CODE_V2.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "CAB_BOX_PCB_CODE_V2.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) CAB_BOX_PCB_CODE_V2.elf CAB_BOX_PCB_CODE_V2.list CAB_BOX_PCB_CODE_V2.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
