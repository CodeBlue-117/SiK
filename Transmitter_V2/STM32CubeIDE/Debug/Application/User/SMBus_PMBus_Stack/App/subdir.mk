################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/Jake\ P/Documents/GitHub/StormEq/CAB_BOX_PCB_CODE_V2/SMBus_PMBus_Stack/App/app_X-CUBE-SMBUS.c 

OBJS += \
./Application/User/SMBus_PMBus_Stack/App/app_X-CUBE-SMBUS.o 

C_DEPS += \
./Application/User/SMBus_PMBus_Stack/App/app_X-CUBE-SMBUS.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/SMBus_PMBus_Stack/App/app_X-CUBE-SMBUS.o: C:/Users/Jake\ P/Documents/GitHub/StormEq/CAB_BOX_PCB_CODE_V2/SMBus_PMBus_Stack/App/app_X-CUBE-SMBUS.c Application/User/SMBus_PMBus_Stack/App/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WL55xx -c -I../../Core/Inc -I"C:/Users/Jake P/Documents/GitHub/StormEq/CAB_BOX_PCB_CODE_V2/STM32CubeIDE/Application/User/SubGHz_Phy/App" -I"C:/Users/Jake P/Documents/GitHub/StormEq/CAB_BOX_PCB_CODE_V2/STM32CubeIDE/Drivers/BSP/STM32WLxx_Nucleo" -I../../SubGHz_Phy/App -I../../SubGHz_Phy/Target -I../../Utilities/trace/adv_trace -I../../Drivers/STM32WLxx_HAL_Driver/Inc -I../../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../../Utilities/misc -I../../Utilities/sequencer -I../../Utilities/timer -I../../Utilities/lpm/tiny_lpm -I../../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../../Middlewares/Third_Party/SubGHz_Phy -I../../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../../Drivers/CMSIS/Include -I../../SMBus_PMBus_Stack/App -I../../SMBus_PMBus_Stack/Target -I../../Middlewares/ST/STM32_SMBus_Stack/inc -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Application/User/SMBus_PMBus_Stack/App/app_X-CUBE-SMBUS.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Application-2f-User-2f-SMBus_PMBus_Stack-2f-App

clean-Application-2f-User-2f-SMBus_PMBus_Stack-2f-App:
	-$(RM) ./Application/User/SMBus_PMBus_Stack/App/app_X-CUBE-SMBUS.cyclo ./Application/User/SMBus_PMBus_Stack/App/app_X-CUBE-SMBUS.d ./Application/User/SMBus_PMBus_Stack/App/app_X-CUBE-SMBUS.o ./Application/User/SMBus_PMBus_Stack/App/app_X-CUBE-SMBUS.su

.PHONY: clean-Application-2f-User-2f-SMBus_PMBus_Stack-2f-App

