################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/FatFs/src/option/syscall.c 

OBJS += \
./Middlewares/Third_Party/FatFs/src/option/syscall.o 

C_DEPS += \
./Middlewares/Third_Party/FatFs/src/option/syscall.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/FatFs/src/option/%.o Middlewares/Third_Party/FatFs/src/option/%.su Middlewares/Third_Party/FatFs/src/option/%.cyclo: ../Middlewares/Third_Party/FatFs/src/option/%.c Middlewares/Third_Party/FatFs/src/option/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WL55xx -c -I../Core/Inc -I"C:/Users/jake-/Documents/GitHub/StormEq/PLOW_BOX_PCB_CODE_V2/Drivers/CMSIS/Device/ST/STM32WLxx/Include" -I"C:/Users/jake-/Documents/GitHub/StormEq/PLOW_BOX_PCB_CODE_V2/Drivers/STM32WLxx_HAL_Driver/Inc" -I../../Drivers/STM32WLxx_HAL_Driver/Inc -I../Drivers/BSP/STM32WLxx_Nucleo -I../SubGHz_Phy/App -I../SubGHz_Phy/Target -I../Utilities/trace/adv_trace -I../../Drivers/STM32WLxx_HAL_Driver/Inc -I../Utilities/misc -I../Utilities/sequencer -I../Utilities/timer -I../Utilities/lpm/tiny_lpm -I"C:/Users/jake-/Documents/GitHub/StormEq/PLOW_BOX_PCB_CODE_V2/Drivers/CMSIS/Include" -I../Middlewares/Third_Party/SubGHz_Phy -I../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../Drivers/CMSIS/Include -I../Drivers/MEMS -I../Middlewares/Third_Party/FatFs/src -I../FATFS/App -I../FATFS/Target -I../Drivers/STM32WLxx_HAL_Driver/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WLxx/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-FatFs-2f-src-2f-option

clean-Middlewares-2f-Third_Party-2f-FatFs-2f-src-2f-option:
	-$(RM) ./Middlewares/Third_Party/FatFs/src/option/syscall.cyclo ./Middlewares/Third_Party/FatFs/src/option/syscall.d ./Middlewares/Third_Party/FatFs/src/option/syscall.o ./Middlewares/Third_Party/FatFs/src/option/syscall.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-FatFs-2f-src-2f-option

