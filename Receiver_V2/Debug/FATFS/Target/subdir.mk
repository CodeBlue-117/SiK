################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../FATFS/Target/user_diskio.c \
../FATFS/Target/user_diskio_spi.c 

OBJS += \
./FATFS/Target/user_diskio.o \
./FATFS/Target/user_diskio_spi.o 

C_DEPS += \
./FATFS/Target/user_diskio.d \
./FATFS/Target/user_diskio_spi.d 


# Each subdirectory must supply rules for building sources it contributes
FATFS/Target/%.o FATFS/Target/%.su FATFS/Target/%.cyclo: ../FATFS/Target/%.c FATFS/Target/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WL55xx -c -I../Core/Inc -I"C:/Users/jake-/Documents/GitHub/StormEq/PLOW_BOX_PCB_CODE_V2/Drivers/CMSIS/Device/ST/STM32WLxx/Include" -I"C:/Users/jake-/Documents/GitHub/StormEq/PLOW_BOX_PCB_CODE_V2/Drivers/STM32WLxx_HAL_Driver/Inc" -I../../Drivers/STM32WLxx_HAL_Driver/Inc -I../Drivers/BSP/STM32WLxx_Nucleo -I../SubGHz_Phy/App -I../SubGHz_Phy/Target -I../Utilities/trace/adv_trace -I../../Drivers/STM32WLxx_HAL_Driver/Inc -I../Utilities/misc -I../Utilities/sequencer -I../Utilities/timer -I../Utilities/lpm/tiny_lpm -I"C:/Users/jake-/Documents/GitHub/StormEq/PLOW_BOX_PCB_CODE_V2/Drivers/CMSIS/Include" -I../Middlewares/Third_Party/SubGHz_Phy -I../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../Drivers/CMSIS/Include -I../Drivers/MEMS -I../Middlewares/Third_Party/FatFs/src -I../FATFS/App -I../FATFS/Target -I../Drivers/STM32WLxx_HAL_Driver/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WLxx/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-FATFS-2f-Target

clean-FATFS-2f-Target:
	-$(RM) ./FATFS/Target/user_diskio.cyclo ./FATFS/Target/user_diskio.d ./FATFS/Target/user_diskio.o ./FATFS/Target/user_diskio.su ./FATFS/Target/user_diskio_spi.cyclo ./FATFS/Target/user_diskio_spi.d ./FATFS/Target/user_diskio_spi.o ./FATFS/Target/user_diskio_spi.su

.PHONY: clean-FATFS-2f-Target

