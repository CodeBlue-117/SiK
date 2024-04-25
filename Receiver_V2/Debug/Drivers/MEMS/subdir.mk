################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/MEMS/app_mems.c \
../Drivers/MEMS/iks01a3_env_sensors.c \
../Drivers/MEMS/iks01a3_env_sensors_ex.c \
../Drivers/MEMS/iks01a3_hybrid_sensors.c \
../Drivers/MEMS/iks01a3_hybrid_sensors_ex.c \
../Drivers/MEMS/iks01a3_motion_sensors.c \
../Drivers/MEMS/iks01a3_motion_sensors_ex.c \
../Drivers/MEMS/lsm6dso.c \
../Drivers/MEMS/lsm6dso_reg.c 

OBJS += \
./Drivers/MEMS/app_mems.o \
./Drivers/MEMS/iks01a3_env_sensors.o \
./Drivers/MEMS/iks01a3_env_sensors_ex.o \
./Drivers/MEMS/iks01a3_hybrid_sensors.o \
./Drivers/MEMS/iks01a3_hybrid_sensors_ex.o \
./Drivers/MEMS/iks01a3_motion_sensors.o \
./Drivers/MEMS/iks01a3_motion_sensors_ex.o \
./Drivers/MEMS/lsm6dso.o \
./Drivers/MEMS/lsm6dso_reg.o 

C_DEPS += \
./Drivers/MEMS/app_mems.d \
./Drivers/MEMS/iks01a3_env_sensors.d \
./Drivers/MEMS/iks01a3_env_sensors_ex.d \
./Drivers/MEMS/iks01a3_hybrid_sensors.d \
./Drivers/MEMS/iks01a3_hybrid_sensors_ex.d \
./Drivers/MEMS/iks01a3_motion_sensors.d \
./Drivers/MEMS/iks01a3_motion_sensors_ex.d \
./Drivers/MEMS/lsm6dso.d \
./Drivers/MEMS/lsm6dso_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/MEMS/%.o Drivers/MEMS/%.su Drivers/MEMS/%.cyclo: ../Drivers/MEMS/%.c Drivers/MEMS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WL55xx -c -I../Core/Inc -I"C:/Users/jake-/Documents/GitHub/StormEq/PLOW_BOX_PCB_CODE_V2/Drivers/CMSIS/Device/ST/STM32WLxx/Include" -I"C:/Users/jake-/Documents/GitHub/StormEq/PLOW_BOX_PCB_CODE_V2/Drivers/STM32WLxx_HAL_Driver/Inc" -I../../Drivers/STM32WLxx_HAL_Driver/Inc -I../Drivers/BSP/STM32WLxx_Nucleo -I../SubGHz_Phy/App -I../SubGHz_Phy/Target -I../Utilities/trace/adv_trace -I../../Drivers/STM32WLxx_HAL_Driver/Inc -I../Utilities/misc -I../Utilities/sequencer -I../Utilities/timer -I../Utilities/lpm/tiny_lpm -I"C:/Users/jake-/Documents/GitHub/StormEq/PLOW_BOX_PCB_CODE_V2/Drivers/CMSIS/Include" -I../Middlewares/Third_Party/SubGHz_Phy -I../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../Drivers/CMSIS/Include -I../Drivers/MEMS -I../Middlewares/Third_Party/FatFs/src -I../FATFS/App -I../FATFS/Target -I../Drivers/STM32WLxx_HAL_Driver/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WLxx/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-MEMS

clean-Drivers-2f-MEMS:
	-$(RM) ./Drivers/MEMS/app_mems.cyclo ./Drivers/MEMS/app_mems.d ./Drivers/MEMS/app_mems.o ./Drivers/MEMS/app_mems.su ./Drivers/MEMS/iks01a3_env_sensors.cyclo ./Drivers/MEMS/iks01a3_env_sensors.d ./Drivers/MEMS/iks01a3_env_sensors.o ./Drivers/MEMS/iks01a3_env_sensors.su ./Drivers/MEMS/iks01a3_env_sensors_ex.cyclo ./Drivers/MEMS/iks01a3_env_sensors_ex.d ./Drivers/MEMS/iks01a3_env_sensors_ex.o ./Drivers/MEMS/iks01a3_env_sensors_ex.su ./Drivers/MEMS/iks01a3_hybrid_sensors.cyclo ./Drivers/MEMS/iks01a3_hybrid_sensors.d ./Drivers/MEMS/iks01a3_hybrid_sensors.o ./Drivers/MEMS/iks01a3_hybrid_sensors.su ./Drivers/MEMS/iks01a3_hybrid_sensors_ex.cyclo ./Drivers/MEMS/iks01a3_hybrid_sensors_ex.d ./Drivers/MEMS/iks01a3_hybrid_sensors_ex.o ./Drivers/MEMS/iks01a3_hybrid_sensors_ex.su ./Drivers/MEMS/iks01a3_motion_sensors.cyclo ./Drivers/MEMS/iks01a3_motion_sensors.d ./Drivers/MEMS/iks01a3_motion_sensors.o ./Drivers/MEMS/iks01a3_motion_sensors.su ./Drivers/MEMS/iks01a3_motion_sensors_ex.cyclo ./Drivers/MEMS/iks01a3_motion_sensors_ex.d ./Drivers/MEMS/iks01a3_motion_sensors_ex.o ./Drivers/MEMS/iks01a3_motion_sensors_ex.su ./Drivers/MEMS/lsm6dso.cyclo ./Drivers/MEMS/lsm6dso.d ./Drivers/MEMS/lsm6dso.o ./Drivers/MEMS/lsm6dso.su ./Drivers/MEMS/lsm6dso_reg.cyclo ./Drivers/MEMS/lsm6dso_reg.d ./Drivers/MEMS/lsm6dso_reg.o ./Drivers/MEMS/lsm6dso_reg.su

.PHONY: clean-Drivers-2f-MEMS

