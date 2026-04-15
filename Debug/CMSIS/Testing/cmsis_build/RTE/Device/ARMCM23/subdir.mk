################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/Users/josnyder/dev/CMSIS-DSP/Testing/cmsis_build/RTE/Device/ARMCM23/startup_ARMCM23.c \
/Users/josnyder/dev/CMSIS-DSP/Testing/cmsis_build/RTE/Device/ARMCM23/system_ARMCM23.c 

S_UPPER_SRCS += \
/Users/josnyder/dev/CMSIS-DSP/Testing/cmsis_build/RTE/Device/ARMCM23/startup_ARMCM23.S 

OBJS += \
./CMSIS/Testing/cmsis_build/RTE/Device/ARMCM23/startup_ARMCM23.o \
./CMSIS/Testing/cmsis_build/RTE/Device/ARMCM23/system_ARMCM23.o 

S_UPPER_DEPS += \
./CMSIS/Testing/cmsis_build/RTE/Device/ARMCM23/startup_ARMCM23.d 

C_DEPS += \
./CMSIS/Testing/cmsis_build/RTE/Device/ARMCM23/startup_ARMCM23.d \
./CMSIS/Testing/cmsis_build/RTE/Device/ARMCM23/system_ARMCM23.d 


# Each subdirectory must supply rules for building sources it contributes
CMSIS/Testing/cmsis_build/RTE/Device/ARMCM23/startup_ARMCM23.o: /Users/josnyder/dev/CMSIS-DSP/Testing/cmsis_build/RTE/Device/ARMCM23/startup_ARMCM23.S CMSIS/Testing/cmsis_build/RTE/Device/ARMCM23/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m7 -g -DDEBUG -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"
CMSIS/Testing/cmsis_build/RTE/Device/ARMCM23/startup_ARMCM23.o: /Users/josnyder/dev/CMSIS-DSP/Testing/cmsis_build/RTE/Device/ARMCM23/startup_ARMCM23.c CMSIS/Testing/cmsis_build/RTE/Device/ARMCM23/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../Drivers/CMSIS/Include -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Testing/cmsis_build/RTE/Device/ARMCM23/system_ARMCM23.o: /Users/josnyder/dev/CMSIS-DSP/Testing/cmsis_build/RTE/Device/ARMCM23/system_ARMCM23.c CMSIS/Testing/cmsis_build/RTE/Device/ARMCM23/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../Drivers/CMSIS/Include -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-CMSIS-2f-Testing-2f-cmsis_build-2f-RTE-2f-Device-2f-ARMCM23

clean-CMSIS-2f-Testing-2f-cmsis_build-2f-RTE-2f-Device-2f-ARMCM23:
	-$(RM) ./CMSIS/Testing/cmsis_build/RTE/Device/ARMCM23/startup_ARMCM23.cyclo ./CMSIS/Testing/cmsis_build/RTE/Device/ARMCM23/startup_ARMCM23.d ./CMSIS/Testing/cmsis_build/RTE/Device/ARMCM23/startup_ARMCM23.o ./CMSIS/Testing/cmsis_build/RTE/Device/ARMCM23/startup_ARMCM23.su ./CMSIS/Testing/cmsis_build/RTE/Device/ARMCM23/system_ARMCM23.cyclo ./CMSIS/Testing/cmsis_build/RTE/Device/ARMCM23/system_ARMCM23.d ./CMSIS/Testing/cmsis_build/RTE/Device/ARMCM23/system_ARMCM23.o ./CMSIS/Testing/cmsis_build/RTE/Device/ARMCM23/system_ARMCM23.su

.PHONY: clean-CMSIS-2f-Testing-2f-cmsis_build-2f-RTE-2f-Device-2f-ARMCM23

