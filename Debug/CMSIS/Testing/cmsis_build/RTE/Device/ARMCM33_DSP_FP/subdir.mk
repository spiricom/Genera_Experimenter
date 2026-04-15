################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/Users/josnyder/dev/CMSIS-DSP/Testing/cmsis_build/RTE/Device/ARMCM33_DSP_FP/startup_ARMCM33.c \
/Users/josnyder/dev/CMSIS-DSP/Testing/cmsis_build/RTE/Device/ARMCM33_DSP_FP/system_ARMCM33.c 

S_UPPER_SRCS += \
/Users/josnyder/dev/CMSIS-DSP/Testing/cmsis_build/RTE/Device/ARMCM33_DSP_FP/startup_ARMCM33.S 

OBJS += \
./CMSIS/Testing/cmsis_build/RTE/Device/ARMCM33_DSP_FP/startup_ARMCM33.o \
./CMSIS/Testing/cmsis_build/RTE/Device/ARMCM33_DSP_FP/system_ARMCM33.o 

S_UPPER_DEPS += \
./CMSIS/Testing/cmsis_build/RTE/Device/ARMCM33_DSP_FP/startup_ARMCM33.d 

C_DEPS += \
./CMSIS/Testing/cmsis_build/RTE/Device/ARMCM33_DSP_FP/startup_ARMCM33.d \
./CMSIS/Testing/cmsis_build/RTE/Device/ARMCM33_DSP_FP/system_ARMCM33.d 


# Each subdirectory must supply rules for building sources it contributes
CMSIS/Testing/cmsis_build/RTE/Device/ARMCM33_DSP_FP/startup_ARMCM33.o: /Users/josnyder/dev/CMSIS-DSP/Testing/cmsis_build/RTE/Device/ARMCM33_DSP_FP/startup_ARMCM33.S CMSIS/Testing/cmsis_build/RTE/Device/ARMCM33_DSP_FP/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m7 -g -DDEBUG -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"
CMSIS/Testing/cmsis_build/RTE/Device/ARMCM33_DSP_FP/startup_ARMCM33.o: /Users/josnyder/dev/CMSIS-DSP/Testing/cmsis_build/RTE/Device/ARMCM33_DSP_FP/startup_ARMCM33.c CMSIS/Testing/cmsis_build/RTE/Device/ARMCM33_DSP_FP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../Drivers/CMSIS/Include -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Testing/cmsis_build/RTE/Device/ARMCM33_DSP_FP/system_ARMCM33.o: /Users/josnyder/dev/CMSIS-DSP/Testing/cmsis_build/RTE/Device/ARMCM33_DSP_FP/system_ARMCM33.c CMSIS/Testing/cmsis_build/RTE/Device/ARMCM33_DSP_FP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../Drivers/CMSIS/Include -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-CMSIS-2f-Testing-2f-cmsis_build-2f-RTE-2f-Device-2f-ARMCM33_DSP_FP

clean-CMSIS-2f-Testing-2f-cmsis_build-2f-RTE-2f-Device-2f-ARMCM33_DSP_FP:
	-$(RM) ./CMSIS/Testing/cmsis_build/RTE/Device/ARMCM33_DSP_FP/startup_ARMCM33.cyclo ./CMSIS/Testing/cmsis_build/RTE/Device/ARMCM33_DSP_FP/startup_ARMCM33.d ./CMSIS/Testing/cmsis_build/RTE/Device/ARMCM33_DSP_FP/startup_ARMCM33.o ./CMSIS/Testing/cmsis_build/RTE/Device/ARMCM33_DSP_FP/startup_ARMCM33.su ./CMSIS/Testing/cmsis_build/RTE/Device/ARMCM33_DSP_FP/system_ARMCM33.cyclo ./CMSIS/Testing/cmsis_build/RTE/Device/ARMCM33_DSP_FP/system_ARMCM33.d ./CMSIS/Testing/cmsis_build/RTE/Device/ARMCM33_DSP_FP/system_ARMCM33.o ./CMSIS/Testing/cmsis_build/RTE/Device/ARMCM33_DSP_FP/system_ARMCM33.su

.PHONY: clean-CMSIS-2f-Testing-2f-cmsis_build-2f-RTE-2f-Device-2f-ARMCM33_DSP_FP

