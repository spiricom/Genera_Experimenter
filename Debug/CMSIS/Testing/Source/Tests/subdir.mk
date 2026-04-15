################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/Users/josnyder/dev/CMSIS-DSP/Testing/Source/Tests/mfccdata.c \
/Users/josnyder/dev/CMSIS-DSP/Testing/Source/Tests/mfccdata_f16.c 

OBJS += \
./CMSIS/Testing/Source/Tests/mfccdata.o \
./CMSIS/Testing/Source/Tests/mfccdata_f16.o 

C_DEPS += \
./CMSIS/Testing/Source/Tests/mfccdata.d \
./CMSIS/Testing/Source/Tests/mfccdata_f16.d 


# Each subdirectory must supply rules for building sources it contributes
CMSIS/Testing/Source/Tests/mfccdata.o: /Users/josnyder/dev/CMSIS-DSP/Testing/Source/Tests/mfccdata.c CMSIS/Testing/Source/Tests/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../Drivers/CMSIS/Include -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Testing/Source/Tests/mfccdata_f16.o: /Users/josnyder/dev/CMSIS-DSP/Testing/Source/Tests/mfccdata_f16.c CMSIS/Testing/Source/Tests/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../Drivers/CMSIS/Include -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-CMSIS-2f-Testing-2f-Source-2f-Tests

clean-CMSIS-2f-Testing-2f-Source-2f-Tests:
	-$(RM) ./CMSIS/Testing/Source/Tests/mfccdata.cyclo ./CMSIS/Testing/Source/Tests/mfccdata.d ./CMSIS/Testing/Source/Tests/mfccdata.o ./CMSIS/Testing/Source/Tests/mfccdata.su ./CMSIS/Testing/Source/Tests/mfccdata_f16.cyclo ./CMSIS/Testing/Source/Tests/mfccdata_f16.d ./CMSIS/Testing/Source/Tests/mfccdata_f16.o ./CMSIS/Testing/Source/Tests/mfccdata_f16.su

.PHONY: clean-CMSIS-2f-Testing-2f-Source-2f-Tests

