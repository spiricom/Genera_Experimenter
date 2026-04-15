################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/Users/josnyder/dev/CMSIS-DSP/Testing/patterndata.c 

OBJS += \
./CMSIS/Testing/patterndata.o 

C_DEPS += \
./CMSIS/Testing/patterndata.d 


# Each subdirectory must supply rules for building sources it contributes
CMSIS/Testing/patterndata.o: /Users/josnyder/dev/CMSIS-DSP/Testing/patterndata.c CMSIS/Testing/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../Drivers/CMSIS/Include -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-CMSIS-2f-Testing

clean-CMSIS-2f-Testing:
	-$(RM) ./CMSIS/Testing/patterndata.cyclo ./CMSIS/Testing/patterndata.d ./CMSIS/Testing/patterndata.o ./CMSIS/Testing/patterndata.su

.PHONY: clean-CMSIS-2f-Testing

