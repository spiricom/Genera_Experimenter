################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/Users/josnyder/dev/CMSIS-DSP/Scripts/mfcctemplates/mfccdata.c 

OBJS += \
./CMSIS/Scripts/mfcctemplates/mfccdata.o 

C_DEPS += \
./CMSIS/Scripts/mfcctemplates/mfccdata.d 


# Each subdirectory must supply rules for building sources it contributes
CMSIS/Scripts/mfcctemplates/mfccdata.o: /Users/josnyder/dev/CMSIS-DSP/Scripts/mfcctemplates/mfccdata.c CMSIS/Scripts/mfcctemplates/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../Drivers/CMSIS/Include -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-CMSIS-2f-Scripts-2f-mfcctemplates

clean-CMSIS-2f-Scripts-2f-mfcctemplates:
	-$(RM) ./CMSIS/Scripts/mfcctemplates/mfccdata.cyclo ./CMSIS/Scripts/mfcctemplates/mfccdata.d ./CMSIS/Scripts/mfcctemplates/mfccdata.o ./CMSIS/Scripts/mfcctemplates/mfccdata.su

.PHONY: clean-CMSIS-2f-Scripts-2f-mfcctemplates

