################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/Users/josnyder/dev/CMSIS-DSP/Examples/ARM/arm_dotproduct_example/RTE/Device/ARMCM55/startup_ARMCM55.c \
/Users/josnyder/dev/CMSIS-DSP/Examples/ARM/arm_dotproduct_example/RTE/Device/ARMCM55/system_ARMCM55.c 

OBJS += \
./CMSIS/Examples/ARM/arm_dotproduct_example/RTE/Device/ARMCM55/startup_ARMCM55.o \
./CMSIS/Examples/ARM/arm_dotproduct_example/RTE/Device/ARMCM55/system_ARMCM55.o 

C_DEPS += \
./CMSIS/Examples/ARM/arm_dotproduct_example/RTE/Device/ARMCM55/startup_ARMCM55.d \
./CMSIS/Examples/ARM/arm_dotproduct_example/RTE/Device/ARMCM55/system_ARMCM55.d 


# Each subdirectory must supply rules for building sources it contributes
CMSIS/Examples/ARM/arm_dotproduct_example/RTE/Device/ARMCM55/startup_ARMCM55.o: /Users/josnyder/dev/CMSIS-DSP/Examples/ARM/arm_dotproduct_example/RTE/Device/ARMCM55/startup_ARMCM55.c CMSIS/Examples/ARM/arm_dotproduct_example/RTE/Device/ARMCM55/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../Drivers/CMSIS/Include -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Examples/ARM/arm_dotproduct_example/RTE/Device/ARMCM55/system_ARMCM55.o: /Users/josnyder/dev/CMSIS-DSP/Examples/ARM/arm_dotproduct_example/RTE/Device/ARMCM55/system_ARMCM55.c CMSIS/Examples/ARM/arm_dotproduct_example/RTE/Device/ARMCM55/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../Drivers/CMSIS/Include -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-CMSIS-2f-Examples-2f-ARM-2f-arm_dotproduct_example-2f-RTE-2f-Device-2f-ARMCM55

clean-CMSIS-2f-Examples-2f-ARM-2f-arm_dotproduct_example-2f-RTE-2f-Device-2f-ARMCM55:
	-$(RM) ./CMSIS/Examples/ARM/arm_dotproduct_example/RTE/Device/ARMCM55/startup_ARMCM55.cyclo ./CMSIS/Examples/ARM/arm_dotproduct_example/RTE/Device/ARMCM55/startup_ARMCM55.d ./CMSIS/Examples/ARM/arm_dotproduct_example/RTE/Device/ARMCM55/startup_ARMCM55.o ./CMSIS/Examples/ARM/arm_dotproduct_example/RTE/Device/ARMCM55/startup_ARMCM55.su ./CMSIS/Examples/ARM/arm_dotproduct_example/RTE/Device/ARMCM55/system_ARMCM55.cyclo ./CMSIS/Examples/ARM/arm_dotproduct_example/RTE/Device/ARMCM55/system_ARMCM55.d ./CMSIS/Examples/ARM/arm_dotproduct_example/RTE/Device/ARMCM55/system_ARMCM55.o ./CMSIS/Examples/ARM/arm_dotproduct_example/RTE/Device/ARMCM55/system_ARMCM55.su

.PHONY: clean-CMSIS-2f-Examples-2f-ARM-2f-arm_dotproduct_example-2f-RTE-2f-Device-2f-ARMCM55

