################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/Users/josnyder/dev/CMSIS-DSP/Examples/ARM/arm_matrix_example/arm_matrix_example_f32.c \
/Users/josnyder/dev/CMSIS-DSP/Examples/ARM/arm_matrix_example/math_helper.c 

OBJS += \
./CMSIS/Examples/ARM/arm_matrix_example/arm_matrix_example_f32.o \
./CMSIS/Examples/ARM/arm_matrix_example/math_helper.o 

C_DEPS += \
./CMSIS/Examples/ARM/arm_matrix_example/arm_matrix_example_f32.d \
./CMSIS/Examples/ARM/arm_matrix_example/math_helper.d 


# Each subdirectory must supply rules for building sources it contributes
CMSIS/Examples/ARM/arm_matrix_example/arm_matrix_example_f32.o: /Users/josnyder/dev/CMSIS-DSP/Examples/ARM/arm_matrix_example/arm_matrix_example_f32.c CMSIS/Examples/ARM/arm_matrix_example/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../Drivers/CMSIS/Include -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Examples/ARM/arm_matrix_example/math_helper.o: /Users/josnyder/dev/CMSIS-DSP/Examples/ARM/arm_matrix_example/math_helper.c CMSIS/Examples/ARM/arm_matrix_example/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../Drivers/CMSIS/Include -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-CMSIS-2f-Examples-2f-ARM-2f-arm_matrix_example

clean-CMSIS-2f-Examples-2f-ARM-2f-arm_matrix_example:
	-$(RM) ./CMSIS/Examples/ARM/arm_matrix_example/arm_matrix_example_f32.cyclo ./CMSIS/Examples/ARM/arm_matrix_example/arm_matrix_example_f32.d ./CMSIS/Examples/ARM/arm_matrix_example/arm_matrix_example_f32.o ./CMSIS/Examples/ARM/arm_matrix_example/arm_matrix_example_f32.su ./CMSIS/Examples/ARM/arm_matrix_example/math_helper.cyclo ./CMSIS/Examples/ARM/arm_matrix_example/math_helper.d ./CMSIS/Examples/ARM/arm_matrix_example/math_helper.o ./CMSIS/Examples/ARM/arm_matrix_example/math_helper.su

.PHONY: clean-CMSIS-2f-Examples-2f-ARM-2f-arm_matrix_example

