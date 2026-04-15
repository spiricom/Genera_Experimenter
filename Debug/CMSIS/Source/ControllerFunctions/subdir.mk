################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/Users/josnyder/dev/CMSIS-DSP/Source/ControllerFunctions/ControllerFunctions.c \
/Users/josnyder/dev/CMSIS-DSP/Source/ControllerFunctions/arm_pid_init_f32.c \
/Users/josnyder/dev/CMSIS-DSP/Source/ControllerFunctions/arm_pid_init_q15.c \
/Users/josnyder/dev/CMSIS-DSP/Source/ControllerFunctions/arm_pid_init_q31.c \
/Users/josnyder/dev/CMSIS-DSP/Source/ControllerFunctions/arm_pid_reset_f32.c \
/Users/josnyder/dev/CMSIS-DSP/Source/ControllerFunctions/arm_pid_reset_q15.c \
/Users/josnyder/dev/CMSIS-DSP/Source/ControllerFunctions/arm_pid_reset_q31.c \
/Users/josnyder/dev/CMSIS-DSP/Source/ControllerFunctions/arm_sin_cos_f32.c \
/Users/josnyder/dev/CMSIS-DSP/Source/ControllerFunctions/arm_sin_cos_q31.c 

OBJS += \
./CMSIS/Source/ControllerFunctions/ControllerFunctions.o \
./CMSIS/Source/ControllerFunctions/arm_pid_init_f32.o \
./CMSIS/Source/ControllerFunctions/arm_pid_init_q15.o \
./CMSIS/Source/ControllerFunctions/arm_pid_init_q31.o \
./CMSIS/Source/ControllerFunctions/arm_pid_reset_f32.o \
./CMSIS/Source/ControllerFunctions/arm_pid_reset_q15.o \
./CMSIS/Source/ControllerFunctions/arm_pid_reset_q31.o \
./CMSIS/Source/ControllerFunctions/arm_sin_cos_f32.o \
./CMSIS/Source/ControllerFunctions/arm_sin_cos_q31.o 

C_DEPS += \
./CMSIS/Source/ControllerFunctions/ControllerFunctions.d \
./CMSIS/Source/ControllerFunctions/arm_pid_init_f32.d \
./CMSIS/Source/ControllerFunctions/arm_pid_init_q15.d \
./CMSIS/Source/ControllerFunctions/arm_pid_init_q31.d \
./CMSIS/Source/ControllerFunctions/arm_pid_reset_f32.d \
./CMSIS/Source/ControllerFunctions/arm_pid_reset_q15.d \
./CMSIS/Source/ControllerFunctions/arm_pid_reset_q31.d \
./CMSIS/Source/ControllerFunctions/arm_sin_cos_f32.d \
./CMSIS/Source/ControllerFunctions/arm_sin_cos_q31.d 


# Each subdirectory must supply rules for building sources it contributes
CMSIS/Source/ControllerFunctions/ControllerFunctions.o: /Users/josnyder/dev/CMSIS-DSP/Source/ControllerFunctions/ControllerFunctions.c CMSIS/Source/ControllerFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/CMSIS-DSP/PrivateInclude" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Source/ControllerFunctions/arm_pid_init_f32.o: /Users/josnyder/dev/CMSIS-DSP/Source/ControllerFunctions/arm_pid_init_f32.c CMSIS/Source/ControllerFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/CMSIS-DSP/PrivateInclude" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Source/ControllerFunctions/arm_pid_init_q15.o: /Users/josnyder/dev/CMSIS-DSP/Source/ControllerFunctions/arm_pid_init_q15.c CMSIS/Source/ControllerFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/CMSIS-DSP/PrivateInclude" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Source/ControllerFunctions/arm_pid_init_q31.o: /Users/josnyder/dev/CMSIS-DSP/Source/ControllerFunctions/arm_pid_init_q31.c CMSIS/Source/ControllerFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/CMSIS-DSP/PrivateInclude" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Source/ControllerFunctions/arm_pid_reset_f32.o: /Users/josnyder/dev/CMSIS-DSP/Source/ControllerFunctions/arm_pid_reset_f32.c CMSIS/Source/ControllerFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/CMSIS-DSP/PrivateInclude" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Source/ControllerFunctions/arm_pid_reset_q15.o: /Users/josnyder/dev/CMSIS-DSP/Source/ControllerFunctions/arm_pid_reset_q15.c CMSIS/Source/ControllerFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/CMSIS-DSP/PrivateInclude" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Source/ControllerFunctions/arm_pid_reset_q31.o: /Users/josnyder/dev/CMSIS-DSP/Source/ControllerFunctions/arm_pid_reset_q31.c CMSIS/Source/ControllerFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/CMSIS-DSP/PrivateInclude" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Source/ControllerFunctions/arm_sin_cos_f32.o: /Users/josnyder/dev/CMSIS-DSP/Source/ControllerFunctions/arm_sin_cos_f32.c CMSIS/Source/ControllerFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/CMSIS-DSP/PrivateInclude" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Source/ControllerFunctions/arm_sin_cos_q31.o: /Users/josnyder/dev/CMSIS-DSP/Source/ControllerFunctions/arm_sin_cos_q31.c CMSIS/Source/ControllerFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/CMSIS-DSP/PrivateInclude" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-CMSIS-2f-Source-2f-ControllerFunctions

clean-CMSIS-2f-Source-2f-ControllerFunctions:
	-$(RM) ./CMSIS/Source/ControllerFunctions/ControllerFunctions.cyclo ./CMSIS/Source/ControllerFunctions/ControllerFunctions.d ./CMSIS/Source/ControllerFunctions/ControllerFunctions.o ./CMSIS/Source/ControllerFunctions/ControllerFunctions.su ./CMSIS/Source/ControllerFunctions/arm_pid_init_f32.cyclo ./CMSIS/Source/ControllerFunctions/arm_pid_init_f32.d ./CMSIS/Source/ControllerFunctions/arm_pid_init_f32.o ./CMSIS/Source/ControllerFunctions/arm_pid_init_f32.su ./CMSIS/Source/ControllerFunctions/arm_pid_init_q15.cyclo ./CMSIS/Source/ControllerFunctions/arm_pid_init_q15.d ./CMSIS/Source/ControllerFunctions/arm_pid_init_q15.o ./CMSIS/Source/ControllerFunctions/arm_pid_init_q15.su ./CMSIS/Source/ControllerFunctions/arm_pid_init_q31.cyclo ./CMSIS/Source/ControllerFunctions/arm_pid_init_q31.d ./CMSIS/Source/ControllerFunctions/arm_pid_init_q31.o ./CMSIS/Source/ControllerFunctions/arm_pid_init_q31.su ./CMSIS/Source/ControllerFunctions/arm_pid_reset_f32.cyclo ./CMSIS/Source/ControllerFunctions/arm_pid_reset_f32.d ./CMSIS/Source/ControllerFunctions/arm_pid_reset_f32.o ./CMSIS/Source/ControllerFunctions/arm_pid_reset_f32.su ./CMSIS/Source/ControllerFunctions/arm_pid_reset_q15.cyclo ./CMSIS/Source/ControllerFunctions/arm_pid_reset_q15.d ./CMSIS/Source/ControllerFunctions/arm_pid_reset_q15.o ./CMSIS/Source/ControllerFunctions/arm_pid_reset_q15.su ./CMSIS/Source/ControllerFunctions/arm_pid_reset_q31.cyclo ./CMSIS/Source/ControllerFunctions/arm_pid_reset_q31.d ./CMSIS/Source/ControllerFunctions/arm_pid_reset_q31.o ./CMSIS/Source/ControllerFunctions/arm_pid_reset_q31.su ./CMSIS/Source/ControllerFunctions/arm_sin_cos_f32.cyclo ./CMSIS/Source/ControllerFunctions/arm_sin_cos_f32.d ./CMSIS/Source/ControllerFunctions/arm_sin_cos_f32.o ./CMSIS/Source/ControllerFunctions/arm_sin_cos_f32.su ./CMSIS/Source/ControllerFunctions/arm_sin_cos_q31.cyclo ./CMSIS/Source/ControllerFunctions/arm_sin_cos_q31.d ./CMSIS/Source/ControllerFunctions/arm_sin_cos_q31.o ./CMSIS/Source/ControllerFunctions/arm_sin_cos_q31.su

.PHONY: clean-CMSIS-2f-Source-2f-ControllerFunctions

