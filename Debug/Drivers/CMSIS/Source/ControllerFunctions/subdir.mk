################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/Users/jeffsnyder/dev/CMSIS/Source/ControllerFunctions/ControllerFunctions.c \
/Users/jeffsnyder/dev/CMSIS/Source/ControllerFunctions/arm_pid_init_f32.c \
/Users/jeffsnyder/dev/CMSIS/Source/ControllerFunctions/arm_pid_init_q15.c \
/Users/jeffsnyder/dev/CMSIS/Source/ControllerFunctions/arm_pid_init_q31.c \
/Users/jeffsnyder/dev/CMSIS/Source/ControllerFunctions/arm_pid_reset_f32.c \
/Users/jeffsnyder/dev/CMSIS/Source/ControllerFunctions/arm_pid_reset_q15.c \
/Users/jeffsnyder/dev/CMSIS/Source/ControllerFunctions/arm_pid_reset_q31.c \
/Users/jeffsnyder/dev/CMSIS/Source/ControllerFunctions/arm_sin_cos_f32.c \
/Users/jeffsnyder/dev/CMSIS/Source/ControllerFunctions/arm_sin_cos_q31.c 

OBJS += \
./Drivers/CMSIS/Source/ControllerFunctions/ControllerFunctions.o \
./Drivers/CMSIS/Source/ControllerFunctions/arm_pid_init_f32.o \
./Drivers/CMSIS/Source/ControllerFunctions/arm_pid_init_q15.o \
./Drivers/CMSIS/Source/ControllerFunctions/arm_pid_init_q31.o \
./Drivers/CMSIS/Source/ControllerFunctions/arm_pid_reset_f32.o \
./Drivers/CMSIS/Source/ControllerFunctions/arm_pid_reset_q15.o \
./Drivers/CMSIS/Source/ControllerFunctions/arm_pid_reset_q31.o \
./Drivers/CMSIS/Source/ControllerFunctions/arm_sin_cos_f32.o \
./Drivers/CMSIS/Source/ControllerFunctions/arm_sin_cos_q31.o 

C_DEPS += \
./Drivers/CMSIS/Source/ControllerFunctions/ControllerFunctions.d \
./Drivers/CMSIS/Source/ControllerFunctions/arm_pid_init_f32.d \
./Drivers/CMSIS/Source/ControllerFunctions/arm_pid_init_q15.d \
./Drivers/CMSIS/Source/ControllerFunctions/arm_pid_init_q31.d \
./Drivers/CMSIS/Source/ControllerFunctions/arm_pid_reset_f32.d \
./Drivers/CMSIS/Source/ControllerFunctions/arm_pid_reset_q15.d \
./Drivers/CMSIS/Source/ControllerFunctions/arm_pid_reset_q31.d \
./Drivers/CMSIS/Source/ControllerFunctions/arm_sin_cos_f32.d \
./Drivers/CMSIS/Source/ControllerFunctions/arm_sin_cos_q31.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/Source/ControllerFunctions/ControllerFunctions.o: /Users/jeffsnyder/dev/CMSIS/Source/ControllerFunctions/ControllerFunctions.c Drivers/CMSIS/Source/ControllerFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DARM_MATH_CM7 -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"/Users/jeffsnyder/dev/CMSIS/Include" -I"/Users/jeffsnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I"/Users/jeffsnyder/dev/LEAF/leaf" -Ofast -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/CMSIS/Source/ControllerFunctions/arm_pid_init_f32.o: /Users/jeffsnyder/dev/CMSIS/Source/ControllerFunctions/arm_pid_init_f32.c Drivers/CMSIS/Source/ControllerFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DARM_MATH_CM7 -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"/Users/jeffsnyder/dev/CMSIS/Include" -I"/Users/jeffsnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I"/Users/jeffsnyder/dev/LEAF/leaf" -Ofast -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/CMSIS/Source/ControllerFunctions/arm_pid_init_q15.o: /Users/jeffsnyder/dev/CMSIS/Source/ControllerFunctions/arm_pid_init_q15.c Drivers/CMSIS/Source/ControllerFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DARM_MATH_CM7 -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"/Users/jeffsnyder/dev/CMSIS/Include" -I"/Users/jeffsnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I"/Users/jeffsnyder/dev/LEAF/leaf" -Ofast -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/CMSIS/Source/ControllerFunctions/arm_pid_init_q31.o: /Users/jeffsnyder/dev/CMSIS/Source/ControllerFunctions/arm_pid_init_q31.c Drivers/CMSIS/Source/ControllerFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DARM_MATH_CM7 -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"/Users/jeffsnyder/dev/CMSIS/Include" -I"/Users/jeffsnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I"/Users/jeffsnyder/dev/LEAF/leaf" -Ofast -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/CMSIS/Source/ControllerFunctions/arm_pid_reset_f32.o: /Users/jeffsnyder/dev/CMSIS/Source/ControllerFunctions/arm_pid_reset_f32.c Drivers/CMSIS/Source/ControllerFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DARM_MATH_CM7 -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"/Users/jeffsnyder/dev/CMSIS/Include" -I"/Users/jeffsnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I"/Users/jeffsnyder/dev/LEAF/leaf" -Ofast -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/CMSIS/Source/ControllerFunctions/arm_pid_reset_q15.o: /Users/jeffsnyder/dev/CMSIS/Source/ControllerFunctions/arm_pid_reset_q15.c Drivers/CMSIS/Source/ControllerFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DARM_MATH_CM7 -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"/Users/jeffsnyder/dev/CMSIS/Include" -I"/Users/jeffsnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I"/Users/jeffsnyder/dev/LEAF/leaf" -Ofast -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/CMSIS/Source/ControllerFunctions/arm_pid_reset_q31.o: /Users/jeffsnyder/dev/CMSIS/Source/ControllerFunctions/arm_pid_reset_q31.c Drivers/CMSIS/Source/ControllerFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DARM_MATH_CM7 -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"/Users/jeffsnyder/dev/CMSIS/Include" -I"/Users/jeffsnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I"/Users/jeffsnyder/dev/LEAF/leaf" -Ofast -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/CMSIS/Source/ControllerFunctions/arm_sin_cos_f32.o: /Users/jeffsnyder/dev/CMSIS/Source/ControllerFunctions/arm_sin_cos_f32.c Drivers/CMSIS/Source/ControllerFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DARM_MATH_CM7 -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"/Users/jeffsnyder/dev/CMSIS/Include" -I"/Users/jeffsnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I"/Users/jeffsnyder/dev/LEAF/leaf" -Ofast -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/CMSIS/Source/ControllerFunctions/arm_sin_cos_q31.o: /Users/jeffsnyder/dev/CMSIS/Source/ControllerFunctions/arm_sin_cos_q31.c Drivers/CMSIS/Source/ControllerFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DARM_MATH_CM7 -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"/Users/jeffsnyder/dev/CMSIS/Include" -I"/Users/jeffsnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I"/Users/jeffsnyder/dev/LEAF/leaf" -Ofast -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS-2f-Source-2f-ControllerFunctions

clean-Drivers-2f-CMSIS-2f-Source-2f-ControllerFunctions:
	-$(RM) ./Drivers/CMSIS/Source/ControllerFunctions/ControllerFunctions.cyclo ./Drivers/CMSIS/Source/ControllerFunctions/ControllerFunctions.d ./Drivers/CMSIS/Source/ControllerFunctions/ControllerFunctions.o ./Drivers/CMSIS/Source/ControllerFunctions/ControllerFunctions.su ./Drivers/CMSIS/Source/ControllerFunctions/arm_pid_init_f32.cyclo ./Drivers/CMSIS/Source/ControllerFunctions/arm_pid_init_f32.d ./Drivers/CMSIS/Source/ControllerFunctions/arm_pid_init_f32.o ./Drivers/CMSIS/Source/ControllerFunctions/arm_pid_init_f32.su ./Drivers/CMSIS/Source/ControllerFunctions/arm_pid_init_q15.cyclo ./Drivers/CMSIS/Source/ControllerFunctions/arm_pid_init_q15.d ./Drivers/CMSIS/Source/ControllerFunctions/arm_pid_init_q15.o ./Drivers/CMSIS/Source/ControllerFunctions/arm_pid_init_q15.su ./Drivers/CMSIS/Source/ControllerFunctions/arm_pid_init_q31.cyclo ./Drivers/CMSIS/Source/ControllerFunctions/arm_pid_init_q31.d ./Drivers/CMSIS/Source/ControllerFunctions/arm_pid_init_q31.o ./Drivers/CMSIS/Source/ControllerFunctions/arm_pid_init_q31.su ./Drivers/CMSIS/Source/ControllerFunctions/arm_pid_reset_f32.cyclo ./Drivers/CMSIS/Source/ControllerFunctions/arm_pid_reset_f32.d ./Drivers/CMSIS/Source/ControllerFunctions/arm_pid_reset_f32.o ./Drivers/CMSIS/Source/ControllerFunctions/arm_pid_reset_f32.su ./Drivers/CMSIS/Source/ControllerFunctions/arm_pid_reset_q15.cyclo ./Drivers/CMSIS/Source/ControllerFunctions/arm_pid_reset_q15.d ./Drivers/CMSIS/Source/ControllerFunctions/arm_pid_reset_q15.o ./Drivers/CMSIS/Source/ControllerFunctions/arm_pid_reset_q15.su ./Drivers/CMSIS/Source/ControllerFunctions/arm_pid_reset_q31.cyclo ./Drivers/CMSIS/Source/ControllerFunctions/arm_pid_reset_q31.d ./Drivers/CMSIS/Source/ControllerFunctions/arm_pid_reset_q31.o ./Drivers/CMSIS/Source/ControllerFunctions/arm_pid_reset_q31.su ./Drivers/CMSIS/Source/ControllerFunctions/arm_sin_cos_f32.cyclo ./Drivers/CMSIS/Source/ControllerFunctions/arm_sin_cos_f32.d ./Drivers/CMSIS/Source/ControllerFunctions/arm_sin_cos_f32.o ./Drivers/CMSIS/Source/ControllerFunctions/arm_sin_cos_f32.su ./Drivers/CMSIS/Source/ControllerFunctions/arm_sin_cos_q31.cyclo ./Drivers/CMSIS/Source/ControllerFunctions/arm_sin_cos_q31.d ./Drivers/CMSIS/Source/ControllerFunctions/arm_sin_cos_q31.o ./Drivers/CMSIS/Source/ControllerFunctions/arm_sin_cos_q31.su

.PHONY: clean-Drivers-2f-CMSIS-2f-Source-2f-ControllerFunctions

