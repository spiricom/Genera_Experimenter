################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/Users/josnyder/dev/CMSIS-DSP/Source/QuaternionMathFunctions/QuaternionMathFunctions.c \
/Users/josnyder/dev/CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion2rotation_f32.c \
/Users/josnyder/dev/CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_conjugate_f32.c \
/Users/josnyder/dev/CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_inverse_f32.c \
/Users/josnyder/dev/CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_norm_f32.c \
/Users/josnyder/dev/CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_normalize_f32.c \
/Users/josnyder/dev/CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_product_f32.c \
/Users/josnyder/dev/CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_product_single_f32.c \
/Users/josnyder/dev/CMSIS-DSP/Source/QuaternionMathFunctions/arm_rotation2quaternion_f32.c 

OBJS += \
./CMSIS/Source/QuaternionMathFunctions/QuaternionMathFunctions.o \
./CMSIS/Source/QuaternionMathFunctions/arm_quaternion2rotation_f32.o \
./CMSIS/Source/QuaternionMathFunctions/arm_quaternion_conjugate_f32.o \
./CMSIS/Source/QuaternionMathFunctions/arm_quaternion_inverse_f32.o \
./CMSIS/Source/QuaternionMathFunctions/arm_quaternion_norm_f32.o \
./CMSIS/Source/QuaternionMathFunctions/arm_quaternion_normalize_f32.o \
./CMSIS/Source/QuaternionMathFunctions/arm_quaternion_product_f32.o \
./CMSIS/Source/QuaternionMathFunctions/arm_quaternion_product_single_f32.o \
./CMSIS/Source/QuaternionMathFunctions/arm_rotation2quaternion_f32.o 

C_DEPS += \
./CMSIS/Source/QuaternionMathFunctions/QuaternionMathFunctions.d \
./CMSIS/Source/QuaternionMathFunctions/arm_quaternion2rotation_f32.d \
./CMSIS/Source/QuaternionMathFunctions/arm_quaternion_conjugate_f32.d \
./CMSIS/Source/QuaternionMathFunctions/arm_quaternion_inverse_f32.d \
./CMSIS/Source/QuaternionMathFunctions/arm_quaternion_norm_f32.d \
./CMSIS/Source/QuaternionMathFunctions/arm_quaternion_normalize_f32.d \
./CMSIS/Source/QuaternionMathFunctions/arm_quaternion_product_f32.d \
./CMSIS/Source/QuaternionMathFunctions/arm_quaternion_product_single_f32.d \
./CMSIS/Source/QuaternionMathFunctions/arm_rotation2quaternion_f32.d 


# Each subdirectory must supply rules for building sources it contributes
CMSIS/Source/QuaternionMathFunctions/QuaternionMathFunctions.o: /Users/josnyder/dev/CMSIS-DSP/Source/QuaternionMathFunctions/QuaternionMathFunctions.c CMSIS/Source/QuaternionMathFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/CMSIS-DSP/PrivateInclude" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Source/QuaternionMathFunctions/arm_quaternion2rotation_f32.o: /Users/josnyder/dev/CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion2rotation_f32.c CMSIS/Source/QuaternionMathFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/CMSIS-DSP/PrivateInclude" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Source/QuaternionMathFunctions/arm_quaternion_conjugate_f32.o: /Users/josnyder/dev/CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_conjugate_f32.c CMSIS/Source/QuaternionMathFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/CMSIS-DSP/PrivateInclude" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Source/QuaternionMathFunctions/arm_quaternion_inverse_f32.o: /Users/josnyder/dev/CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_inverse_f32.c CMSIS/Source/QuaternionMathFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/CMSIS-DSP/PrivateInclude" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Source/QuaternionMathFunctions/arm_quaternion_norm_f32.o: /Users/josnyder/dev/CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_norm_f32.c CMSIS/Source/QuaternionMathFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/CMSIS-DSP/PrivateInclude" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Source/QuaternionMathFunctions/arm_quaternion_normalize_f32.o: /Users/josnyder/dev/CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_normalize_f32.c CMSIS/Source/QuaternionMathFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/CMSIS-DSP/PrivateInclude" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Source/QuaternionMathFunctions/arm_quaternion_product_f32.o: /Users/josnyder/dev/CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_product_f32.c CMSIS/Source/QuaternionMathFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/CMSIS-DSP/PrivateInclude" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Source/QuaternionMathFunctions/arm_quaternion_product_single_f32.o: /Users/josnyder/dev/CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_product_single_f32.c CMSIS/Source/QuaternionMathFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/CMSIS-DSP/PrivateInclude" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Source/QuaternionMathFunctions/arm_rotation2quaternion_f32.o: /Users/josnyder/dev/CMSIS-DSP/Source/QuaternionMathFunctions/arm_rotation2quaternion_f32.c CMSIS/Source/QuaternionMathFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/CMSIS-DSP/PrivateInclude" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-CMSIS-2f-Source-2f-QuaternionMathFunctions

clean-CMSIS-2f-Source-2f-QuaternionMathFunctions:
	-$(RM) ./CMSIS/Source/QuaternionMathFunctions/QuaternionMathFunctions.cyclo ./CMSIS/Source/QuaternionMathFunctions/QuaternionMathFunctions.d ./CMSIS/Source/QuaternionMathFunctions/QuaternionMathFunctions.o ./CMSIS/Source/QuaternionMathFunctions/QuaternionMathFunctions.su ./CMSIS/Source/QuaternionMathFunctions/arm_quaternion2rotation_f32.cyclo ./CMSIS/Source/QuaternionMathFunctions/arm_quaternion2rotation_f32.d ./CMSIS/Source/QuaternionMathFunctions/arm_quaternion2rotation_f32.o ./CMSIS/Source/QuaternionMathFunctions/arm_quaternion2rotation_f32.su ./CMSIS/Source/QuaternionMathFunctions/arm_quaternion_conjugate_f32.cyclo ./CMSIS/Source/QuaternionMathFunctions/arm_quaternion_conjugate_f32.d ./CMSIS/Source/QuaternionMathFunctions/arm_quaternion_conjugate_f32.o ./CMSIS/Source/QuaternionMathFunctions/arm_quaternion_conjugate_f32.su ./CMSIS/Source/QuaternionMathFunctions/arm_quaternion_inverse_f32.cyclo ./CMSIS/Source/QuaternionMathFunctions/arm_quaternion_inverse_f32.d ./CMSIS/Source/QuaternionMathFunctions/arm_quaternion_inverse_f32.o ./CMSIS/Source/QuaternionMathFunctions/arm_quaternion_inverse_f32.su ./CMSIS/Source/QuaternionMathFunctions/arm_quaternion_norm_f32.cyclo ./CMSIS/Source/QuaternionMathFunctions/arm_quaternion_norm_f32.d ./CMSIS/Source/QuaternionMathFunctions/arm_quaternion_norm_f32.o ./CMSIS/Source/QuaternionMathFunctions/arm_quaternion_norm_f32.su ./CMSIS/Source/QuaternionMathFunctions/arm_quaternion_normalize_f32.cyclo ./CMSIS/Source/QuaternionMathFunctions/arm_quaternion_normalize_f32.d ./CMSIS/Source/QuaternionMathFunctions/arm_quaternion_normalize_f32.o ./CMSIS/Source/QuaternionMathFunctions/arm_quaternion_normalize_f32.su ./CMSIS/Source/QuaternionMathFunctions/arm_quaternion_product_f32.cyclo ./CMSIS/Source/QuaternionMathFunctions/arm_quaternion_product_f32.d ./CMSIS/Source/QuaternionMathFunctions/arm_quaternion_product_f32.o ./CMSIS/Source/QuaternionMathFunctions/arm_quaternion_product_f32.su ./CMSIS/Source/QuaternionMathFunctions/arm_quaternion_product_single_f32.cyclo ./CMSIS/Source/QuaternionMathFunctions/arm_quaternion_product_single_f32.d ./CMSIS/Source/QuaternionMathFunctions/arm_quaternion_product_single_f32.o ./CMSIS/Source/QuaternionMathFunctions/arm_quaternion_product_single_f32.su ./CMSIS/Source/QuaternionMathFunctions/arm_rotation2quaternion_f32.cyclo ./CMSIS/Source/QuaternionMathFunctions/arm_rotation2quaternion_f32.d ./CMSIS/Source/QuaternionMathFunctions/arm_rotation2quaternion_f32.o ./CMSIS/Source/QuaternionMathFunctions/arm_rotation2quaternion_f32.su

.PHONY: clean-CMSIS-2f-Source-2f-QuaternionMathFunctions

