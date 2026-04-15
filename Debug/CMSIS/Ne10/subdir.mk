################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/Users/josnyder/dev/CMSIS-DSP/Ne10/CMSIS_NE10_fft_generic_float16.neonintrisic.c \
/Users/josnyder/dev/CMSIS-DSP/Ne10/CMSIS_NE10_fft_generic_float32.neonintrisic.c \
/Users/josnyder/dev/CMSIS-DSP/Ne10/CMSIS_NE10_fft_generic_int32.c \
/Users/josnyder/dev/CMSIS-DSP/Ne10/CMSIS_NE10_fft_generic_int32.neonintrisic.c \
/Users/josnyder/dev/CMSIS-DSP/Ne10/CMSIS_NE10_fft_init.c \
/Users/josnyder/dev/CMSIS-DSP/Ne10/NE10_fft_float16.neonintrinsic.c \
/Users/josnyder/dev/CMSIS-DSP/Ne10/NE10_fft_float32.neonintrinsic.c \
/Users/josnyder/dev/CMSIS-DSP/Ne10/NE10_fft_int16.neonintrinsic.c \
/Users/josnyder/dev/CMSIS-DSP/Ne10/NE10_fft_int32.neonintrinsic.c \
/Users/josnyder/dev/CMSIS-DSP/Ne10/NE10_rfft_float16.neonintrinsic.c \
/Users/josnyder/dev/CMSIS-DSP/Ne10/NE10_rfft_float32.neonintrinsic.c 

OBJS += \
./CMSIS/Ne10/CMSIS_NE10_fft_generic_float16.neonintrisic.o \
./CMSIS/Ne10/CMSIS_NE10_fft_generic_float32.neonintrisic.o \
./CMSIS/Ne10/CMSIS_NE10_fft_generic_int32.o \
./CMSIS/Ne10/CMSIS_NE10_fft_generic_int32.neonintrisic.o \
./CMSIS/Ne10/CMSIS_NE10_fft_init.o \
./CMSIS/Ne10/NE10_fft_float16.neonintrinsic.o \
./CMSIS/Ne10/NE10_fft_float32.neonintrinsic.o \
./CMSIS/Ne10/NE10_fft_int16.neonintrinsic.o \
./CMSIS/Ne10/NE10_fft_int32.neonintrinsic.o \
./CMSIS/Ne10/NE10_rfft_float16.neonintrinsic.o \
./CMSIS/Ne10/NE10_rfft_float32.neonintrinsic.o 

C_DEPS += \
./CMSIS/Ne10/CMSIS_NE10_fft_generic_float16.neonintrisic.d \
./CMSIS/Ne10/CMSIS_NE10_fft_generic_float32.neonintrisic.d \
./CMSIS/Ne10/CMSIS_NE10_fft_generic_int32.d \
./CMSIS/Ne10/CMSIS_NE10_fft_generic_int32.neonintrisic.d \
./CMSIS/Ne10/CMSIS_NE10_fft_init.d \
./CMSIS/Ne10/NE10_fft_float16.neonintrinsic.d \
./CMSIS/Ne10/NE10_fft_float32.neonintrinsic.d \
./CMSIS/Ne10/NE10_fft_int16.neonintrinsic.d \
./CMSIS/Ne10/NE10_fft_int32.neonintrinsic.d \
./CMSIS/Ne10/NE10_rfft_float16.neonintrinsic.d \
./CMSIS/Ne10/NE10_rfft_float32.neonintrinsic.d 


# Each subdirectory must supply rules for building sources it contributes
CMSIS/Ne10/CMSIS_NE10_fft_generic_float16.neonintrisic.o: /Users/josnyder/dev/CMSIS-DSP/Ne10/CMSIS_NE10_fft_generic_float16.neonintrisic.c CMSIS/Ne10/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../Drivers/CMSIS/Include -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Ne10/CMSIS_NE10_fft_generic_float32.neonintrisic.o: /Users/josnyder/dev/CMSIS-DSP/Ne10/CMSIS_NE10_fft_generic_float32.neonintrisic.c CMSIS/Ne10/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../Drivers/CMSIS/Include -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Ne10/CMSIS_NE10_fft_generic_int32.o: /Users/josnyder/dev/CMSIS-DSP/Ne10/CMSIS_NE10_fft_generic_int32.c CMSIS/Ne10/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../Drivers/CMSIS/Include -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Ne10/CMSIS_NE10_fft_generic_int32.neonintrisic.o: /Users/josnyder/dev/CMSIS-DSP/Ne10/CMSIS_NE10_fft_generic_int32.neonintrisic.c CMSIS/Ne10/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../Drivers/CMSIS/Include -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Ne10/CMSIS_NE10_fft_init.o: /Users/josnyder/dev/CMSIS-DSP/Ne10/CMSIS_NE10_fft_init.c CMSIS/Ne10/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../Drivers/CMSIS/Include -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Ne10/NE10_fft_float16.neonintrinsic.o: /Users/josnyder/dev/CMSIS-DSP/Ne10/NE10_fft_float16.neonintrinsic.c CMSIS/Ne10/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../Drivers/CMSIS/Include -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Ne10/NE10_fft_float32.neonintrinsic.o: /Users/josnyder/dev/CMSIS-DSP/Ne10/NE10_fft_float32.neonintrinsic.c CMSIS/Ne10/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../Drivers/CMSIS/Include -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Ne10/NE10_fft_int16.neonintrinsic.o: /Users/josnyder/dev/CMSIS-DSP/Ne10/NE10_fft_int16.neonintrinsic.c CMSIS/Ne10/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../Drivers/CMSIS/Include -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Ne10/NE10_fft_int32.neonintrinsic.o: /Users/josnyder/dev/CMSIS-DSP/Ne10/NE10_fft_int32.neonintrinsic.c CMSIS/Ne10/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../Drivers/CMSIS/Include -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Ne10/NE10_rfft_float16.neonintrinsic.o: /Users/josnyder/dev/CMSIS-DSP/Ne10/NE10_rfft_float16.neonintrinsic.c CMSIS/Ne10/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../Drivers/CMSIS/Include -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Ne10/NE10_rfft_float32.neonintrinsic.o: /Users/josnyder/dev/CMSIS-DSP/Ne10/NE10_rfft_float32.neonintrinsic.c CMSIS/Ne10/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../Drivers/CMSIS/Include -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-CMSIS-2f-Ne10

clean-CMSIS-2f-Ne10:
	-$(RM) ./CMSIS/Ne10/CMSIS_NE10_fft_generic_float16.neonintrisic.cyclo ./CMSIS/Ne10/CMSIS_NE10_fft_generic_float16.neonintrisic.d ./CMSIS/Ne10/CMSIS_NE10_fft_generic_float16.neonintrisic.o ./CMSIS/Ne10/CMSIS_NE10_fft_generic_float16.neonintrisic.su ./CMSIS/Ne10/CMSIS_NE10_fft_generic_float32.neonintrisic.cyclo ./CMSIS/Ne10/CMSIS_NE10_fft_generic_float32.neonintrisic.d ./CMSIS/Ne10/CMSIS_NE10_fft_generic_float32.neonintrisic.o ./CMSIS/Ne10/CMSIS_NE10_fft_generic_float32.neonintrisic.su ./CMSIS/Ne10/CMSIS_NE10_fft_generic_int32.cyclo ./CMSIS/Ne10/CMSIS_NE10_fft_generic_int32.d ./CMSIS/Ne10/CMSIS_NE10_fft_generic_int32.neonintrisic.cyclo ./CMSIS/Ne10/CMSIS_NE10_fft_generic_int32.neonintrisic.d ./CMSIS/Ne10/CMSIS_NE10_fft_generic_int32.neonintrisic.o ./CMSIS/Ne10/CMSIS_NE10_fft_generic_int32.neonintrisic.su ./CMSIS/Ne10/CMSIS_NE10_fft_generic_int32.o ./CMSIS/Ne10/CMSIS_NE10_fft_generic_int32.su ./CMSIS/Ne10/CMSIS_NE10_fft_init.cyclo ./CMSIS/Ne10/CMSIS_NE10_fft_init.d ./CMSIS/Ne10/CMSIS_NE10_fft_init.o ./CMSIS/Ne10/CMSIS_NE10_fft_init.su ./CMSIS/Ne10/NE10_fft_float16.neonintrinsic.cyclo ./CMSIS/Ne10/NE10_fft_float16.neonintrinsic.d ./CMSIS/Ne10/NE10_fft_float16.neonintrinsic.o ./CMSIS/Ne10/NE10_fft_float16.neonintrinsic.su ./CMSIS/Ne10/NE10_fft_float32.neonintrinsic.cyclo ./CMSIS/Ne10/NE10_fft_float32.neonintrinsic.d ./CMSIS/Ne10/NE10_fft_float32.neonintrinsic.o ./CMSIS/Ne10/NE10_fft_float32.neonintrinsic.su ./CMSIS/Ne10/NE10_fft_int16.neonintrinsic.cyclo ./CMSIS/Ne10/NE10_fft_int16.neonintrinsic.d ./CMSIS/Ne10/NE10_fft_int16.neonintrinsic.o ./CMSIS/Ne10/NE10_fft_int16.neonintrinsic.su ./CMSIS/Ne10/NE10_fft_int32.neonintrinsic.cyclo ./CMSIS/Ne10/NE10_fft_int32.neonintrinsic.d ./CMSIS/Ne10/NE10_fft_int32.neonintrinsic.o ./CMSIS/Ne10/NE10_fft_int32.neonintrinsic.su ./CMSIS/Ne10/NE10_rfft_float16.neonintrinsic.cyclo ./CMSIS/Ne10/NE10_rfft_float16.neonintrinsic.d ./CMSIS/Ne10/NE10_rfft_float16.neonintrinsic.o ./CMSIS/Ne10/NE10_rfft_float16.neonintrinsic.su ./CMSIS/Ne10/NE10_rfft_float32.neonintrinsic.cyclo ./CMSIS/Ne10/NE10_rfft_float32.neonintrinsic.d ./CMSIS/Ne10/NE10_rfft_float32.neonintrinsic.o ./CMSIS/Ne10/NE10_rfft_float32.neonintrinsic.su

.PHONY: clean-CMSIS-2f-Ne10

