################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/Users/jeffsnyder/dev/CMSIS/Source/arm_atan2_f32.c \
/Users/jeffsnyder/dev/CMSIS/Source/arm_common_tables.c \
/Users/jeffsnyder/dev/CMSIS/Source/arm_const_structs.c \
/Users/jeffsnyder/dev/CMSIS/Source/arm_cos_f32.c \
/Users/jeffsnyder/dev/CMSIS/Source/arm_fir_decimate_f32.c \
/Users/jeffsnyder/dev/CMSIS/Source/arm_fir_decimate_init_f32.c \
/Users/jeffsnyder/dev/CMSIS/Source/arm_fir_init_f32.c \
/Users/jeffsnyder/dev/CMSIS/Source/arm_fir_interpolate_f32.c \
/Users/jeffsnyder/dev/CMSIS/Source/arm_fir_interpolate_init_f32.c \
/Users/jeffsnyder/dev/CMSIS/Source/arm_mve_tables.c \
/Users/jeffsnyder/dev/CMSIS/Source/arm_sin_f32.c \
/Users/jeffsnyder/dev/CMSIS/Source/arm_vexp_f32.c \
/Users/jeffsnyder/dev/CMSIS/Source/arm_vlog_f32.c 

OBJS += \
./CMSIS/Source/arm_atan2_f32.o \
./CMSIS/Source/arm_common_tables.o \
./CMSIS/Source/arm_const_structs.o \
./CMSIS/Source/arm_cos_f32.o \
./CMSIS/Source/arm_fir_decimate_f32.o \
./CMSIS/Source/arm_fir_decimate_init_f32.o \
./CMSIS/Source/arm_fir_init_f32.o \
./CMSIS/Source/arm_fir_interpolate_f32.o \
./CMSIS/Source/arm_fir_interpolate_init_f32.o \
./CMSIS/Source/arm_mve_tables.o \
./CMSIS/Source/arm_sin_f32.o \
./CMSIS/Source/arm_vexp_f32.o \
./CMSIS/Source/arm_vlog_f32.o 

C_DEPS += \
./CMSIS/Source/arm_atan2_f32.d \
./CMSIS/Source/arm_common_tables.d \
./CMSIS/Source/arm_const_structs.d \
./CMSIS/Source/arm_cos_f32.d \
./CMSIS/Source/arm_fir_decimate_f32.d \
./CMSIS/Source/arm_fir_decimate_init_f32.d \
./CMSIS/Source/arm_fir_init_f32.d \
./CMSIS/Source/arm_fir_interpolate_f32.d \
./CMSIS/Source/arm_fir_interpolate_init_f32.d \
./CMSIS/Source/arm_mve_tables.d \
./CMSIS/Source/arm_sin_f32.d \
./CMSIS/Source/arm_vexp_f32.d \
./CMSIS/Source/arm_vlog_f32.d 


# Each subdirectory must supply rules for building sources it contributes
CMSIS/Source/arm_atan2_f32.o: /Users/jeffsnyder/dev/CMSIS/Source/arm_atan2_f32.c CMSIS/Source/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DARM_MATH_CM7 -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"/Users/jeffsnyder/dev/CMSIS/Include" -I"/Users/jeffsnyder/dev/CMSIS/PrivateInclude" -I"/Users/jeffsnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I"/Users/jeffsnyder/dev/LEAF/leaf" -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Source/arm_common_tables.o: /Users/jeffsnyder/dev/CMSIS/Source/arm_common_tables.c CMSIS/Source/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DARM_MATH_CM7 -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"/Users/jeffsnyder/dev/CMSIS/Include" -I"/Users/jeffsnyder/dev/CMSIS/PrivateInclude" -I"/Users/jeffsnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I"/Users/jeffsnyder/dev/LEAF/leaf" -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Source/arm_const_structs.o: /Users/jeffsnyder/dev/CMSIS/Source/arm_const_structs.c CMSIS/Source/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DARM_MATH_CM7 -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"/Users/jeffsnyder/dev/CMSIS/Include" -I"/Users/jeffsnyder/dev/CMSIS/PrivateInclude" -I"/Users/jeffsnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I"/Users/jeffsnyder/dev/LEAF/leaf" -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Source/arm_cos_f32.o: /Users/jeffsnyder/dev/CMSIS/Source/arm_cos_f32.c CMSIS/Source/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DARM_MATH_CM7 -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"/Users/jeffsnyder/dev/CMSIS/Include" -I"/Users/jeffsnyder/dev/CMSIS/PrivateInclude" -I"/Users/jeffsnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I"/Users/jeffsnyder/dev/LEAF/leaf" -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Source/arm_fir_decimate_f32.o: /Users/jeffsnyder/dev/CMSIS/Source/arm_fir_decimate_f32.c CMSIS/Source/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DARM_MATH_CM7 -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"/Users/jeffsnyder/dev/CMSIS/Include" -I"/Users/jeffsnyder/dev/CMSIS/PrivateInclude" -I"/Users/jeffsnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I"/Users/jeffsnyder/dev/LEAF/leaf" -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Source/arm_fir_decimate_init_f32.o: /Users/jeffsnyder/dev/CMSIS/Source/arm_fir_decimate_init_f32.c CMSIS/Source/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DARM_MATH_CM7 -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"/Users/jeffsnyder/dev/CMSIS/Include" -I"/Users/jeffsnyder/dev/CMSIS/PrivateInclude" -I"/Users/jeffsnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I"/Users/jeffsnyder/dev/LEAF/leaf" -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Source/arm_fir_init_f32.o: /Users/jeffsnyder/dev/CMSIS/Source/arm_fir_init_f32.c CMSIS/Source/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DARM_MATH_CM7 -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"/Users/jeffsnyder/dev/CMSIS/Include" -I"/Users/jeffsnyder/dev/CMSIS/PrivateInclude" -I"/Users/jeffsnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I"/Users/jeffsnyder/dev/LEAF/leaf" -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Source/arm_fir_interpolate_f32.o: /Users/jeffsnyder/dev/CMSIS/Source/arm_fir_interpolate_f32.c CMSIS/Source/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DARM_MATH_CM7 -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"/Users/jeffsnyder/dev/CMSIS/Include" -I"/Users/jeffsnyder/dev/CMSIS/PrivateInclude" -I"/Users/jeffsnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I"/Users/jeffsnyder/dev/LEAF/leaf" -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Source/arm_fir_interpolate_init_f32.o: /Users/jeffsnyder/dev/CMSIS/Source/arm_fir_interpolate_init_f32.c CMSIS/Source/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DARM_MATH_CM7 -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"/Users/jeffsnyder/dev/CMSIS/Include" -I"/Users/jeffsnyder/dev/CMSIS/PrivateInclude" -I"/Users/jeffsnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I"/Users/jeffsnyder/dev/LEAF/leaf" -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Source/arm_mve_tables.o: /Users/jeffsnyder/dev/CMSIS/Source/arm_mve_tables.c CMSIS/Source/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DARM_MATH_CM7 -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"/Users/jeffsnyder/dev/CMSIS/Include" -I"/Users/jeffsnyder/dev/CMSIS/PrivateInclude" -I"/Users/jeffsnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I"/Users/jeffsnyder/dev/LEAF/leaf" -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Source/arm_sin_f32.o: /Users/jeffsnyder/dev/CMSIS/Source/arm_sin_f32.c CMSIS/Source/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DARM_MATH_CM7 -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"/Users/jeffsnyder/dev/CMSIS/Include" -I"/Users/jeffsnyder/dev/CMSIS/PrivateInclude" -I"/Users/jeffsnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I"/Users/jeffsnyder/dev/LEAF/leaf" -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Source/arm_vexp_f32.o: /Users/jeffsnyder/dev/CMSIS/Source/arm_vexp_f32.c CMSIS/Source/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DARM_MATH_CM7 -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"/Users/jeffsnyder/dev/CMSIS/Include" -I"/Users/jeffsnyder/dev/CMSIS/PrivateInclude" -I"/Users/jeffsnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I"/Users/jeffsnyder/dev/LEAF/leaf" -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Source/arm_vlog_f32.o: /Users/jeffsnyder/dev/CMSIS/Source/arm_vlog_f32.c CMSIS/Source/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DARM_MATH_CM7 -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"/Users/jeffsnyder/dev/CMSIS/Include" -I"/Users/jeffsnyder/dev/CMSIS/PrivateInclude" -I"/Users/jeffsnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I"/Users/jeffsnyder/dev/LEAF/leaf" -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-CMSIS-2f-Source

clean-CMSIS-2f-Source:
	-$(RM) ./CMSIS/Source/arm_atan2_f32.cyclo ./CMSIS/Source/arm_atan2_f32.d ./CMSIS/Source/arm_atan2_f32.o ./CMSIS/Source/arm_atan2_f32.su ./CMSIS/Source/arm_common_tables.cyclo ./CMSIS/Source/arm_common_tables.d ./CMSIS/Source/arm_common_tables.o ./CMSIS/Source/arm_common_tables.su ./CMSIS/Source/arm_const_structs.cyclo ./CMSIS/Source/arm_const_structs.d ./CMSIS/Source/arm_const_structs.o ./CMSIS/Source/arm_const_structs.su ./CMSIS/Source/arm_cos_f32.cyclo ./CMSIS/Source/arm_cos_f32.d ./CMSIS/Source/arm_cos_f32.o ./CMSIS/Source/arm_cos_f32.su ./CMSIS/Source/arm_fir_decimate_f32.cyclo ./CMSIS/Source/arm_fir_decimate_f32.d ./CMSIS/Source/arm_fir_decimate_f32.o ./CMSIS/Source/arm_fir_decimate_f32.su ./CMSIS/Source/arm_fir_decimate_init_f32.cyclo ./CMSIS/Source/arm_fir_decimate_init_f32.d ./CMSIS/Source/arm_fir_decimate_init_f32.o ./CMSIS/Source/arm_fir_decimate_init_f32.su ./CMSIS/Source/arm_fir_init_f32.cyclo ./CMSIS/Source/arm_fir_init_f32.d ./CMSIS/Source/arm_fir_init_f32.o ./CMSIS/Source/arm_fir_init_f32.su ./CMSIS/Source/arm_fir_interpolate_f32.cyclo ./CMSIS/Source/arm_fir_interpolate_f32.d ./CMSIS/Source/arm_fir_interpolate_f32.o ./CMSIS/Source/arm_fir_interpolate_f32.su ./CMSIS/Source/arm_fir_interpolate_init_f32.cyclo ./CMSIS/Source/arm_fir_interpolate_init_f32.d ./CMSIS/Source/arm_fir_interpolate_init_f32.o ./CMSIS/Source/arm_fir_interpolate_init_f32.su ./CMSIS/Source/arm_mve_tables.cyclo ./CMSIS/Source/arm_mve_tables.d ./CMSIS/Source/arm_mve_tables.o ./CMSIS/Source/arm_mve_tables.su ./CMSIS/Source/arm_sin_f32.cyclo ./CMSIS/Source/arm_sin_f32.d ./CMSIS/Source/arm_sin_f32.o ./CMSIS/Source/arm_sin_f32.su ./CMSIS/Source/arm_vexp_f32.cyclo ./CMSIS/Source/arm_vexp_f32.d ./CMSIS/Source/arm_vexp_f32.o ./CMSIS/Source/arm_vexp_f32.su ./CMSIS/Source/arm_vlog_f32.cyclo ./CMSIS/Source/arm_vlog_f32.d ./CMSIS/Source/arm_vlog_f32.o ./CMSIS/Source/arm_vlog_f32.su

.PHONY: clean-CMSIS-2f-Source

