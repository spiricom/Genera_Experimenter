################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/Users/josnyder/dev/CMSIS-DSP/Source/CommonTables/CommonTables.c \
/Users/josnyder/dev/CMSIS-DSP/Source/CommonTables/CommonTablesF16.c \
/Users/josnyder/dev/CMSIS-DSP/Source/CommonTables/arm_common_tables.c \
/Users/josnyder/dev/CMSIS-DSP/Source/CommonTables/arm_common_tables_f16.c \
/Users/josnyder/dev/CMSIS-DSP/Source/CommonTables/arm_const_structs.c \
/Users/josnyder/dev/CMSIS-DSP/Source/CommonTables/arm_const_structs_f16.c \
/Users/josnyder/dev/CMSIS-DSP/Source/CommonTables/arm_mve_tables.c \
/Users/josnyder/dev/CMSIS-DSP/Source/CommonTables/arm_mve_tables_f16.c \
/Users/josnyder/dev/CMSIS-DSP/Source/CommonTables/arm_neon_tables.c \
/Users/josnyder/dev/CMSIS-DSP/Source/CommonTables/arm_neon_tables_f16.c 

OBJS += \
./CMSIS/Source/CommonTables/CommonTables.o \
./CMSIS/Source/CommonTables/CommonTablesF16.o \
./CMSIS/Source/CommonTables/arm_common_tables.o \
./CMSIS/Source/CommonTables/arm_common_tables_f16.o \
./CMSIS/Source/CommonTables/arm_const_structs.o \
./CMSIS/Source/CommonTables/arm_const_structs_f16.o \
./CMSIS/Source/CommonTables/arm_mve_tables.o \
./CMSIS/Source/CommonTables/arm_mve_tables_f16.o \
./CMSIS/Source/CommonTables/arm_neon_tables.o \
./CMSIS/Source/CommonTables/arm_neon_tables_f16.o 

C_DEPS += \
./CMSIS/Source/CommonTables/CommonTables.d \
./CMSIS/Source/CommonTables/CommonTablesF16.d \
./CMSIS/Source/CommonTables/arm_common_tables.d \
./CMSIS/Source/CommonTables/arm_common_tables_f16.d \
./CMSIS/Source/CommonTables/arm_const_structs.d \
./CMSIS/Source/CommonTables/arm_const_structs_f16.d \
./CMSIS/Source/CommonTables/arm_mve_tables.d \
./CMSIS/Source/CommonTables/arm_mve_tables_f16.d \
./CMSIS/Source/CommonTables/arm_neon_tables.d \
./CMSIS/Source/CommonTables/arm_neon_tables_f16.d 


# Each subdirectory must supply rules for building sources it contributes
CMSIS/Source/CommonTables/CommonTables.o: /Users/josnyder/dev/CMSIS-DSP/Source/CommonTables/CommonTables.c CMSIS/Source/CommonTables/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/CMSIS-DSP/PrivateInclude" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Source/CommonTables/CommonTablesF16.o: /Users/josnyder/dev/CMSIS-DSP/Source/CommonTables/CommonTablesF16.c CMSIS/Source/CommonTables/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/CMSIS-DSP/PrivateInclude" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Source/CommonTables/arm_common_tables.o: /Users/josnyder/dev/CMSIS-DSP/Source/CommonTables/arm_common_tables.c CMSIS/Source/CommonTables/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/CMSIS-DSP/PrivateInclude" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Source/CommonTables/arm_common_tables_f16.o: /Users/josnyder/dev/CMSIS-DSP/Source/CommonTables/arm_common_tables_f16.c CMSIS/Source/CommonTables/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/CMSIS-DSP/PrivateInclude" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Source/CommonTables/arm_const_structs.o: /Users/josnyder/dev/CMSIS-DSP/Source/CommonTables/arm_const_structs.c CMSIS/Source/CommonTables/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/CMSIS-DSP/PrivateInclude" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Source/CommonTables/arm_const_structs_f16.o: /Users/josnyder/dev/CMSIS-DSP/Source/CommonTables/arm_const_structs_f16.c CMSIS/Source/CommonTables/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/CMSIS-DSP/PrivateInclude" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Source/CommonTables/arm_mve_tables.o: /Users/josnyder/dev/CMSIS-DSP/Source/CommonTables/arm_mve_tables.c CMSIS/Source/CommonTables/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/CMSIS-DSP/PrivateInclude" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Source/CommonTables/arm_mve_tables_f16.o: /Users/josnyder/dev/CMSIS-DSP/Source/CommonTables/arm_mve_tables_f16.c CMSIS/Source/CommonTables/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/CMSIS-DSP/PrivateInclude" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Source/CommonTables/arm_neon_tables.o: /Users/josnyder/dev/CMSIS-DSP/Source/CommonTables/arm_neon_tables.c CMSIS/Source/CommonTables/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/CMSIS-DSP/PrivateInclude" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
CMSIS/Source/CommonTables/arm_neon_tables_f16.o: /Users/josnyder/dev/CMSIS-DSP/Source/CommonTables/arm_neon_tables_f16.c CMSIS/Source/CommonTables/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/CMSIS-DSP/Include" -I"/Users/josnyder/dev/CMSIS-DSP/PrivateInclude" -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-CMSIS-2f-Source-2f-CommonTables

clean-CMSIS-2f-Source-2f-CommonTables:
	-$(RM) ./CMSIS/Source/CommonTables/CommonTables.cyclo ./CMSIS/Source/CommonTables/CommonTables.d ./CMSIS/Source/CommonTables/CommonTables.o ./CMSIS/Source/CommonTables/CommonTables.su ./CMSIS/Source/CommonTables/CommonTablesF16.cyclo ./CMSIS/Source/CommonTables/CommonTablesF16.d ./CMSIS/Source/CommonTables/CommonTablesF16.o ./CMSIS/Source/CommonTables/CommonTablesF16.su ./CMSIS/Source/CommonTables/arm_common_tables.cyclo ./CMSIS/Source/CommonTables/arm_common_tables.d ./CMSIS/Source/CommonTables/arm_common_tables.o ./CMSIS/Source/CommonTables/arm_common_tables.su ./CMSIS/Source/CommonTables/arm_common_tables_f16.cyclo ./CMSIS/Source/CommonTables/arm_common_tables_f16.d ./CMSIS/Source/CommonTables/arm_common_tables_f16.o ./CMSIS/Source/CommonTables/arm_common_tables_f16.su ./CMSIS/Source/CommonTables/arm_const_structs.cyclo ./CMSIS/Source/CommonTables/arm_const_structs.d ./CMSIS/Source/CommonTables/arm_const_structs.o ./CMSIS/Source/CommonTables/arm_const_structs.su ./CMSIS/Source/CommonTables/arm_const_structs_f16.cyclo ./CMSIS/Source/CommonTables/arm_const_structs_f16.d ./CMSIS/Source/CommonTables/arm_const_structs_f16.o ./CMSIS/Source/CommonTables/arm_const_structs_f16.su ./CMSIS/Source/CommonTables/arm_mve_tables.cyclo ./CMSIS/Source/CommonTables/arm_mve_tables.d ./CMSIS/Source/CommonTables/arm_mve_tables.o ./CMSIS/Source/CommonTables/arm_mve_tables.su ./CMSIS/Source/CommonTables/arm_mve_tables_f16.cyclo ./CMSIS/Source/CommonTables/arm_mve_tables_f16.d ./CMSIS/Source/CommonTables/arm_mve_tables_f16.o ./CMSIS/Source/CommonTables/arm_mve_tables_f16.su ./CMSIS/Source/CommonTables/arm_neon_tables.cyclo ./CMSIS/Source/CommonTables/arm_neon_tables.d ./CMSIS/Source/CommonTables/arm_neon_tables.o ./CMSIS/Source/CommonTables/arm_neon_tables.su ./CMSIS/Source/CommonTables/arm_neon_tables_f16.cyclo ./CMSIS/Source/CommonTables/arm_neon_tables_f16.d ./CMSIS/Source/CommonTables/arm_neon_tables_f16.o ./CMSIS/Source/CommonTables/arm_neon_tables_f16.su

.PHONY: clean-CMSIS-2f-Source-2f-CommonTables

