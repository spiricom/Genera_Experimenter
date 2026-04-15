################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/Users/jeffsnyder/dev/CMSIS/Source/CommonTables/CommonTables.c \
/Users/jeffsnyder/dev/CMSIS/Source/CommonTables/CommonTablesF16.c \
/Users/jeffsnyder/dev/CMSIS/Source/CommonTables/arm_common_tables.c \
/Users/jeffsnyder/dev/CMSIS/Source/CommonTables/arm_common_tables_f16.c \
/Users/jeffsnyder/dev/CMSIS/Source/CommonTables/arm_const_structs.c \
/Users/jeffsnyder/dev/CMSIS/Source/CommonTables/arm_const_structs_f16.c \
/Users/jeffsnyder/dev/CMSIS/Source/CommonTables/arm_mve_tables.c \
/Users/jeffsnyder/dev/CMSIS/Source/CommonTables/arm_mve_tables_f16.c 

OBJS += \
./Drivers/CMSIS/Source/CommonTables/CommonTables.o \
./Drivers/CMSIS/Source/CommonTables/CommonTablesF16.o \
./Drivers/CMSIS/Source/CommonTables/arm_common_tables.o \
./Drivers/CMSIS/Source/CommonTables/arm_common_tables_f16.o \
./Drivers/CMSIS/Source/CommonTables/arm_const_structs.o \
./Drivers/CMSIS/Source/CommonTables/arm_const_structs_f16.o \
./Drivers/CMSIS/Source/CommonTables/arm_mve_tables.o \
./Drivers/CMSIS/Source/CommonTables/arm_mve_tables_f16.o 

C_DEPS += \
./Drivers/CMSIS/Source/CommonTables/CommonTables.d \
./Drivers/CMSIS/Source/CommonTables/CommonTablesF16.d \
./Drivers/CMSIS/Source/CommonTables/arm_common_tables.d \
./Drivers/CMSIS/Source/CommonTables/arm_common_tables_f16.d \
./Drivers/CMSIS/Source/CommonTables/arm_const_structs.d \
./Drivers/CMSIS/Source/CommonTables/arm_const_structs_f16.d \
./Drivers/CMSIS/Source/CommonTables/arm_mve_tables.d \
./Drivers/CMSIS/Source/CommonTables/arm_mve_tables_f16.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/Source/CommonTables/CommonTables.o: /Users/jeffsnyder/dev/CMSIS/Source/CommonTables/CommonTables.c Drivers/CMSIS/Source/CommonTables/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DARM_MATH_CM7 -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"/Users/jeffsnyder/dev/CMSIS/Include" -I"/Users/jeffsnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I"/Users/jeffsnyder/dev/LEAF/leaf" -Ofast -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/CMSIS/Source/CommonTables/CommonTablesF16.o: /Users/jeffsnyder/dev/CMSIS/Source/CommonTables/CommonTablesF16.c Drivers/CMSIS/Source/CommonTables/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DARM_MATH_CM7 -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"/Users/jeffsnyder/dev/CMSIS/Include" -I"/Users/jeffsnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I"/Users/jeffsnyder/dev/LEAF/leaf" -Ofast -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/CMSIS/Source/CommonTables/arm_common_tables.o: /Users/jeffsnyder/dev/CMSIS/Source/CommonTables/arm_common_tables.c Drivers/CMSIS/Source/CommonTables/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DARM_MATH_CM7 -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"/Users/jeffsnyder/dev/CMSIS/Include" -I"/Users/jeffsnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I"/Users/jeffsnyder/dev/LEAF/leaf" -Ofast -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/CMSIS/Source/CommonTables/arm_common_tables_f16.o: /Users/jeffsnyder/dev/CMSIS/Source/CommonTables/arm_common_tables_f16.c Drivers/CMSIS/Source/CommonTables/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DARM_MATH_CM7 -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"/Users/jeffsnyder/dev/CMSIS/Include" -I"/Users/jeffsnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I"/Users/jeffsnyder/dev/LEAF/leaf" -Ofast -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/CMSIS/Source/CommonTables/arm_const_structs.o: /Users/jeffsnyder/dev/CMSIS/Source/CommonTables/arm_const_structs.c Drivers/CMSIS/Source/CommonTables/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DARM_MATH_CM7 -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"/Users/jeffsnyder/dev/CMSIS/Include" -I"/Users/jeffsnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I"/Users/jeffsnyder/dev/LEAF/leaf" -Ofast -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/CMSIS/Source/CommonTables/arm_const_structs_f16.o: /Users/jeffsnyder/dev/CMSIS/Source/CommonTables/arm_const_structs_f16.c Drivers/CMSIS/Source/CommonTables/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DARM_MATH_CM7 -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"/Users/jeffsnyder/dev/CMSIS/Include" -I"/Users/jeffsnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I"/Users/jeffsnyder/dev/LEAF/leaf" -Ofast -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/CMSIS/Source/CommonTables/arm_mve_tables.o: /Users/jeffsnyder/dev/CMSIS/Source/CommonTables/arm_mve_tables.c Drivers/CMSIS/Source/CommonTables/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DARM_MATH_CM7 -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"/Users/jeffsnyder/dev/CMSIS/Include" -I"/Users/jeffsnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I"/Users/jeffsnyder/dev/LEAF/leaf" -Ofast -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/CMSIS/Source/CommonTables/arm_mve_tables_f16.o: /Users/jeffsnyder/dev/CMSIS/Source/CommonTables/arm_mve_tables_f16.c Drivers/CMSIS/Source/CommonTables/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DARM_MATH_CM7 -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"/Users/jeffsnyder/dev/CMSIS/Include" -I"/Users/jeffsnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I"/Users/jeffsnyder/dev/LEAF/leaf" -Ofast -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS-2f-Source-2f-CommonTables

clean-Drivers-2f-CMSIS-2f-Source-2f-CommonTables:
	-$(RM) ./Drivers/CMSIS/Source/CommonTables/CommonTables.cyclo ./Drivers/CMSIS/Source/CommonTables/CommonTables.d ./Drivers/CMSIS/Source/CommonTables/CommonTables.o ./Drivers/CMSIS/Source/CommonTables/CommonTables.su ./Drivers/CMSIS/Source/CommonTables/CommonTablesF16.cyclo ./Drivers/CMSIS/Source/CommonTables/CommonTablesF16.d ./Drivers/CMSIS/Source/CommonTables/CommonTablesF16.o ./Drivers/CMSIS/Source/CommonTables/CommonTablesF16.su ./Drivers/CMSIS/Source/CommonTables/arm_common_tables.cyclo ./Drivers/CMSIS/Source/CommonTables/arm_common_tables.d ./Drivers/CMSIS/Source/CommonTables/arm_common_tables.o ./Drivers/CMSIS/Source/CommonTables/arm_common_tables.su ./Drivers/CMSIS/Source/CommonTables/arm_common_tables_f16.cyclo ./Drivers/CMSIS/Source/CommonTables/arm_common_tables_f16.d ./Drivers/CMSIS/Source/CommonTables/arm_common_tables_f16.o ./Drivers/CMSIS/Source/CommonTables/arm_common_tables_f16.su ./Drivers/CMSIS/Source/CommonTables/arm_const_structs.cyclo ./Drivers/CMSIS/Source/CommonTables/arm_const_structs.d ./Drivers/CMSIS/Source/CommonTables/arm_const_structs.o ./Drivers/CMSIS/Source/CommonTables/arm_const_structs.su ./Drivers/CMSIS/Source/CommonTables/arm_const_structs_f16.cyclo ./Drivers/CMSIS/Source/CommonTables/arm_const_structs_f16.d ./Drivers/CMSIS/Source/CommonTables/arm_const_structs_f16.o ./Drivers/CMSIS/Source/CommonTables/arm_const_structs_f16.su ./Drivers/CMSIS/Source/CommonTables/arm_mve_tables.cyclo ./Drivers/CMSIS/Source/CommonTables/arm_mve_tables.d ./Drivers/CMSIS/Source/CommonTables/arm_mve_tables.o ./Drivers/CMSIS/Source/CommonTables/arm_mve_tables.su ./Drivers/CMSIS/Source/CommonTables/arm_mve_tables_f16.cyclo ./Drivers/CMSIS/Source/CommonTables/arm_mve_tables_f16.d ./Drivers/CMSIS/Source/CommonTables/arm_mve_tables_f16.o ./Drivers/CMSIS/Source/CommonTables/arm_mve_tables_f16.su

.PHONY: clean-Drivers-2f-CMSIS-2f-Source-2f-CommonTables

