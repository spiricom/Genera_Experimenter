################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Drivers/CMSIS/DSP/Examples/ARM/arm_sin_cos_example/RTE/Device/ARMCM7_SP/startup_ARMCM7.s 

C_SRCS += \
../Drivers/CMSIS/DSP/Examples/ARM/arm_sin_cos_example/RTE/Device/ARMCM7_SP/system_ARMCM7.c 

OBJS += \
./Drivers/CMSIS/DSP/Examples/ARM/arm_sin_cos_example/RTE/Device/ARMCM7_SP/startup_ARMCM7.o \
./Drivers/CMSIS/DSP/Examples/ARM/arm_sin_cos_example/RTE/Device/ARMCM7_SP/system_ARMCM7.o 

S_DEPS += \
./Drivers/CMSIS/DSP/Examples/ARM/arm_sin_cos_example/RTE/Device/ARMCM7_SP/startup_ARMCM7.d 

C_DEPS += \
./Drivers/CMSIS/DSP/Examples/ARM/arm_sin_cos_example/RTE/Device/ARMCM7_SP/system_ARMCM7.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/DSP/Examples/ARM/arm_sin_cos_example/RTE/Device/ARMCM7_SP/%.o: ../Drivers/CMSIS/DSP/Examples/ARM/arm_sin_cos_example/RTE/Device/ARMCM7_SP/%.s Drivers/CMSIS/DSP/Examples/ARM/arm_sin_cos_example/RTE/Device/ARMCM7_SP/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m7 -g -DDEBUG -c -I../Drivers/CMSIS/DSP/Include -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"
Drivers/CMSIS/DSP/Examples/ARM/arm_sin_cos_example/RTE/Device/ARMCM7_SP/%.o Drivers/CMSIS/DSP/Examples/ARM/arm_sin_cos_example/RTE/Device/ARMCM7_SP/%.su Drivers/CMSIS/DSP/Examples/ARM/arm_sin_cos_example/RTE/Device/ARMCM7_SP/%.cyclo: ../Drivers/CMSIS/DSP/Examples/ARM/arm_sin_cos_example/RTE/Device/ARMCM7_SP/%.c Drivers/CMSIS/DSP/Examples/ARM/arm_sin_cos_example/RTE/Device/ARMCM7_SP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DARM_MATH_CM7 -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Drivers/CMSIS/Include -I"/Users/jeffsnyder/dev/LEAF/leaf" -I../Drivers/CMSIS/DSP/Include -I"/Users/jeffsnyder/dev/Electrosteel_embedded/Electrosteel_audio/Drivers/CMSIS/DSP/Include" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I"/Users/jeffsnyder/dev/LEAF/leaf" -Ofast -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS-2f-DSP-2f-Examples-2f-ARM-2f-arm_sin_cos_example-2f-RTE-2f-Device-2f-ARMCM7_SP

clean-Drivers-2f-CMSIS-2f-DSP-2f-Examples-2f-ARM-2f-arm_sin_cos_example-2f-RTE-2f-Device-2f-ARMCM7_SP:
	-$(RM) ./Drivers/CMSIS/DSP/Examples/ARM/arm_sin_cos_example/RTE/Device/ARMCM7_SP/startup_ARMCM7.d ./Drivers/CMSIS/DSP/Examples/ARM/arm_sin_cos_example/RTE/Device/ARMCM7_SP/startup_ARMCM7.o ./Drivers/CMSIS/DSP/Examples/ARM/arm_sin_cos_example/RTE/Device/ARMCM7_SP/system_ARMCM7.cyclo ./Drivers/CMSIS/DSP/Examples/ARM/arm_sin_cos_example/RTE/Device/ARMCM7_SP/system_ARMCM7.d ./Drivers/CMSIS/DSP/Examples/ARM/arm_sin_cos_example/RTE/Device/ARMCM7_SP/system_ARMCM7.o ./Drivers/CMSIS/DSP/Examples/ARM/arm_sin_cos_example/RTE/Device/ARMCM7_SP/system_ARMCM7.su

.PHONY: clean-Drivers-2f-CMSIS-2f-DSP-2f-Examples-2f-ARM-2f-arm_sin_cos_example-2f-RTE-2f-Device-2f-ARMCM7_SP

