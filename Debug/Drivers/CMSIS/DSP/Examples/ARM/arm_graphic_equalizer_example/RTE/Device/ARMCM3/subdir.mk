################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Drivers/CMSIS/DSP/Examples/ARM/arm_graphic_equalizer_example/RTE/Device/ARMCM3/startup_ARMCM3.s 

C_SRCS += \
../Drivers/CMSIS/DSP/Examples/ARM/arm_graphic_equalizer_example/RTE/Device/ARMCM3/system_ARMCM3.c 

OBJS += \
./Drivers/CMSIS/DSP/Examples/ARM/arm_graphic_equalizer_example/RTE/Device/ARMCM3/startup_ARMCM3.o \
./Drivers/CMSIS/DSP/Examples/ARM/arm_graphic_equalizer_example/RTE/Device/ARMCM3/system_ARMCM3.o 

S_DEPS += \
./Drivers/CMSIS/DSP/Examples/ARM/arm_graphic_equalizer_example/RTE/Device/ARMCM3/startup_ARMCM3.d 

C_DEPS += \
./Drivers/CMSIS/DSP/Examples/ARM/arm_graphic_equalizer_example/RTE/Device/ARMCM3/system_ARMCM3.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/DSP/Examples/ARM/arm_graphic_equalizer_example/RTE/Device/ARMCM3/%.o: ../Drivers/CMSIS/DSP/Examples/ARM/arm_graphic_equalizer_example/RTE/Device/ARMCM3/%.s Drivers/CMSIS/DSP/Examples/ARM/arm_graphic_equalizer_example/RTE/Device/ARMCM3/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m7 -g -DDEBUG -c -I../Drivers/CMSIS/DSP/Include -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"
Drivers/CMSIS/DSP/Examples/ARM/arm_graphic_equalizer_example/RTE/Device/ARMCM3/%.o Drivers/CMSIS/DSP/Examples/ARM/arm_graphic_equalizer_example/RTE/Device/ARMCM3/%.su Drivers/CMSIS/DSP/Examples/ARM/arm_graphic_equalizer_example/RTE/Device/ARMCM3/%.cyclo: ../Drivers/CMSIS/DSP/Examples/ARM/arm_graphic_equalizer_example/RTE/Device/ARMCM3/%.c Drivers/CMSIS/DSP/Examples/ARM/arm_graphic_equalizer_example/RTE/Device/ARMCM3/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DARM_MATH_CM7 -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Drivers/CMSIS/Include -I"/Users/jeffsnyder/dev/LEAF/leaf" -I../Drivers/CMSIS/DSP/Include -I"/Users/jeffsnyder/dev/Electrosteel_embedded/Electrosteel_audio/Drivers/CMSIS/DSP/Include" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I"/Users/jeffsnyder/dev/LEAF/leaf" -Ofast -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS-2f-DSP-2f-Examples-2f-ARM-2f-arm_graphic_equalizer_example-2f-RTE-2f-Device-2f-ARMCM3

clean-Drivers-2f-CMSIS-2f-DSP-2f-Examples-2f-ARM-2f-arm_graphic_equalizer_example-2f-RTE-2f-Device-2f-ARMCM3:
	-$(RM) ./Drivers/CMSIS/DSP/Examples/ARM/arm_graphic_equalizer_example/RTE/Device/ARMCM3/startup_ARMCM3.d ./Drivers/CMSIS/DSP/Examples/ARM/arm_graphic_equalizer_example/RTE/Device/ARMCM3/startup_ARMCM3.o ./Drivers/CMSIS/DSP/Examples/ARM/arm_graphic_equalizer_example/RTE/Device/ARMCM3/system_ARMCM3.cyclo ./Drivers/CMSIS/DSP/Examples/ARM/arm_graphic_equalizer_example/RTE/Device/ARMCM3/system_ARMCM3.d ./Drivers/CMSIS/DSP/Examples/ARM/arm_graphic_equalizer_example/RTE/Device/ARMCM3/system_ARMCM3.o ./Drivers/CMSIS/DSP/Examples/ARM/arm_graphic_equalizer_example/RTE/Device/ARMCM3/system_ARMCM3.su

.PHONY: clean-Drivers-2f-CMSIS-2f-DSP-2f-Examples-2f-ARM-2f-arm_graphic_equalizer_example-2f-RTE-2f-Device-2f-ARMCM3

