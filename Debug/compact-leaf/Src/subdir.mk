################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../compact-leaf/Src/leaf-analysis.c \
../compact-leaf/Src/leaf-delay.c \
../compact-leaf/Src/leaf-distortion.c \
../compact-leaf/Src/leaf-dynamics.c \
../compact-leaf/Src/leaf-effects.c \
../compact-leaf/Src/leaf-electrical.c \
../compact-leaf/Src/leaf-envelopes.c \
../compact-leaf/Src/leaf-filters.c \
../compact-leaf/Src/leaf-instruments.c \
../compact-leaf/Src/leaf-math.c \
../compact-leaf/Src/leaf-mempool.c \
../compact-leaf/Src/leaf-midi.c \
../compact-leaf/Src/leaf-oscillators.c \
../compact-leaf/Src/leaf-physical.c \
../compact-leaf/Src/leaf-reverb.c \
../compact-leaf/Src/leaf-sampling.c \
../compact-leaf/Src/leaf-tables.c \
../compact-leaf/Src/leaf-vocal.c \
../compact-leaf/Src/leaf.c 

OBJS += \
./compact-leaf/Src/leaf-analysis.o \
./compact-leaf/Src/leaf-delay.o \
./compact-leaf/Src/leaf-distortion.o \
./compact-leaf/Src/leaf-dynamics.o \
./compact-leaf/Src/leaf-effects.o \
./compact-leaf/Src/leaf-electrical.o \
./compact-leaf/Src/leaf-envelopes.o \
./compact-leaf/Src/leaf-filters.o \
./compact-leaf/Src/leaf-instruments.o \
./compact-leaf/Src/leaf-math.o \
./compact-leaf/Src/leaf-mempool.o \
./compact-leaf/Src/leaf-midi.o \
./compact-leaf/Src/leaf-oscillators.o \
./compact-leaf/Src/leaf-physical.o \
./compact-leaf/Src/leaf-reverb.o \
./compact-leaf/Src/leaf-sampling.o \
./compact-leaf/Src/leaf-tables.o \
./compact-leaf/Src/leaf-vocal.o \
./compact-leaf/Src/leaf.o 

C_DEPS += \
./compact-leaf/Src/leaf-analysis.d \
./compact-leaf/Src/leaf-delay.d \
./compact-leaf/Src/leaf-distortion.d \
./compact-leaf/Src/leaf-dynamics.d \
./compact-leaf/Src/leaf-effects.d \
./compact-leaf/Src/leaf-electrical.d \
./compact-leaf/Src/leaf-envelopes.d \
./compact-leaf/Src/leaf-filters.d \
./compact-leaf/Src/leaf-instruments.d \
./compact-leaf/Src/leaf-math.d \
./compact-leaf/Src/leaf-mempool.d \
./compact-leaf/Src/leaf-midi.d \
./compact-leaf/Src/leaf-oscillators.d \
./compact-leaf/Src/leaf-physical.d \
./compact-leaf/Src/leaf-reverb.d \
./compact-leaf/Src/leaf-sampling.d \
./compact-leaf/Src/leaf-tables.d \
./compact-leaf/Src/leaf-vocal.d \
./compact-leaf/Src/leaf.d 


# Each subdirectory must supply rules for building sources it contributes
compact-leaf/Src/%.o compact-leaf/Src/%.su compact-leaf/Src/%.cyclo: ../compact-leaf/Src/%.c compact-leaf/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Inc -I"C:/Users/mattm/ChucCKS/Lab/JUCE stuff/Embedded stuff/Genera_Experimenter/compact-leaf" -I"C:/Users/mattm/ChucCKS/Lab/JUCE stuff/Embedded stuff/Genera_Experimenter/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/AUDIO/Inc -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -Og -ffunction-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-compact-2d-leaf-2f-Src

clean-compact-2d-leaf-2f-Src:
	-$(RM) ./compact-leaf/Src/leaf-analysis.cyclo ./compact-leaf/Src/leaf-analysis.d ./compact-leaf/Src/leaf-analysis.o ./compact-leaf/Src/leaf-analysis.su ./compact-leaf/Src/leaf-delay.cyclo ./compact-leaf/Src/leaf-delay.d ./compact-leaf/Src/leaf-delay.o ./compact-leaf/Src/leaf-delay.su ./compact-leaf/Src/leaf-distortion.cyclo ./compact-leaf/Src/leaf-distortion.d ./compact-leaf/Src/leaf-distortion.o ./compact-leaf/Src/leaf-distortion.su ./compact-leaf/Src/leaf-dynamics.cyclo ./compact-leaf/Src/leaf-dynamics.d ./compact-leaf/Src/leaf-dynamics.o ./compact-leaf/Src/leaf-dynamics.su ./compact-leaf/Src/leaf-effects.cyclo ./compact-leaf/Src/leaf-effects.d ./compact-leaf/Src/leaf-effects.o ./compact-leaf/Src/leaf-effects.su ./compact-leaf/Src/leaf-electrical.cyclo ./compact-leaf/Src/leaf-electrical.d ./compact-leaf/Src/leaf-electrical.o ./compact-leaf/Src/leaf-electrical.su ./compact-leaf/Src/leaf-envelopes.cyclo ./compact-leaf/Src/leaf-envelopes.d ./compact-leaf/Src/leaf-envelopes.o ./compact-leaf/Src/leaf-envelopes.su ./compact-leaf/Src/leaf-filters.cyclo ./compact-leaf/Src/leaf-filters.d ./compact-leaf/Src/leaf-filters.o ./compact-leaf/Src/leaf-filters.su ./compact-leaf/Src/leaf-instruments.cyclo ./compact-leaf/Src/leaf-instruments.d ./compact-leaf/Src/leaf-instruments.o ./compact-leaf/Src/leaf-instruments.su ./compact-leaf/Src/leaf-math.cyclo ./compact-leaf/Src/leaf-math.d ./compact-leaf/Src/leaf-math.o ./compact-leaf/Src/leaf-math.su ./compact-leaf/Src/leaf-mempool.cyclo ./compact-leaf/Src/leaf-mempool.d ./compact-leaf/Src/leaf-mempool.o ./compact-leaf/Src/leaf-mempool.su ./compact-leaf/Src/leaf-midi.cyclo ./compact-leaf/Src/leaf-midi.d ./compact-leaf/Src/leaf-midi.o ./compact-leaf/Src/leaf-midi.su ./compact-leaf/Src/leaf-oscillators.cyclo ./compact-leaf/Src/leaf-oscillators.d ./compact-leaf/Src/leaf-oscillators.o ./compact-leaf/Src/leaf-oscillators.su ./compact-leaf/Src/leaf-physical.cyclo ./compact-leaf/Src/leaf-physical.d ./compact-leaf/Src/leaf-physical.o ./compact-leaf/Src/leaf-physical.su ./compact-leaf/Src/leaf-reverb.cyclo ./compact-leaf/Src/leaf-reverb.d ./compact-leaf/Src/leaf-reverb.o ./compact-leaf/Src/leaf-reverb.su ./compact-leaf/Src/leaf-sampling.cyclo ./compact-leaf/Src/leaf-sampling.d ./compact-leaf/Src/leaf-sampling.o ./compact-leaf/Src/leaf-sampling.su ./compact-leaf/Src/leaf-tables.cyclo ./compact-leaf/Src/leaf-tables.d ./compact-leaf/Src/leaf-tables.o ./compact-leaf/Src/leaf-tables.su ./compact-leaf/Src/leaf-vocal.cyclo ./compact-leaf/Src/leaf-vocal.d ./compact-leaf/Src/leaf-vocal.o ./compact-leaf/Src/leaf-vocal.su ./compact-leaf/Src/leaf.cyclo ./compact-leaf/Src/leaf.d ./compact-leaf/Src/leaf.o ./compact-leaf/Src/leaf.su

.PHONY: clean-compact-2d-leaf-2f-Src

