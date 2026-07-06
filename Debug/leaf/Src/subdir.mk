################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../leaf/Src/leaf-analysis.c \
../leaf/Src/leaf-delay.c \
../leaf/Src/leaf-distortion.c \
../leaf/Src/leaf-dynamics.c \
../leaf/Src/leaf-effects.c \
../leaf/Src/leaf-electrical.c \
../leaf/Src/leaf-envelopes.c \
../leaf/Src/leaf-filters.c \
../leaf/Src/leaf-instruments.c \
../leaf/Src/leaf-math.c \
../leaf/Src/leaf-mempool.c \
../leaf/Src/leaf-midi.c \
../leaf/Src/leaf-oscillators.c \
../leaf/Src/leaf-physical.c \
../leaf/Src/leaf-reverb.c \
../leaf/Src/leaf-sampling.c \
../leaf/Src/leaf-tables.c \
../leaf/Src/leaf-vocal.c \
../leaf/Src/leaf.c 

OBJS += \
./leaf/Src/leaf-analysis.o \
./leaf/Src/leaf-delay.o \
./leaf/Src/leaf-distortion.o \
./leaf/Src/leaf-dynamics.o \
./leaf/Src/leaf-effects.o \
./leaf/Src/leaf-electrical.o \
./leaf/Src/leaf-envelopes.o \
./leaf/Src/leaf-filters.o \
./leaf/Src/leaf-instruments.o \
./leaf/Src/leaf-math.o \
./leaf/Src/leaf-mempool.o \
./leaf/Src/leaf-midi.o \
./leaf/Src/leaf-oscillators.o \
./leaf/Src/leaf-physical.o \
./leaf/Src/leaf-reverb.o \
./leaf/Src/leaf-sampling.o \
./leaf/Src/leaf-tables.o \
./leaf/Src/leaf-vocal.o \
./leaf/Src/leaf.o 

C_DEPS += \
./leaf/Src/leaf-analysis.d \
./leaf/Src/leaf-delay.d \
./leaf/Src/leaf-distortion.d \
./leaf/Src/leaf-dynamics.d \
./leaf/Src/leaf-effects.d \
./leaf/Src/leaf-electrical.d \
./leaf/Src/leaf-envelopes.d \
./leaf/Src/leaf-filters.d \
./leaf/Src/leaf-instruments.d \
./leaf/Src/leaf-math.d \
./leaf/Src/leaf-mempool.d \
./leaf/Src/leaf-midi.d \
./leaf/Src/leaf-oscillators.d \
./leaf/Src/leaf-physical.d \
./leaf/Src/leaf-reverb.d \
./leaf/Src/leaf-sampling.d \
./leaf/Src/leaf-tables.d \
./leaf/Src/leaf-vocal.d \
./leaf/Src/leaf.d 


# Each subdirectory must supply rules for building sources it contributes
leaf/Src/%.o leaf/Src/%.su leaf/Src/%.cyclo: ../leaf/Src/%.c leaf/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu99 -g3 -DUSE_HAL_DRIVER -DSTM32H743xx -DDEBUG -DUSE_PWR_LDO_SUPPLY -c -I../Inc -I"/Users/josnyder/dev/Vocodec/leaf" -I../Drivers/CMSIS/Include -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/ST/STM32_USB_Host_Library/Class/AUDIO/Inc -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -Ofast -ffunction-sections -fdata-sections -mslow-flash-data -fno-strict-aliasing -Wall -Wextra -fwrapv -fno-finite-math-only -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-leaf-2f-Src

clean-leaf-2f-Src:
	-$(RM) ./leaf/Src/leaf-analysis.cyclo ./leaf/Src/leaf-analysis.d ./leaf/Src/leaf-analysis.o ./leaf/Src/leaf-analysis.su ./leaf/Src/leaf-delay.cyclo ./leaf/Src/leaf-delay.d ./leaf/Src/leaf-delay.o ./leaf/Src/leaf-delay.su ./leaf/Src/leaf-distortion.cyclo ./leaf/Src/leaf-distortion.d ./leaf/Src/leaf-distortion.o ./leaf/Src/leaf-distortion.su ./leaf/Src/leaf-dynamics.cyclo ./leaf/Src/leaf-dynamics.d ./leaf/Src/leaf-dynamics.o ./leaf/Src/leaf-dynamics.su ./leaf/Src/leaf-effects.cyclo ./leaf/Src/leaf-effects.d ./leaf/Src/leaf-effects.o ./leaf/Src/leaf-effects.su ./leaf/Src/leaf-electrical.cyclo ./leaf/Src/leaf-electrical.d ./leaf/Src/leaf-electrical.o ./leaf/Src/leaf-electrical.su ./leaf/Src/leaf-envelopes.cyclo ./leaf/Src/leaf-envelopes.d ./leaf/Src/leaf-envelopes.o ./leaf/Src/leaf-envelopes.su ./leaf/Src/leaf-filters.cyclo ./leaf/Src/leaf-filters.d ./leaf/Src/leaf-filters.o ./leaf/Src/leaf-filters.su ./leaf/Src/leaf-instruments.cyclo ./leaf/Src/leaf-instruments.d ./leaf/Src/leaf-instruments.o ./leaf/Src/leaf-instruments.su ./leaf/Src/leaf-math.cyclo ./leaf/Src/leaf-math.d ./leaf/Src/leaf-math.o ./leaf/Src/leaf-math.su ./leaf/Src/leaf-mempool.cyclo ./leaf/Src/leaf-mempool.d ./leaf/Src/leaf-mempool.o ./leaf/Src/leaf-mempool.su ./leaf/Src/leaf-midi.cyclo ./leaf/Src/leaf-midi.d ./leaf/Src/leaf-midi.o ./leaf/Src/leaf-midi.su ./leaf/Src/leaf-oscillators.cyclo ./leaf/Src/leaf-oscillators.d ./leaf/Src/leaf-oscillators.o ./leaf/Src/leaf-oscillators.su ./leaf/Src/leaf-physical.cyclo ./leaf/Src/leaf-physical.d ./leaf/Src/leaf-physical.o ./leaf/Src/leaf-physical.su ./leaf/Src/leaf-reverb.cyclo ./leaf/Src/leaf-reverb.d ./leaf/Src/leaf-reverb.o ./leaf/Src/leaf-reverb.su ./leaf/Src/leaf-sampling.cyclo ./leaf/Src/leaf-sampling.d ./leaf/Src/leaf-sampling.o ./leaf/Src/leaf-sampling.su ./leaf/Src/leaf-tables.cyclo ./leaf/Src/leaf-tables.d ./leaf/Src/leaf-tables.o ./leaf/Src/leaf-tables.su ./leaf/Src/leaf-vocal.cyclo ./leaf/Src/leaf-vocal.d ./leaf/Src/leaf-vocal.o ./leaf/Src/leaf-vocal.su ./leaf/Src/leaf.cyclo ./leaf/Src/leaf.d ./leaf/Src/leaf.o ./leaf/Src/leaf.su

.PHONY: clean-leaf-2f-Src

