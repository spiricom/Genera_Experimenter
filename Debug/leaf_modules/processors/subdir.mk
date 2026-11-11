################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/Users/josnyder/dev/lm/processors/event_emitter.c \
/Users/josnyder/dev/lm/processors/funcmaps.c \
/Users/josnyder/dev/lm/processors/mapping.c \
/Users/josnyder/dev/lm/processors/midi_sysex.c \
/Users/josnyder/dev/lm/processors/sysex_chunks.c 

OBJS += \
./leaf_modules/processors/event_emitter.o \
./leaf_modules/processors/funcmaps.o \
./leaf_modules/processors/mapping.o \
./leaf_modules/processors/midi_sysex.o \
./leaf_modules/processors/sysex_chunks.o 

C_DEPS += \
./leaf_modules/processors/event_emitter.d \
./leaf_modules/processors/funcmaps.d \
./leaf_modules/processors/mapping.d \
./leaf_modules/processors/midi_sysex.d \
./leaf_modules/processors/sysex_chunks.d 


# Each subdirectory must supply rules for building sources it contributes
leaf_modules/processors/event_emitter.o: /Users/josnyder/dev/lm/processors/event_emitter.c leaf_modules/processors/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DUSE_HAL_DRIVER -DSTM32H743xx -c -I/Users/josnyder/dev/LEAF/leaf/Inc -I/Users/josnyder/dev/LEAF/leaf -I/Users/josnyder/dev/lm/processors -I/Users/josnyder/dev/lm/modules -I"/Users/josnyder/dev/Genera_Experimenter/Startup" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/CMSIS/Device/ST/STM32H7xx/Include" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/CMSIS/Include" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/STM32H7xx_HAL_Driver/Inc" -I../Inc -I"/Users/josnyder/dev/Genera_Experimenter/Debug" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/Third_Party/FatFs/src" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/ST/STM32_USB_Host_Library/Core/Inc" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/ST/STM32_USB_Host_Library/Class/AUDIO/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
leaf_modules/processors/funcmaps.o: /Users/josnyder/dev/lm/processors/funcmaps.c leaf_modules/processors/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DUSE_HAL_DRIVER -DSTM32H743xx -c -I/Users/josnyder/dev/LEAF/leaf/Inc -I/Users/josnyder/dev/LEAF/leaf -I/Users/josnyder/dev/lm/processors -I/Users/josnyder/dev/lm/modules -I"/Users/josnyder/dev/Genera_Experimenter/Startup" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/CMSIS/Device/ST/STM32H7xx/Include" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/CMSIS/Include" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/STM32H7xx_HAL_Driver/Inc" -I../Inc -I"/Users/josnyder/dev/Genera_Experimenter/Debug" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/Third_Party/FatFs/src" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/ST/STM32_USB_Host_Library/Core/Inc" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/ST/STM32_USB_Host_Library/Class/AUDIO/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
leaf_modules/processors/mapping.o: /Users/josnyder/dev/lm/processors/mapping.c leaf_modules/processors/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DUSE_HAL_DRIVER -DSTM32H743xx -c -I/Users/josnyder/dev/LEAF/leaf/Inc -I/Users/josnyder/dev/LEAF/leaf -I/Users/josnyder/dev/lm/processors -I/Users/josnyder/dev/lm/modules -I"/Users/josnyder/dev/Genera_Experimenter/Startup" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/CMSIS/Device/ST/STM32H7xx/Include" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/CMSIS/Include" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/STM32H7xx_HAL_Driver/Inc" -I../Inc -I"/Users/josnyder/dev/Genera_Experimenter/Debug" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/Third_Party/FatFs/src" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/ST/STM32_USB_Host_Library/Core/Inc" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/ST/STM32_USB_Host_Library/Class/AUDIO/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
leaf_modules/processors/midi_sysex.o: /Users/josnyder/dev/lm/processors/midi_sysex.c leaf_modules/processors/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DUSE_HAL_DRIVER -DSTM32H743xx -c -I/Users/josnyder/dev/LEAF/leaf/Inc -I/Users/josnyder/dev/LEAF/leaf -I/Users/josnyder/dev/lm/processors -I/Users/josnyder/dev/lm/modules -I"/Users/josnyder/dev/Genera_Experimenter/Startup" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/CMSIS/Device/ST/STM32H7xx/Include" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/CMSIS/Include" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/STM32H7xx_HAL_Driver/Inc" -I../Inc -I"/Users/josnyder/dev/Genera_Experimenter/Debug" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/Third_Party/FatFs/src" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/ST/STM32_USB_Host_Library/Core/Inc" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/ST/STM32_USB_Host_Library/Class/AUDIO/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
leaf_modules/processors/sysex_chunks.o: /Users/josnyder/dev/lm/processors/sysex_chunks.c leaf_modules/processors/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DUSE_HAL_DRIVER -DSTM32H743xx -c -I/Users/josnyder/dev/LEAF/leaf/Inc -I/Users/josnyder/dev/LEAF/leaf -I/Users/josnyder/dev/lm/processors -I/Users/josnyder/dev/lm/modules -I"/Users/josnyder/dev/Genera_Experimenter/Startup" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/CMSIS/Device/ST/STM32H7xx/Include" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/CMSIS/Include" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/STM32H7xx_HAL_Driver/Inc" -I../Inc -I"/Users/josnyder/dev/Genera_Experimenter/Debug" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/Third_Party/FatFs/src" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/ST/STM32_USB_Host_Library/Core/Inc" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/ST/STM32_USB_Host_Library/Class/AUDIO/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-leaf_modules-2f-processors

clean-leaf_modules-2f-processors:
	-$(RM) ./leaf_modules/processors/event_emitter.cyclo ./leaf_modules/processors/event_emitter.d ./leaf_modules/processors/event_emitter.o ./leaf_modules/processors/event_emitter.su ./leaf_modules/processors/funcmaps.cyclo ./leaf_modules/processors/funcmaps.d ./leaf_modules/processors/funcmaps.o ./leaf_modules/processors/funcmaps.su ./leaf_modules/processors/mapping.cyclo ./leaf_modules/processors/mapping.d ./leaf_modules/processors/mapping.o ./leaf_modules/processors/mapping.su ./leaf_modules/processors/midi_sysex.cyclo ./leaf_modules/processors/midi_sysex.d ./leaf_modules/processors/midi_sysex.o ./leaf_modules/processors/midi_sysex.su ./leaf_modules/processors/sysex_chunks.cyclo ./leaf_modules/processors/sysex_chunks.d ./leaf_modules/processors/sysex_chunks.o ./leaf_modules/processors/sysex_chunks.su

.PHONY: clean-leaf_modules-2f-processors

