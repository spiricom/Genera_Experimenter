################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/Users/josnyder/dev/MappingandProcessors/processors/event_emitter.c \
/Users/josnyder/dev/MappingandProcessors/processors/funcmaps.c \
/Users/josnyder/dev/MappingandProcessors/processors/mapping.c \
/Users/josnyder/dev/MappingandProcessors/processors/midi_sysex.c \
/Users/josnyder/dev/MappingandProcessors/processors/sysex_chunks.c 

OBJS += \
./MappingandProcessors/processors/event_emitter.o \
./MappingandProcessors/processors/funcmaps.o \
./MappingandProcessors/processors/mapping.o \
./MappingandProcessors/processors/midi_sysex.o \
./MappingandProcessors/processors/sysex_chunks.o 

C_DEPS += \
./MappingandProcessors/processors/event_emitter.d \
./MappingandProcessors/processors/funcmaps.d \
./MappingandProcessors/processors/mapping.d \
./MappingandProcessors/processors/midi_sysex.d \
./MappingandProcessors/processors/sysex_chunks.d 


# Each subdirectory must supply rules for building sources it contributes
MappingandProcessors/processors/event_emitter.o: /Users/josnyder/dev/MappingandProcessors/processors/event_emitter.c MappingandProcessors/processors/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32H743xx -c -I/Users/josnyder/dev/MappingandProcessors/LEAF/leaf -I/Users/josnyder/dev/lm/LEAF/leaf/Inc -I/Users/josnyder/dev/lm/LEAF/leaf -I/Users/josnyder/dev/MappingandProcessors/modules -I/Users/josnyder/dev/MappingandProcessors/processors -I"/Users/josnyder/dev/Genera_Experimenter/Startup" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/CMSIS/Device/ST/STM32H7xx/Include" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/CMSIS/Include" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/STM32H7xx_HAL_Driver/Inc" -I../Inc -I"/Users/josnyder/dev/Genera_Experimenter/Debug" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/Third_Party/FatFs/src" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/ST/STM32_USB_Host_Library/Core/Inc" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/ST/STM32_USB_Host_Library/Class/AUDIO/Inc" -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -v -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
MappingandProcessors/processors/funcmaps.o: /Users/josnyder/dev/MappingandProcessors/processors/funcmaps.c MappingandProcessors/processors/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32H743xx -c -I/Users/josnyder/dev/MappingandProcessors/LEAF/leaf -I/Users/josnyder/dev/lm/LEAF/leaf/Inc -I/Users/josnyder/dev/lm/LEAF/leaf -I/Users/josnyder/dev/MappingandProcessors/modules -I/Users/josnyder/dev/MappingandProcessors/processors -I"/Users/josnyder/dev/Genera_Experimenter/Startup" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/CMSIS/Device/ST/STM32H7xx/Include" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/CMSIS/Include" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/STM32H7xx_HAL_Driver/Inc" -I../Inc -I"/Users/josnyder/dev/Genera_Experimenter/Debug" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/Third_Party/FatFs/src" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/ST/STM32_USB_Host_Library/Core/Inc" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/ST/STM32_USB_Host_Library/Class/AUDIO/Inc" -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -v -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
MappingandProcessors/processors/mapping.o: /Users/josnyder/dev/MappingandProcessors/processors/mapping.c MappingandProcessors/processors/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32H743xx -c -I/Users/josnyder/dev/MappingandProcessors/LEAF/leaf -I/Users/josnyder/dev/lm/LEAF/leaf/Inc -I/Users/josnyder/dev/lm/LEAF/leaf -I/Users/josnyder/dev/MappingandProcessors/modules -I/Users/josnyder/dev/MappingandProcessors/processors -I"/Users/josnyder/dev/Genera_Experimenter/Startup" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/CMSIS/Device/ST/STM32H7xx/Include" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/CMSIS/Include" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/STM32H7xx_HAL_Driver/Inc" -I../Inc -I"/Users/josnyder/dev/Genera_Experimenter/Debug" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/Third_Party/FatFs/src" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/ST/STM32_USB_Host_Library/Core/Inc" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/ST/STM32_USB_Host_Library/Class/AUDIO/Inc" -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -v -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
MappingandProcessors/processors/midi_sysex.o: /Users/josnyder/dev/MappingandProcessors/processors/midi_sysex.c MappingandProcessors/processors/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32H743xx -c -I/Users/josnyder/dev/MappingandProcessors/LEAF/leaf -I/Users/josnyder/dev/lm/LEAF/leaf/Inc -I/Users/josnyder/dev/lm/LEAF/leaf -I/Users/josnyder/dev/MappingandProcessors/modules -I/Users/josnyder/dev/MappingandProcessors/processors -I"/Users/josnyder/dev/Genera_Experimenter/Startup" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/CMSIS/Device/ST/STM32H7xx/Include" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/CMSIS/Include" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/STM32H7xx_HAL_Driver/Inc" -I../Inc -I"/Users/josnyder/dev/Genera_Experimenter/Debug" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/Third_Party/FatFs/src" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/ST/STM32_USB_Host_Library/Core/Inc" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/ST/STM32_USB_Host_Library/Class/AUDIO/Inc" -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -v -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
MappingandProcessors/processors/sysex_chunks.o: /Users/josnyder/dev/MappingandProcessors/processors/sysex_chunks.c MappingandProcessors/processors/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32H743xx -c -I/Users/josnyder/dev/MappingandProcessors/LEAF/leaf -I/Users/josnyder/dev/lm/LEAF/leaf/Inc -I/Users/josnyder/dev/lm/LEAF/leaf -I/Users/josnyder/dev/MappingandProcessors/modules -I/Users/josnyder/dev/MappingandProcessors/processors -I"/Users/josnyder/dev/Genera_Experimenter/Startup" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/CMSIS/Device/ST/STM32H7xx/Include" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/CMSIS/Include" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/STM32H7xx_HAL_Driver/Inc" -I../Inc -I"/Users/josnyder/dev/Genera_Experimenter/Debug" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/Third_Party/FatFs/src" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/ST/STM32_USB_Host_Library/Core/Inc" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/ST/STM32_USB_Host_Library/Class/AUDIO/Inc" -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -v -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-MappingandProcessors-2f-processors

clean-MappingandProcessors-2f-processors:
	-$(RM) ./MappingandProcessors/processors/event_emitter.cyclo ./MappingandProcessors/processors/event_emitter.d ./MappingandProcessors/processors/event_emitter.o ./MappingandProcessors/processors/event_emitter.su ./MappingandProcessors/processors/funcmaps.cyclo ./MappingandProcessors/processors/funcmaps.d ./MappingandProcessors/processors/funcmaps.o ./MappingandProcessors/processors/funcmaps.su ./MappingandProcessors/processors/mapping.cyclo ./MappingandProcessors/processors/mapping.d ./MappingandProcessors/processors/mapping.o ./MappingandProcessors/processors/mapping.su ./MappingandProcessors/processors/midi_sysex.cyclo ./MappingandProcessors/processors/midi_sysex.d ./MappingandProcessors/processors/midi_sysex.o ./MappingandProcessors/processors/midi_sysex.su ./MappingandProcessors/processors/sysex_chunks.cyclo ./MappingandProcessors/processors/sysex_chunks.d ./MappingandProcessors/processors/sysex_chunks.o ./MappingandProcessors/processors/sysex_chunks.su

.PHONY: clean-MappingandProcessors-2f-processors

