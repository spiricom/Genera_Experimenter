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
./processors/event_emitter.o \
./processors/funcmaps.o \
./processors/mapping.o \
./processors/midi_sysex.o \
./processors/sysex_chunks.o 

C_DEPS += \
./processors/event_emitter.d \
./processors/funcmaps.d \
./processors/mapping.d \
./processors/midi_sysex.d \
./processors/sysex_chunks.d 


# Each subdirectory must supply rules for building sources it contributes
processors/event_emitter.o: /Users/josnyder/dev/lm/processors/event_emitter.c processors/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"/Users/josnyder/dev/Genera_Experimenter/leaf" -I/Users/josnyder/dev/lm/modules -I"/Users/josnyder/dev/Genera_Experimenter/Startup" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/CMSIS/Device/ST/STM32H7xx/Include" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/CMSIS/Include" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/STM32H7xx_HAL_Driver/Inc" -I../Inc -I"/Users/josnyder/dev/Genera_Experimenter/Debug" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/Third_Party/FatFs/src" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/ST/STM32_USB_Host_Library/Core/Inc" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/ST/STM32_USB_Host_Library/Class/AUDIO/Inc" -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -v -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
processors/funcmaps.o: /Users/josnyder/dev/lm/processors/funcmaps.c processors/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"/Users/josnyder/dev/Genera_Experimenter/leaf" -I/Users/josnyder/dev/lm/modules -I"/Users/josnyder/dev/Genera_Experimenter/Startup" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/CMSIS/Device/ST/STM32H7xx/Include" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/CMSIS/Include" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/STM32H7xx_HAL_Driver/Inc" -I../Inc -I"/Users/josnyder/dev/Genera_Experimenter/Debug" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/Third_Party/FatFs/src" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/ST/STM32_USB_Host_Library/Core/Inc" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/ST/STM32_USB_Host_Library/Class/AUDIO/Inc" -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -v -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
processors/mapping.o: /Users/josnyder/dev/lm/processors/mapping.c processors/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"/Users/josnyder/dev/Genera_Experimenter/leaf" -I/Users/josnyder/dev/lm/modules -I"/Users/josnyder/dev/Genera_Experimenter/Startup" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/CMSIS/Device/ST/STM32H7xx/Include" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/CMSIS/Include" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/STM32H7xx_HAL_Driver/Inc" -I../Inc -I"/Users/josnyder/dev/Genera_Experimenter/Debug" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/Third_Party/FatFs/src" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/ST/STM32_USB_Host_Library/Core/Inc" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/ST/STM32_USB_Host_Library/Class/AUDIO/Inc" -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -v -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
processors/midi_sysex.o: /Users/josnyder/dev/lm/processors/midi_sysex.c processors/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"/Users/josnyder/dev/Genera_Experimenter/leaf" -I/Users/josnyder/dev/lm/modules -I"/Users/josnyder/dev/Genera_Experimenter/Startup" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/CMSIS/Device/ST/STM32H7xx/Include" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/CMSIS/Include" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/STM32H7xx_HAL_Driver/Inc" -I../Inc -I"/Users/josnyder/dev/Genera_Experimenter/Debug" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/Third_Party/FatFs/src" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/ST/STM32_USB_Host_Library/Core/Inc" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/ST/STM32_USB_Host_Library/Class/AUDIO/Inc" -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -v -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
processors/sysex_chunks.o: /Users/josnyder/dev/lm/processors/sysex_chunks.c processors/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"/Users/josnyder/dev/Genera_Experimenter/leaf" -I/Users/josnyder/dev/lm/modules -I"/Users/josnyder/dev/Genera_Experimenter/Startup" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/CMSIS/Device/ST/STM32H7xx/Include" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/CMSIS/Include" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/STM32H7xx_HAL_Driver/Inc" -I../Inc -I"/Users/josnyder/dev/Genera_Experimenter/Debug" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/Third_Party/FatFs/src" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/ST/STM32_USB_Host_Library/Core/Inc" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/ST/STM32_USB_Host_Library/Class/AUDIO/Inc" -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -v -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-processors

clean-processors:
	-$(RM) ./processors/event_emitter.cyclo ./processors/event_emitter.d ./processors/event_emitter.o ./processors/event_emitter.su ./processors/funcmaps.cyclo ./processors/funcmaps.d ./processors/funcmaps.o ./processors/funcmaps.su ./processors/mapping.cyclo ./processors/mapping.d ./processors/mapping.o ./processors/mapping.su ./processors/midi_sysex.cyclo ./processors/midi_sysex.d ./processors/midi_sysex.o ./processors/midi_sysex.su ./processors/sysex_chunks.cyclo ./processors/sysex_chunks.d ./processors/sysex_chunks.o ./processors/sysex_chunks.su

.PHONY: clean-processors

