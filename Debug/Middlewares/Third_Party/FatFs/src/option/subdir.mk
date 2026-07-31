################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/FatFs/src/option/ccsbcs.c \
../Middlewares/Third_Party/FatFs/src/option/syscall.c 

OBJS += \
./Middlewares/Third_Party/FatFs/src/option/ccsbcs.o \
./Middlewares/Third_Party/FatFs/src/option/syscall.o 

C_DEPS += \
./Middlewares/Third_Party/FatFs/src/option/ccsbcs.d \
./Middlewares/Third_Party/FatFs/src/option/syscall.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/FatFs/src/option/%.o Middlewares/Third_Party/FatFs/src/option/%.su Middlewares/Third_Party/FatFs/src/option/%.cyclo: ../Middlewares/Third_Party/FatFs/src/option/%.c Middlewares/Third_Party/FatFs/src/option/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"/Users/josnyder/dev/Genera_Experimenter/leaf" -I"/Users/josnyder/dev/Genera_Experimenter/Startup" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/CMSIS/Device/ST/STM32H7xx/Include" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/CMSIS/Include" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/STM32H7xx_HAL_Driver/Inc" -I../Inc -I"/Users/josnyder/dev/Genera_Experimenter/Debug" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/Third_Party/FatFs/src" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/ST/STM32_USB_Host_Library/Core/Inc" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/ST/STM32_USB_Host_Library/Class/AUDIO/Inc" -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -v -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-FatFs-2f-src-2f-option

clean-Middlewares-2f-Third_Party-2f-FatFs-2f-src-2f-option:
	-$(RM) ./Middlewares/Third_Party/FatFs/src/option/ccsbcs.cyclo ./Middlewares/Third_Party/FatFs/src/option/ccsbcs.d ./Middlewares/Third_Party/FatFs/src/option/ccsbcs.o ./Middlewares/Third_Party/FatFs/src/option/ccsbcs.su ./Middlewares/Third_Party/FatFs/src/option/syscall.cyclo ./Middlewares/Third_Party/FatFs/src/option/syscall.d ./Middlewares/Third_Party/FatFs/src/option/syscall.o ./Middlewares/Third_Party/FatFs/src/option/syscall.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-FatFs-2f-src-2f-option

