################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../leaf/Externals/d_fft_mayer.c 

OBJS += \
./leaf/Externals/d_fft_mayer.o 

C_DEPS += \
./leaf/Externals/d_fft_mayer.d 


# Each subdirectory must supply rules for building sources it contributes
leaf/Externals/%.o leaf/Externals/%.su leaf/Externals/%.cyclo: ../leaf/Externals/%.c leaf/Externals/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"/Users/josnyder/dev/Genera_Experimenter/leaf" -I"/Users/josnyder/dev/Genera_Experimenter/Startup" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/CMSIS/Device/ST/STM32H7xx/Include" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/CMSIS/Include" -I"/Users/josnyder/dev/Genera_Experimenter/Drivers/STM32H7xx_HAL_Driver/Inc" -I../Inc -I"/Users/josnyder/dev/Genera_Experimenter/Debug" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/Third_Party/FatFs/src" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/ST/STM32_USB_Host_Library/Core/Inc" -I"/Users/josnyder/dev/Genera_Experimenter/Middlewares/ST/STM32_USB_Host_Library/Class/AUDIO/Inc" -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -v -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-leaf-2f-Externals

clean-leaf-2f-Externals:
	-$(RM) ./leaf/Externals/d_fft_mayer.cyclo ./leaf/Externals/d_fft_mayer.d ./leaf/Externals/d_fft_mayer.o ./leaf/Externals/d_fft_mayer.su

.PHONY: clean-leaf-2f-Externals

