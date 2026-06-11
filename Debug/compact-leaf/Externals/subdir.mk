################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../compact-leaf/Externals/d_fft_mayer.c 

OBJS += \
./compact-leaf/Externals/d_fft_mayer.o 

C_DEPS += \
./compact-leaf/Externals/d_fft_mayer.d 


# Each subdirectory must supply rules for building sources it contributes
compact-leaf/Externals/%.o compact-leaf/Externals/%.su compact-leaf/Externals/%.cyclo: ../compact-leaf/Externals/%.c compact-leaf/Externals/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Inc -I"C:/Users/mattm/ChucCKS/Lab/JUCE stuff/Embedded stuff/Genera_Experimenter/compact-leaf" -I"C:/Users/mattm/ChucCKS/Lab/JUCE stuff/Embedded stuff/Genera_Experimenter/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/AUDIO/Inc -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -Og -ffunction-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-compact-2d-leaf-2f-Externals

clean-compact-2d-leaf-2f-Externals:
	-$(RM) ./compact-leaf/Externals/d_fft_mayer.cyclo ./compact-leaf/Externals/d_fft_mayer.d ./compact-leaf/Externals/d_fft_mayer.o ./compact-leaf/Externals/d_fft_mayer.su

.PHONY: clean-compact-2d-leaf-2f-Externals

