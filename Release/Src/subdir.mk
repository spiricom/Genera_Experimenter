################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/adc.c \
../Src/audiostream.c \
../Src/bsp_driver_sd.c \
../Src/codec.c \
../Src/debug.c \
../Src/dma.c \
../Src/fatfs.c \
../Src/fatfs_platform.c \
../Src/fmc.c \
../Src/gpio.c \
../Src/hrtim.c \
../Src/i2c.c \
../Src/main.c \
../Src/mdma.c \
../Src/rng.c \
../Src/sai.c \
../Src/sd_diskio.c \
../Src/sdmmc.c \
../Src/stm32h7xx_hal_msp.c \
../Src/stm32h7xx_it.c \
../Src/syscalls.c \
../Src/system_stm32h7xx.c \
../Src/ui.c \
../Src/wave.c 

OBJS += \
./Src/adc.o \
./Src/audiostream.o \
./Src/bsp_driver_sd.o \
./Src/codec.o \
./Src/debug.o \
./Src/dma.o \
./Src/fatfs.o \
./Src/fatfs_platform.o \
./Src/fmc.o \
./Src/gpio.o \
./Src/hrtim.o \
./Src/i2c.o \
./Src/main.o \
./Src/mdma.o \
./Src/rng.o \
./Src/sai.o \
./Src/sd_diskio.o \
./Src/sdmmc.o \
./Src/stm32h7xx_hal_msp.o \
./Src/stm32h7xx_it.o \
./Src/syscalls.o \
./Src/system_stm32h7xx.o \
./Src/ui.o \
./Src/wave.o 

C_DEPS += \
./Src/adc.d \
./Src/audiostream.d \
./Src/bsp_driver_sd.d \
./Src/codec.d \
./Src/debug.d \
./Src/dma.d \
./Src/fatfs.d \
./Src/fatfs_platform.d \
./Src/fmc.d \
./Src/gpio.d \
./Src/hrtim.d \
./Src/i2c.d \
./Src/main.d \
./Src/mdma.d \
./Src/rng.d \
./Src/sai.d \
./Src/sd_diskio.d \
./Src/sdmmc.d \
./Src/stm32h7xx_hal_msp.d \
./Src/stm32h7xx_it.d \
./Src/syscalls.d \
./Src/system_stm32h7xx.d \
./Src/ui.d \
./Src/wave.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32H743xx -c -I/Users/josnyder/dev/LEAF/leaf -I../Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FatFs/src -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -Og -ffunction-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/adc.cyclo ./Src/adc.d ./Src/adc.o ./Src/adc.su ./Src/audiostream.cyclo ./Src/audiostream.d ./Src/audiostream.o ./Src/audiostream.su ./Src/bsp_driver_sd.cyclo ./Src/bsp_driver_sd.d ./Src/bsp_driver_sd.o ./Src/bsp_driver_sd.su ./Src/codec.cyclo ./Src/codec.d ./Src/codec.o ./Src/codec.su ./Src/debug.cyclo ./Src/debug.d ./Src/debug.o ./Src/debug.su ./Src/dma.cyclo ./Src/dma.d ./Src/dma.o ./Src/dma.su ./Src/fatfs.cyclo ./Src/fatfs.d ./Src/fatfs.o ./Src/fatfs.su ./Src/fatfs_platform.cyclo ./Src/fatfs_platform.d ./Src/fatfs_platform.o ./Src/fatfs_platform.su ./Src/fmc.cyclo ./Src/fmc.d ./Src/fmc.o ./Src/fmc.su ./Src/gpio.cyclo ./Src/gpio.d ./Src/gpio.o ./Src/gpio.su ./Src/hrtim.cyclo ./Src/hrtim.d ./Src/hrtim.o ./Src/hrtim.su ./Src/i2c.cyclo ./Src/i2c.d ./Src/i2c.o ./Src/i2c.su ./Src/main.cyclo ./Src/main.d ./Src/main.o ./Src/main.su ./Src/mdma.cyclo ./Src/mdma.d ./Src/mdma.o ./Src/mdma.su ./Src/rng.cyclo ./Src/rng.d ./Src/rng.o ./Src/rng.su ./Src/sai.cyclo ./Src/sai.d ./Src/sai.o ./Src/sai.su ./Src/sd_diskio.cyclo ./Src/sd_diskio.d ./Src/sd_diskio.o ./Src/sd_diskio.su ./Src/sdmmc.cyclo ./Src/sdmmc.d ./Src/sdmmc.o ./Src/sdmmc.su ./Src/stm32h7xx_hal_msp.cyclo ./Src/stm32h7xx_hal_msp.d ./Src/stm32h7xx_hal_msp.o ./Src/stm32h7xx_hal_msp.su ./Src/stm32h7xx_it.cyclo ./Src/stm32h7xx_it.d ./Src/stm32h7xx_it.o ./Src/stm32h7xx_it.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/system_stm32h7xx.cyclo ./Src/system_stm32h7xx.d ./Src/system_stm32h7xx.o ./Src/system_stm32h7xx.su ./Src/ui.cyclo ./Src/ui.d ./Src/ui.o ./Src/ui.su ./Src/wave.cyclo ./Src/wave.d ./Src/wave.o ./Src/wave.su

.PHONY: clean-Src

