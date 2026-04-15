################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/additive.c \
../Core/Src/audiostream.c \
../Core/Src/codec.c \
../Core/Src/dma.c \
../Core/Src/fmc.c \
../Core/Src/gpio.c \
../Core/Src/i2c.c \
../Core/Src/main.c \
../Core/Src/rng.c \
../Core/Src/sai.c \
../Core/Src/sdmmc.c \
../Core/Src/spi.c \
../Core/Src/startup_stm32h750xx.c \
../Core/Src/stm32h7xx_hal_msp.c \
../Core/Src/stm32h7xx_it.c \
../Core/Src/string1.c \
../Core/Src/string2.c \
../Core/Src/string3.c \
../Core/Src/string4.c \
../Core/Src/synth.c \
../Core/Src/system_stm32h7xx.c \
../Core/Src/vocal.c 

OBJS += \
./Core/Src/additive.o \
./Core/Src/audiostream.o \
./Core/Src/codec.o \
./Core/Src/dma.o \
./Core/Src/fmc.o \
./Core/Src/gpio.o \
./Core/Src/i2c.o \
./Core/Src/main.o \
./Core/Src/rng.o \
./Core/Src/sai.o \
./Core/Src/sdmmc.o \
./Core/Src/spi.o \
./Core/Src/startup_stm32h750xx.o \
./Core/Src/stm32h7xx_hal_msp.o \
./Core/Src/stm32h7xx_it.o \
./Core/Src/string1.o \
./Core/Src/string2.o \
./Core/Src/string3.o \
./Core/Src/string4.o \
./Core/Src/synth.o \
./Core/Src/system_stm32h7xx.o \
./Core/Src/vocal.o 

C_DEPS += \
./Core/Src/additive.d \
./Core/Src/audiostream.d \
./Core/Src/codec.d \
./Core/Src/dma.d \
./Core/Src/fmc.d \
./Core/Src/gpio.d \
./Core/Src/i2c.d \
./Core/Src/main.d \
./Core/Src/rng.d \
./Core/Src/sai.d \
./Core/Src/sdmmc.d \
./Core/Src/spi.d \
./Core/Src/startup_stm32h750xx.d \
./Core/Src/stm32h7xx_hal_msp.d \
./Core/Src/stm32h7xx_it.d \
./Core/Src/string1.d \
./Core/Src/string2.d \
./Core/Src/string3.d \
./Core/Src/string4.d \
./Core/Src/synth.d \
./Core/Src/system_stm32h7xx.d \
./Core/Src/vocal.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../Drivers/CMSIS/Include -Ofast -ffunction-sections -fdata-sections -mslow-flash-data -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/additive.cyclo ./Core/Src/additive.d ./Core/Src/additive.o ./Core/Src/additive.su ./Core/Src/audiostream.cyclo ./Core/Src/audiostream.d ./Core/Src/audiostream.o ./Core/Src/audiostream.su ./Core/Src/codec.cyclo ./Core/Src/codec.d ./Core/Src/codec.o ./Core/Src/codec.su ./Core/Src/dma.cyclo ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/dma.su ./Core/Src/fmc.cyclo ./Core/Src/fmc.d ./Core/Src/fmc.o ./Core/Src/fmc.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/i2c.cyclo ./Core/Src/i2c.d ./Core/Src/i2c.o ./Core/Src/i2c.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/rng.cyclo ./Core/Src/rng.d ./Core/Src/rng.o ./Core/Src/rng.su ./Core/Src/sai.cyclo ./Core/Src/sai.d ./Core/Src/sai.o ./Core/Src/sai.su ./Core/Src/sdmmc.cyclo ./Core/Src/sdmmc.d ./Core/Src/sdmmc.o ./Core/Src/sdmmc.su ./Core/Src/spi.cyclo ./Core/Src/spi.d ./Core/Src/spi.o ./Core/Src/spi.su ./Core/Src/startup_stm32h750xx.cyclo ./Core/Src/startup_stm32h750xx.d ./Core/Src/startup_stm32h750xx.o ./Core/Src/startup_stm32h750xx.su ./Core/Src/stm32h7xx_hal_msp.cyclo ./Core/Src/stm32h7xx_hal_msp.d ./Core/Src/stm32h7xx_hal_msp.o ./Core/Src/stm32h7xx_hal_msp.su ./Core/Src/stm32h7xx_it.cyclo ./Core/Src/stm32h7xx_it.d ./Core/Src/stm32h7xx_it.o ./Core/Src/stm32h7xx_it.su ./Core/Src/string1.cyclo ./Core/Src/string1.d ./Core/Src/string1.o ./Core/Src/string1.su ./Core/Src/string2.cyclo ./Core/Src/string2.d ./Core/Src/string2.o ./Core/Src/string2.su ./Core/Src/string3.cyclo ./Core/Src/string3.d ./Core/Src/string3.o ./Core/Src/string3.su ./Core/Src/string4.cyclo ./Core/Src/string4.d ./Core/Src/string4.o ./Core/Src/string4.su ./Core/Src/synth.cyclo ./Core/Src/synth.d ./Core/Src/synth.o ./Core/Src/synth.su ./Core/Src/system_stm32h7xx.cyclo ./Core/Src/system_stm32h7xx.d ./Core/Src/system_stm32h7xx.o ./Core/Src/system_stm32h7xx.su ./Core/Src/vocal.cyclo ./Core/Src/vocal.d ./Core/Src/vocal.o ./Core/Src/vocal.su

.PHONY: clean-Core-2f-Src

