################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/FatFs/src/diskio.c \
../Middlewares/Third_Party/FatFs/src/ff.c \
../Middlewares/Third_Party/FatFs/src/ff_gen_drv.c 

OBJS += \
./Middlewares/Third_Party/FatFs/src/diskio.o \
./Middlewares/Third_Party/FatFs/src/ff.o \
./Middlewares/Third_Party/FatFs/src/ff_gen_drv.o 

C_DEPS += \
./Middlewares/Third_Party/FatFs/src/diskio.d \
./Middlewares/Third_Party/FatFs/src/ff.d \
./Middlewares/Third_Party/FatFs/src/ff_gen_drv.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/FatFs/src/%.o Middlewares/Third_Party/FatFs/src/%.su Middlewares/Third_Party/FatFs/src/%.cyclo: ../Middlewares/Third_Party/FatFs/src/%.c Middlewares/Third_Party/FatFs/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"/Users/josnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../Drivers/CMSIS/Include -Ofast -ffunction-sections -fdata-sections -mslow-flash-data -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-FatFs-2f-src

clean-Middlewares-2f-Third_Party-2f-FatFs-2f-src:
	-$(RM) ./Middlewares/Third_Party/FatFs/src/diskio.cyclo ./Middlewares/Third_Party/FatFs/src/diskio.d ./Middlewares/Third_Party/FatFs/src/diskio.o ./Middlewares/Third_Party/FatFs/src/diskio.su ./Middlewares/Third_Party/FatFs/src/ff.cyclo ./Middlewares/Third_Party/FatFs/src/ff.d ./Middlewares/Third_Party/FatFs/src/ff.o ./Middlewares/Third_Party/FatFs/src/ff.su ./Middlewares/Third_Party/FatFs/src/ff_gen_drv.cyclo ./Middlewares/Third_Party/FatFs/src/ff_gen_drv.d ./Middlewares/Third_Party/FatFs/src/ff_gen_drv.o ./Middlewares/Third_Party/FatFs/src/ff_gen_drv.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-FatFs-2f-src

