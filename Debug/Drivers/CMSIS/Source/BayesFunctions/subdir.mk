################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/Users/jeffsnyder/dev/CMSIS/Source/BayesFunctions/BayesFunctions.c \
/Users/jeffsnyder/dev/CMSIS/Source/BayesFunctions/BayesFunctionsF16.c \
/Users/jeffsnyder/dev/CMSIS/Source/BayesFunctions/arm_gaussian_naive_bayes_predict_f16.c \
/Users/jeffsnyder/dev/CMSIS/Source/BayesFunctions/arm_gaussian_naive_bayes_predict_f32.c 

OBJS += \
./Drivers/CMSIS/Source/BayesFunctions/BayesFunctions.o \
./Drivers/CMSIS/Source/BayesFunctions/BayesFunctionsF16.o \
./Drivers/CMSIS/Source/BayesFunctions/arm_gaussian_naive_bayes_predict_f16.o \
./Drivers/CMSIS/Source/BayesFunctions/arm_gaussian_naive_bayes_predict_f32.o 

C_DEPS += \
./Drivers/CMSIS/Source/BayesFunctions/BayesFunctions.d \
./Drivers/CMSIS/Source/BayesFunctions/BayesFunctionsF16.d \
./Drivers/CMSIS/Source/BayesFunctions/arm_gaussian_naive_bayes_predict_f16.d \
./Drivers/CMSIS/Source/BayesFunctions/arm_gaussian_naive_bayes_predict_f32.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/Source/BayesFunctions/BayesFunctions.o: /Users/jeffsnyder/dev/CMSIS/Source/BayesFunctions/BayesFunctions.c Drivers/CMSIS/Source/BayesFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DARM_MATH_CM7 -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"/Users/jeffsnyder/dev/CMSIS/Include" -I"/Users/jeffsnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I"/Users/jeffsnyder/dev/LEAF/leaf" -Ofast -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/CMSIS/Source/BayesFunctions/BayesFunctionsF16.o: /Users/jeffsnyder/dev/CMSIS/Source/BayesFunctions/BayesFunctionsF16.c Drivers/CMSIS/Source/BayesFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DARM_MATH_CM7 -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"/Users/jeffsnyder/dev/CMSIS/Include" -I"/Users/jeffsnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I"/Users/jeffsnyder/dev/LEAF/leaf" -Ofast -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/CMSIS/Source/BayesFunctions/arm_gaussian_naive_bayes_predict_f16.o: /Users/jeffsnyder/dev/CMSIS/Source/BayesFunctions/arm_gaussian_naive_bayes_predict_f16.c Drivers/CMSIS/Source/BayesFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DARM_MATH_CM7 -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"/Users/jeffsnyder/dev/CMSIS/Include" -I"/Users/jeffsnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I"/Users/jeffsnyder/dev/LEAF/leaf" -Ofast -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/CMSIS/Source/BayesFunctions/arm_gaussian_naive_bayes_predict_f32.o: /Users/jeffsnyder/dev/CMSIS/Source/BayesFunctions/arm_gaussian_naive_bayes_predict_f32.c Drivers/CMSIS/Source/BayesFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DNO_DENORMAL_CHECK -DARM_MATH_CM7 -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"/Users/jeffsnyder/dev/CMSIS/Include" -I"/Users/jeffsnyder/dev/LEAF/leaf" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I"/Users/jeffsnyder/dev/LEAF/leaf" -Ofast -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS-2f-Source-2f-BayesFunctions

clean-Drivers-2f-CMSIS-2f-Source-2f-BayesFunctions:
	-$(RM) ./Drivers/CMSIS/Source/BayesFunctions/BayesFunctions.cyclo ./Drivers/CMSIS/Source/BayesFunctions/BayesFunctions.d ./Drivers/CMSIS/Source/BayesFunctions/BayesFunctions.o ./Drivers/CMSIS/Source/BayesFunctions/BayesFunctions.su ./Drivers/CMSIS/Source/BayesFunctions/BayesFunctionsF16.cyclo ./Drivers/CMSIS/Source/BayesFunctions/BayesFunctionsF16.d ./Drivers/CMSIS/Source/BayesFunctions/BayesFunctionsF16.o ./Drivers/CMSIS/Source/BayesFunctions/BayesFunctionsF16.su ./Drivers/CMSIS/Source/BayesFunctions/arm_gaussian_naive_bayes_predict_f16.cyclo ./Drivers/CMSIS/Source/BayesFunctions/arm_gaussian_naive_bayes_predict_f16.d ./Drivers/CMSIS/Source/BayesFunctions/arm_gaussian_naive_bayes_predict_f16.o ./Drivers/CMSIS/Source/BayesFunctions/arm_gaussian_naive_bayes_predict_f16.su ./Drivers/CMSIS/Source/BayesFunctions/arm_gaussian_naive_bayes_predict_f32.cyclo ./Drivers/CMSIS/Source/BayesFunctions/arm_gaussian_naive_bayes_predict_f32.d ./Drivers/CMSIS/Source/BayesFunctions/arm_gaussian_naive_bayes_predict_f32.o ./Drivers/CMSIS/Source/BayesFunctions/arm_gaussian_naive_bayes_predict_f32.su

.PHONY: clean-Drivers-2f-CMSIS-2f-Source-2f-BayesFunctions

