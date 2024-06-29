################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lib/pca9685/Src/pca9685.c 

C_DEPS += \
./lib/pca9685/Src/pca9685.d 

OBJS += \
./lib/pca9685/Src/pca9685.o 


# Each subdirectory must supply rules for building sources it contributes
lib/pca9685/Src/%.o lib/pca9685/Src/%.su lib/pca9685/Src/%.cyclo: ../lib/pca9685/Src/%.c lib/pca9685/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/migue/Desktop/Embedded/Workspace/hexapod_pruebas/lib/pca9685/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-lib-2f-pca9685-2f-Src

clean-lib-2f-pca9685-2f-Src:
	-$(RM) ./lib/pca9685/Src/pca9685.cyclo ./lib/pca9685/Src/pca9685.d ./lib/pca9685/Src/pca9685.o ./lib/pca9685/Src/pca9685.su

.PHONY: clean-lib-2f-pca9685-2f-Src

