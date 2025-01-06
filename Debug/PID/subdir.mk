################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../PID/PID.c 

OBJS += \
./PID/PID.o 

C_DEPS += \
./PID/PID.d 


# Each subdirectory must supply rules for building sources it contributes
PID/%.o PID/%.su PID/%.cyclo: ../PID/%.c PID/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"C:/Users/ADMIN/STM32CubeIDE/workspace_1.16.1/DoAnRoBot/ENCODER" -I"C:/Users/ADMIN/STM32CubeIDE/workspace_1.16.1/DoAnRoBot/PID" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -gdwarf-4 -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-PID

clean-PID:
	-$(RM) ./PID/PID.cyclo ./PID/PID.d ./PID/PID.o ./PID/PID.su

.PHONY: clean-PID

