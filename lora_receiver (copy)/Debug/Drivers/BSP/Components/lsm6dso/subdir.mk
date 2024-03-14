################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/lsm6dso/lsm6dso.c \
../Drivers/BSP/Components/lsm6dso/lsm6dso_reg.c 

OBJS += \
./Drivers/BSP/Components/lsm6dso/lsm6dso.o \
./Drivers/BSP/Components/lsm6dso/lsm6dso_reg.o 

C_DEPS += \
./Drivers/BSP/Components/lsm6dso/lsm6dso.d \
./Drivers/BSP/Components/lsm6dso/lsm6dso_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/lsm6dso/%.o Drivers/BSP/Components/lsm6dso/%.su Drivers/BSP/Components/lsm6dso/%.cyclo: ../Drivers/BSP/Components/lsm6dso/%.c Drivers/BSP/Components/lsm6dso/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I../FATFS/Target -I/home/chospa/Documents/Github/MIRA/MIRA_OBC_Project/miracode/Core/bmp388 -I../FATFS/App -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"/home/shonyb/electronics/mira/miracode/Core" -I../X-CUBE-MEMS1/Target -I../Drivers/BSP/Components/lsm6dso -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Components-2f-lsm6dso

clean-Drivers-2f-BSP-2f-Components-2f-lsm6dso:
	-$(RM) ./Drivers/BSP/Components/lsm6dso/lsm6dso.cyclo ./Drivers/BSP/Components/lsm6dso/lsm6dso.d ./Drivers/BSP/Components/lsm6dso/lsm6dso.o ./Drivers/BSP/Components/lsm6dso/lsm6dso.su ./Drivers/BSP/Components/lsm6dso/lsm6dso_reg.cyclo ./Drivers/BSP/Components/lsm6dso/lsm6dso_reg.d ./Drivers/BSP/Components/lsm6dso/lsm6dso_reg.o ./Drivers/BSP/Components/lsm6dso/lsm6dso_reg.su

.PHONY: clean-Drivers-2f-BSP-2f-Components-2f-lsm6dso

