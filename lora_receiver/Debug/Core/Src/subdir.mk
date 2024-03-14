################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/bmp3.c \
../Core/Src/bmp390_task.c \
../Core/Src/common_porting.c \
../Core/Src/custom_bus.c \
../Core/Src/lora_sx1276.c \
../Core/Src/main.c \
../Core/Src/mira.c \
../Core/Src/nmea_parse.c \
../Core/Src/stm32l4xx_hal_msp.c \
../Core/Src/stm32l4xx_it.c \
../Core/Src/system_stm32l4xx.c 

OBJS += \
./Core/Src/bmp3.o \
./Core/Src/bmp390_task.o \
./Core/Src/common_porting.o \
./Core/Src/custom_bus.o \
./Core/Src/lora_sx1276.o \
./Core/Src/main.o \
./Core/Src/mira.o \
./Core/Src/nmea_parse.o \
./Core/Src/stm32l4xx_hal_msp.o \
./Core/Src/stm32l4xx_it.o \
./Core/Src/system_stm32l4xx.o 

C_DEPS += \
./Core/Src/bmp3.d \
./Core/Src/bmp390_task.d \
./Core/Src/common_porting.d \
./Core/Src/custom_bus.d \
./Core/Src/lora_sx1276.d \
./Core/Src/main.d \
./Core/Src/mira.d \
./Core/Src/nmea_parse.d \
./Core/Src/stm32l4xx_hal_msp.d \
./Core/Src/stm32l4xx_it.d \
./Core/Src/system_stm32l4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I../FATFS/Target -I/bmp388 -I/home/chospa/Documents/Github/MIRA/MIRA_OBC_Project/miracode/Core/bmp388 -I../FATFS/App -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"/home/shonyb/electronics/mira/lora_receiver/Core" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/main.o: ../Core/Src/main.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I../FATFS/Target -I/bmp388 -I/home/chospa/Documents/Github/MIRA/MIRA_OBC_Project/miracode/Core/bmp388 -I../FATFS/App -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"/home/shonyb/electronics/mira/lora_receiver/Core" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/bmp3.cyclo ./Core/Src/bmp3.d ./Core/Src/bmp3.o ./Core/Src/bmp3.su ./Core/Src/bmp390_task.cyclo ./Core/Src/bmp390_task.d ./Core/Src/bmp390_task.o ./Core/Src/bmp390_task.su ./Core/Src/common_porting.cyclo ./Core/Src/common_porting.d ./Core/Src/common_porting.o ./Core/Src/common_porting.su ./Core/Src/custom_bus.cyclo ./Core/Src/custom_bus.d ./Core/Src/custom_bus.o ./Core/Src/custom_bus.su ./Core/Src/lora_sx1276.cyclo ./Core/Src/lora_sx1276.d ./Core/Src/lora_sx1276.o ./Core/Src/lora_sx1276.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/mira.cyclo ./Core/Src/mira.d ./Core/Src/mira.o ./Core/Src/mira.su ./Core/Src/nmea_parse.cyclo ./Core/Src/nmea_parse.d ./Core/Src/nmea_parse.o ./Core/Src/nmea_parse.su ./Core/Src/stm32l4xx_hal_msp.cyclo ./Core/Src/stm32l4xx_hal_msp.d ./Core/Src/stm32l4xx_hal_msp.o ./Core/Src/stm32l4xx_hal_msp.su ./Core/Src/stm32l4xx_it.cyclo ./Core/Src/stm32l4xx_it.d ./Core/Src/stm32l4xx_it.o ./Core/Src/stm32l4xx_it.su ./Core/Src/system_stm32l4xx.cyclo ./Core/Src/system_stm32l4xx.d ./Core/Src/system_stm32l4xx.o ./Core/Src/system_stm32l4xx.su

.PHONY: clean-Core-2f-Src

