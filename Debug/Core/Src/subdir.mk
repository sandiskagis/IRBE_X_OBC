################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/main.c \
../Core/Src/stm32f0xx_hal_msp.c \
../Core/Src/stm32f0xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f0xx.c 

OBJS += \
./Core/Src/main.o \
./Core/Src/stm32f0xx_hal_msp.o \
./Core/Src/stm32f0xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f0xx.o 

C_DEPS += \
./Core/Src/main.d \
./Core/Src/stm32f0xx_hal_msp.d \
./Core/Src/stm32f0xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f0xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F030xC -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/axis0/Desktop/STM32/IRBE-X_OBC/Modules" -I"C:/Users/axis0/Desktop/STM32/IRBE-X_OBC/Modules/BME280" -I"C:/Users/axis0/Desktop/STM32/IRBE-X_OBC/Modules/MPU6050" -I"C:/Users/axis0/Desktop/STM32/IRBE-X_OBC/Modules/QMC5883" -I"C:/Users/axis0/Desktop/STM32/IRBE-X_OBC/FATFS/App" -I"C:/Users/axis0/Desktop/STM32/IRBE-X_OBC/FATFS/Target" -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I"C:/Users/axis0/Desktop/STM32/IRBE-X_OBC/Modules/SDCard" -I"C:/Users/kagis/OneDrive/Dators/Skola/EIPP3_2024/Faili_no_ieprieksejiem_gadiem/OBC_code/IRBE-X_OBC/Modules/SDCard" -I"C:/Users/kagis/OneDrive/Dators/Skola/EIPP3_2024/Faili_no_ieprieksejiem_gadiem/OBC_code/IRBE-X_OBC/Modules/BME280" -I"C:/Users/kagis/OneDrive/Dators/Skola/EIPP3_2024/Faili_no_ieprieksejiem_gadiem/OBC_code/IRBE-X_OBC/Modules/MPU6050" -I"C:/Users/kagis/OneDrive/Dators/Skola/EIPP3_2024/Faili_no_ieprieksejiem_gadiem/OBC_code/IRBE-X_OBC/Modules/QMC5883" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32f0xx_hal_msp.cyclo ./Core/Src/stm32f0xx_hal_msp.d ./Core/Src/stm32f0xx_hal_msp.o ./Core/Src/stm32f0xx_hal_msp.su ./Core/Src/stm32f0xx_it.cyclo ./Core/Src/stm32f0xx_it.d ./Core/Src/stm32f0xx_it.o ./Core/Src/stm32f0xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f0xx.cyclo ./Core/Src/system_stm32f0xx.d ./Core/Src/system_stm32f0xx.o ./Core/Src/system_stm32f0xx.su

.PHONY: clean-Core-2f-Src

