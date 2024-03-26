################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Modules/QMC5883/QMC5883_lib.c 

OBJS += \
./Modules/QMC5883/QMC5883_lib.o 

C_DEPS += \
./Modules/QMC5883/QMC5883_lib.d 


# Each subdirectory must supply rules for building sources it contributes
Modules/QMC5883/%.o Modules/QMC5883/%.su Modules/QMC5883/%.cyclo: ../Modules/QMC5883/%.c Modules/QMC5883/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F030xC -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/axis0/Desktop/STM32/IRBE-X_OBC/Modules" -I"C:/Users/axis0/Desktop/STM32/IRBE-X_OBC/Modules/BME280" -I"C:/Users/axis0/Desktop/STM32/IRBE-X_OBC/Modules/MPU6050" -I"C:/Users/axis0/Desktop/STM32/IRBE-X_OBC/Modules/QMC5883" -I"C:/Users/axis0/Desktop/STM32/IRBE-X_OBC/FATFS/App" -I"C:/Users/axis0/Desktop/STM32/IRBE-X_OBC/FATFS/Target" -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I"C:/Users/axis0/Desktop/STM32/IRBE-X_OBC/Modules/SDCard" -I"C:/Users/kagis/OneDrive/Dators/Skola/EIPP3_2024/Faili_no_ieprieksejiem_gadiem/OBC_code/IRBE-X_OBC/Modules/SDCard" -I"C:/Users/kagis/OneDrive/Dators/Skola/EIPP3_2024/Faili_no_ieprieksejiem_gadiem/OBC_code/IRBE-X_OBC/Modules/BME280" -I"C:/Users/kagis/OneDrive/Dators/Skola/EIPP3_2024/Faili_no_ieprieksejiem_gadiem/OBC_code/IRBE-X_OBC/Modules/MPU6050" -I"C:/Users/kagis/OneDrive/Dators/Skola/EIPP3_2024/Faili_no_ieprieksejiem_gadiem/OBC_code/IRBE-X_OBC/Modules/QMC5883" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Modules-2f-QMC5883

clean-Modules-2f-QMC5883:
	-$(RM) ./Modules/QMC5883/QMC5883_lib.cyclo ./Modules/QMC5883/QMC5883_lib.d ./Modules/QMC5883/QMC5883_lib.o ./Modules/QMC5883/QMC5883_lib.su

.PHONY: clean-Modules-2f-QMC5883

