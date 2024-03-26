################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
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
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F030xC -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/axis0/Desktop/STM32/IRBE-X_OBC/Modules" -I"C:/Users/axis0/Desktop/STM32/IRBE-X_OBC/Modules/BME280" -I"C:/Users/axis0/Desktop/STM32/IRBE-X_OBC/Modules/MPU6050" -I"C:/Users/axis0/Desktop/STM32/IRBE-X_OBC/Modules/QMC5883" -I"C:/Users/axis0/Desktop/STM32/IRBE-X_OBC/FATFS/App" -I"C:/Users/axis0/Desktop/STM32/IRBE-X_OBC/FATFS/Target" -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I"C:/Users/axis0/Desktop/STM32/IRBE-X_OBC/Modules/SDCard" -I"C:/Users/kagis/OneDrive/Dators/Skola/EIPP3_2024/Faili_no_ieprieksejiem_gadiem/OBC_code/IRBE-X_OBC/Modules/SDCard" -I"C:/Users/kagis/OneDrive/Dators/Skola/EIPP3_2024/Faili_no_ieprieksejiem_gadiem/OBC_code/IRBE-X_OBC/Modules/BME280" -I"C:/Users/kagis/OneDrive/Dators/Skola/EIPP3_2024/Faili_no_ieprieksejiem_gadiem/OBC_code/IRBE-X_OBC/Modules/MPU6050" -I"C:/Users/kagis/OneDrive/Dators/Skola/EIPP3_2024/Faili_no_ieprieksejiem_gadiem/OBC_code/IRBE-X_OBC/Modules/QMC5883" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-FatFs-2f-src

clean-Middlewares-2f-Third_Party-2f-FatFs-2f-src:
	-$(RM) ./Middlewares/Third_Party/FatFs/src/diskio.cyclo ./Middlewares/Third_Party/FatFs/src/diskio.d ./Middlewares/Third_Party/FatFs/src/diskio.o ./Middlewares/Third_Party/FatFs/src/diskio.su ./Middlewares/Third_Party/FatFs/src/ff.cyclo ./Middlewares/Third_Party/FatFs/src/ff.d ./Middlewares/Third_Party/FatFs/src/ff.o ./Middlewares/Third_Party/FatFs/src/ff.su ./Middlewares/Third_Party/FatFs/src/ff_gen_drv.cyclo ./Middlewares/Third_Party/FatFs/src/ff_gen_drv.d ./Middlewares/Third_Party/FatFs/src/ff_gen_drv.o ./Middlewares/Third_Party/FatFs/src/ff_gen_drv.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-FatFs-2f-src

