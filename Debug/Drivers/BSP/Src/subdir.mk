################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Src/ft5336.c \
../Drivers/BSP/Src/stm32746g_discovery.c \
../Drivers/BSP/Src/stm32746g_discovery_eeprom.c \
../Drivers/BSP/Src/stm32746g_discovery_lcd.c \
../Drivers/BSP/Src/stm32746g_discovery_qspi.c \
../Drivers/BSP/Src/stm32746g_discovery_sdram.c \
../Drivers/BSP/Src/stm32746g_discovery_ts.c 

OBJS += \
./Drivers/BSP/Src/ft5336.o \
./Drivers/BSP/Src/stm32746g_discovery.o \
./Drivers/BSP/Src/stm32746g_discovery_eeprom.o \
./Drivers/BSP/Src/stm32746g_discovery_lcd.o \
./Drivers/BSP/Src/stm32746g_discovery_qspi.o \
./Drivers/BSP/Src/stm32746g_discovery_sdram.o \
./Drivers/BSP/Src/stm32746g_discovery_ts.o 

C_DEPS += \
./Drivers/BSP/Src/ft5336.d \
./Drivers/BSP/Src/stm32746g_discovery.d \
./Drivers/BSP/Src/stm32746g_discovery_eeprom.d \
./Drivers/BSP/Src/stm32746g_discovery_lcd.d \
./Drivers/BSP/Src/stm32746g_discovery_qspi.d \
./Drivers/BSP/Src/stm32746g_discovery_sdram.d \
./Drivers/BSP/Src/stm32746g_discovery_ts.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Src/%.o Drivers/BSP/Src/%.su: ../Drivers/BSP/Src/%.c Drivers/BSP/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F746xx -c -I../LWIP/App -I../LWIP/Target -I../Core/Inc -I../Middlewares/Third_Party/LwIP/src/include -I../Middlewares/Third_Party/LwIP/system -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/BSP/Components/lan8742 -I../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Middlewares/Third_Party/LwIP/src/include/lwip -I../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../Middlewares/Third_Party/LwIP/src/include/netif -I../Middlewares/Third_Party/LwIP/src/include/compat/posix -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/net -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/sys -I../Middlewares/Third_Party/LwIP/src/include/compat/stdc -I../Middlewares/Third_Party/LwIP/system/arch -I../Drivers/CMSIS/Include -I../Drivers/BSP/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Src

clean-Drivers-2f-BSP-2f-Src:
	-$(RM) ./Drivers/BSP/Src/ft5336.d ./Drivers/BSP/Src/ft5336.o ./Drivers/BSP/Src/ft5336.su ./Drivers/BSP/Src/stm32746g_discovery.d ./Drivers/BSP/Src/stm32746g_discovery.o ./Drivers/BSP/Src/stm32746g_discovery.su ./Drivers/BSP/Src/stm32746g_discovery_eeprom.d ./Drivers/BSP/Src/stm32746g_discovery_eeprom.o ./Drivers/BSP/Src/stm32746g_discovery_eeprom.su ./Drivers/BSP/Src/stm32746g_discovery_lcd.d ./Drivers/BSP/Src/stm32746g_discovery_lcd.o ./Drivers/BSP/Src/stm32746g_discovery_lcd.su ./Drivers/BSP/Src/stm32746g_discovery_qspi.d ./Drivers/BSP/Src/stm32746g_discovery_qspi.o ./Drivers/BSP/Src/stm32746g_discovery_qspi.su ./Drivers/BSP/Src/stm32746g_discovery_sdram.d ./Drivers/BSP/Src/stm32746g_discovery_sdram.o ./Drivers/BSP/Src/stm32746g_discovery_sdram.su ./Drivers/BSP/Src/stm32746g_discovery_ts.d ./Drivers/BSP/Src/stm32746g_discovery_ts.o ./Drivers/BSP/Src/stm32746g_discovery_ts.su

.PHONY: clean-Drivers-2f-BSP-2f-Src

