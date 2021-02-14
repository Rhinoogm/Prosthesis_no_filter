################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/ecl_library.c \
../Src/main.c \
../Src/stm32f3xx_hal_msp.c \
../Src/stm32f3xx_it.c \
../Src/syscalls.c \
../Src/system_stm32f3xx.c \
../Src/vector_matrix.c 

OBJS += \
./Src/ecl_library.o \
./Src/main.o \
./Src/stm32f3xx_hal_msp.o \
./Src/stm32f3xx_it.o \
./Src/syscalls.o \
./Src/system_stm32f3xx.o \
./Src/vector_matrix.o 

C_DEPS += \
./Src/ecl_library.d \
./Src/main.d \
./Src/stm32f3xx_hal_msp.d \
./Src/stm32f3xx_it.d \
./Src/syscalls.d \
./Src/system_stm32f3xx.d \
./Src/vector_matrix.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F303x8 -I"C:/Users/rhinp/Desktop/Repos/Repos_Eclipse/prosthesis_new_connect_kalman filter 적용 전, 오히려 얘가 안정 적/Inc" -I"C:/Users/rhinp/Desktop/Repos/Repos_Eclipse/prosthesis_new_connect_kalman filter 적용 전, 오히려 얘가 안정 적/Drivers/STM32F3xx_HAL_Driver/Inc" -I"C:/Users/rhinp/Desktop/Repos/Repos_Eclipse/prosthesis_new_connect_kalman filter 적용 전, 오히려 얘가 안정 적/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"C:/Users/rhinp/Desktop/Repos/Repos_Eclipse/prosthesis_new_connect_kalman filter 적용 전, 오히려 얘가 안정 적/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"C:/Users/rhinp/Desktop/Repos/Repos_Eclipse/prosthesis_new_connect_kalman filter 적용 전, 오히려 얘가 안정 적/Drivers/CMSIS/Include"  -O2 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


