################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/PID.c \
../Src/avoidence.c \
../Src/circular_buff.c \
../Src/interrupt.c \
../Src/main.c \
../Src/setup.c \
../Src/syscalls.c \
../Src/sysmem.c \
../Src/ultrasonic.c 

OBJS += \
./Src/PID.o \
./Src/avoidence.o \
./Src/circular_buff.o \
./Src/interrupt.o \
./Src/main.o \
./Src/setup.o \
./Src/syscalls.o \
./Src/sysmem.o \
./Src/ultrasonic.o 

C_DEPS += \
./Src/PID.d \
./Src/avoidence.d \
./Src/circular_buff.d \
./Src/interrupt.d \
./Src/main.d \
./Src/setup.d \
./Src/syscalls.d \
./Src/sysmem.d \
./Src/ultrasonic.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -c -I../Inc -I"C:/Users/kiscs/STM32CubeIDE/workspace_1.18.1/RC_Car/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/PID.cyclo ./Src/PID.d ./Src/PID.o ./Src/PID.su ./Src/avoidence.cyclo ./Src/avoidence.d ./Src/avoidence.o ./Src/avoidence.su ./Src/circular_buff.cyclo ./Src/circular_buff.d ./Src/circular_buff.o ./Src/circular_buff.su ./Src/interrupt.cyclo ./Src/interrupt.d ./Src/interrupt.o ./Src/interrupt.su ./Src/main.cyclo ./Src/main.d ./Src/main.o ./Src/main.su ./Src/setup.cyclo ./Src/setup.d ./Src/setup.o ./Src/setup.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su ./Src/ultrasonic.cyclo ./Src/ultrasonic.d ./Src/ultrasonic.o ./Src/ultrasonic.su

.PHONY: clean-Src

