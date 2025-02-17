################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Libraries/iLLD/TC27D/Tricore/Iom/Driver/ASCLIN_Shell_UART.c \
../Libraries/iLLD/TC27D/Tricore/Iom/Driver/Com_Filter.c \
../Libraries/iLLD/TC27D/Tricore/Iom/Driver/IMU_Driver.c \
../Libraries/iLLD/TC27D/Tricore/Iom/Driver/IfxIom_Driver.c \
../Libraries/iLLD/TC27D/Tricore/Iom/Driver/MadgwickAHRS.c \
../Libraries/iLLD/TC27D/Tricore/Iom/Driver/Ultra_Driver.c \
../Libraries/iLLD/TC27D/Tricore/Iom/Driver/ert_main.c \
../Libraries/iLLD/TC27D/Tricore/Iom/Driver/vel_filter.c 

C_DEPS += \
./Libraries/iLLD/TC27D/Tricore/Iom/Driver/ASCLIN_Shell_UART.d \
./Libraries/iLLD/TC27D/Tricore/Iom/Driver/Com_Filter.d \
./Libraries/iLLD/TC27D/Tricore/Iom/Driver/IMU_Driver.d \
./Libraries/iLLD/TC27D/Tricore/Iom/Driver/IfxIom_Driver.d \
./Libraries/iLLD/TC27D/Tricore/Iom/Driver/MadgwickAHRS.d \
./Libraries/iLLD/TC27D/Tricore/Iom/Driver/Ultra_Driver.d \
./Libraries/iLLD/TC27D/Tricore/Iom/Driver/ert_main.d \
./Libraries/iLLD/TC27D/Tricore/Iom/Driver/vel_filter.d 

OBJS += \
./Libraries/iLLD/TC27D/Tricore/Iom/Driver/ASCLIN_Shell_UART.o \
./Libraries/iLLD/TC27D/Tricore/Iom/Driver/Com_Filter.o \
./Libraries/iLLD/TC27D/Tricore/Iom/Driver/IMU_Driver.o \
./Libraries/iLLD/TC27D/Tricore/Iom/Driver/IfxIom_Driver.o \
./Libraries/iLLD/TC27D/Tricore/Iom/Driver/MadgwickAHRS.o \
./Libraries/iLLD/TC27D/Tricore/Iom/Driver/Ultra_Driver.o \
./Libraries/iLLD/TC27D/Tricore/Iom/Driver/ert_main.o \
./Libraries/iLLD/TC27D/Tricore/Iom/Driver/vel_filter.o 


# Each subdirectory must supply rules for building sources it contributes
Libraries/iLLD/TC27D/Tricore/Iom/Driver/%.o: ../Libraries/iLLD/TC27D/Tricore/Iom/Driver/%.c Libraries/iLLD/TC27D/Tricore/Iom/Driver/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: AURIX GCC Compiler'
	tricore-elf-gcc -std=c99 "@C:/Users/USER/AURIX_PJT/AutoDPJT/TriCore Debug (GCC)/AURIX_GCC_Compiler-Include_paths__-I_.opt" -Og -g3 -gdwarf-3 -Wall -c -fmessage-length=0 -fno-common -fstrict-volatile-bitfields -fdata-sections -ffunction-sections -mtc161 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-Libraries-2f-iLLD-2f-TC27D-2f-Tricore-2f-Iom-2f-Driver

clean-Libraries-2f-iLLD-2f-TC27D-2f-Tricore-2f-Iom-2f-Driver:
	-$(RM) ./Libraries/iLLD/TC27D/Tricore/Iom/Driver/ASCLIN_Shell_UART.d ./Libraries/iLLD/TC27D/Tricore/Iom/Driver/ASCLIN_Shell_UART.o ./Libraries/iLLD/TC27D/Tricore/Iom/Driver/Com_Filter.d ./Libraries/iLLD/TC27D/Tricore/Iom/Driver/Com_Filter.o ./Libraries/iLLD/TC27D/Tricore/Iom/Driver/IMU_Driver.d ./Libraries/iLLD/TC27D/Tricore/Iom/Driver/IMU_Driver.o ./Libraries/iLLD/TC27D/Tricore/Iom/Driver/IfxIom_Driver.d ./Libraries/iLLD/TC27D/Tricore/Iom/Driver/IfxIom_Driver.o ./Libraries/iLLD/TC27D/Tricore/Iom/Driver/MadgwickAHRS.d ./Libraries/iLLD/TC27D/Tricore/Iom/Driver/MadgwickAHRS.o ./Libraries/iLLD/TC27D/Tricore/Iom/Driver/Ultra_Driver.d ./Libraries/iLLD/TC27D/Tricore/Iom/Driver/Ultra_Driver.o ./Libraries/iLLD/TC27D/Tricore/Iom/Driver/ert_main.d ./Libraries/iLLD/TC27D/Tricore/Iom/Driver/ert_main.o ./Libraries/iLLD/TC27D/Tricore/Iom/Driver/vel_filter.d ./Libraries/iLLD/TC27D/Tricore/Iom/Driver/vel_filter.o

.PHONY: clean-Libraries-2f-iLLD-2f-TC27D-2f-Tricore-2f-Iom-2f-Driver

