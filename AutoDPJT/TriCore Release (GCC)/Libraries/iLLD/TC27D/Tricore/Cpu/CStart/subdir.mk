################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Libraries/iLLD/TC27D/Tricore/Cpu/CStart/IfxCpu_CStart0.c \
../Libraries/iLLD/TC27D/Tricore/Cpu/CStart/IfxCpu_CStart1.c \
../Libraries/iLLD/TC27D/Tricore/Cpu/CStart/IfxCpu_CStart2.c 

C_DEPS += \
./Libraries/iLLD/TC27D/Tricore/Cpu/CStart/IfxCpu_CStart0.d \
./Libraries/iLLD/TC27D/Tricore/Cpu/CStart/IfxCpu_CStart1.d \
./Libraries/iLLD/TC27D/Tricore/Cpu/CStart/IfxCpu_CStart2.d 

OBJS += \
./Libraries/iLLD/TC27D/Tricore/Cpu/CStart/IfxCpu_CStart0.o \
./Libraries/iLLD/TC27D/Tricore/Cpu/CStart/IfxCpu_CStart1.o \
./Libraries/iLLD/TC27D/Tricore/Cpu/CStart/IfxCpu_CStart2.o 


# Each subdirectory must supply rules for building sources it contributes
Libraries/iLLD/TC27D/Tricore/Cpu/CStart/%.o: ../Libraries/iLLD/TC27D/Tricore/Cpu/CStart/%.c Libraries/iLLD/TC27D/Tricore/Cpu/CStart/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: AURIX GCC Compiler'
	tricore-elf-gcc -std=c99 "@C:/Users/USER/AURIX_PJT/AutoDPJT/TriCore Release (GCC)/AURIX_GCC_Compiler-Include_paths__-I_.opt" -O3 -Wall -c -fmessage-length=0 -fno-common -fstrict-volatile-bitfields -fdata-sections -ffunction-sections -mtc161 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-Libraries-2f-iLLD-2f-TC27D-2f-Tricore-2f-Cpu-2f-CStart

clean-Libraries-2f-iLLD-2f-TC27D-2f-Tricore-2f-Cpu-2f-CStart:
	-$(RM) ./Libraries/iLLD/TC27D/Tricore/Cpu/CStart/IfxCpu_CStart0.d ./Libraries/iLLD/TC27D/Tricore/Cpu/CStart/IfxCpu_CStart0.o ./Libraries/iLLD/TC27D/Tricore/Cpu/CStart/IfxCpu_CStart1.d ./Libraries/iLLD/TC27D/Tricore/Cpu/CStart/IfxCpu_CStart1.o ./Libraries/iLLD/TC27D/Tricore/Cpu/CStart/IfxCpu_CStart2.d ./Libraries/iLLD/TC27D/Tricore/Cpu/CStart/IfxCpu_CStart2.o

.PHONY: clean-Libraries-2f-iLLD-2f-TC27D-2f-Tricore-2f-Cpu-2f-CStart

