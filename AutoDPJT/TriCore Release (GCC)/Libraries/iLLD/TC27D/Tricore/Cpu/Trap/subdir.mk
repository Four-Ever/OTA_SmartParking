################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Libraries/iLLD/TC27D/Tricore/Cpu/Trap/IfxCpu_Trap.c 

C_DEPS += \
./Libraries/iLLD/TC27D/Tricore/Cpu/Trap/IfxCpu_Trap.d 

OBJS += \
./Libraries/iLLD/TC27D/Tricore/Cpu/Trap/IfxCpu_Trap.o 


# Each subdirectory must supply rules for building sources it contributes
Libraries/iLLD/TC27D/Tricore/Cpu/Trap/%.o: ../Libraries/iLLD/TC27D/Tricore/Cpu/Trap/%.c Libraries/iLLD/TC27D/Tricore/Cpu/Trap/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: AURIX GCC Compiler'
	tricore-elf-gcc -std=c99 "@C:/Users/USER/AURIX_PJT/AutoDPJT/TriCore Release (GCC)/AURIX_GCC_Compiler-Include_paths__-I_.opt" -O3 -Wall -c -fmessage-length=0 -fno-common -fstrict-volatile-bitfields -fdata-sections -ffunction-sections -mtc161 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-Libraries-2f-iLLD-2f-TC27D-2f-Tricore-2f-Cpu-2f-Trap

clean-Libraries-2f-iLLD-2f-TC27D-2f-Tricore-2f-Cpu-2f-Trap:
	-$(RM) ./Libraries/iLLD/TC27D/Tricore/Cpu/Trap/IfxCpu_Trap.d ./Libraries/iLLD/TC27D/Tricore/Cpu/Trap/IfxCpu_Trap.o

.PHONY: clean-Libraries-2f-iLLD-2f-TC27D-2f-Tricore-2f-Cpu-2f-Trap

