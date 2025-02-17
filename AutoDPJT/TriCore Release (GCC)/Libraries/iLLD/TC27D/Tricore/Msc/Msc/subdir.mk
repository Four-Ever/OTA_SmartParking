################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Libraries/iLLD/TC27D/Tricore/Msc/Msc/IfxMsc_Msc.c 

C_DEPS += \
./Libraries/iLLD/TC27D/Tricore/Msc/Msc/IfxMsc_Msc.d 

OBJS += \
./Libraries/iLLD/TC27D/Tricore/Msc/Msc/IfxMsc_Msc.o 


# Each subdirectory must supply rules for building sources it contributes
Libraries/iLLD/TC27D/Tricore/Msc/Msc/%.o: ../Libraries/iLLD/TC27D/Tricore/Msc/Msc/%.c Libraries/iLLD/TC27D/Tricore/Msc/Msc/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: AURIX GCC Compiler'
	tricore-elf-gcc -std=c99 "@C:/Users/USER/AURIX_PJT/AutoDPJT/TriCore Release (GCC)/AURIX_GCC_Compiler-Include_paths__-I_.opt" -O3 -Wall -c -fmessage-length=0 -fno-common -fstrict-volatile-bitfields -fdata-sections -ffunction-sections -mtc161 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-Libraries-2f-iLLD-2f-TC27D-2f-Tricore-2f-Msc-2f-Msc

clean-Libraries-2f-iLLD-2f-TC27D-2f-Tricore-2f-Msc-2f-Msc:
	-$(RM) ./Libraries/iLLD/TC27D/Tricore/Msc/Msc/IfxMsc_Msc.d ./Libraries/iLLD/TC27D/Tricore/Msc/Msc/IfxMsc_Msc.o

.PHONY: clean-Libraries-2f-iLLD-2f-TC27D-2f-Tricore-2f-Msc-2f-Msc

