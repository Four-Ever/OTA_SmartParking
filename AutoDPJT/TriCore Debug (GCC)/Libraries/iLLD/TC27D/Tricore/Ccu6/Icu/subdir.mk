################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Libraries/iLLD/TC27D/Tricore/Ccu6/Icu/IfxCcu6_Icu.c 

C_DEPS += \
./Libraries/iLLD/TC27D/Tricore/Ccu6/Icu/IfxCcu6_Icu.d 

OBJS += \
./Libraries/iLLD/TC27D/Tricore/Ccu6/Icu/IfxCcu6_Icu.o 


# Each subdirectory must supply rules for building sources it contributes
Libraries/iLLD/TC27D/Tricore/Ccu6/Icu/%.o: ../Libraries/iLLD/TC27D/Tricore/Ccu6/Icu/%.c Libraries/iLLD/TC27D/Tricore/Ccu6/Icu/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: AURIX GCC Compiler'
	tricore-elf-gcc -std=c99 "@C:/Users/USER/AURIX_PJT/AutoDPJT/TriCore Debug (GCC)/AURIX_GCC_Compiler-Include_paths__-I_.opt" -Og -g3 -gdwarf-3 -Wall -c -fmessage-length=0 -fno-common -fstrict-volatile-bitfields -fdata-sections -ffunction-sections -mtc161 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-Libraries-2f-iLLD-2f-TC27D-2f-Tricore-2f-Ccu6-2f-Icu

clean-Libraries-2f-iLLD-2f-TC27D-2f-Tricore-2f-Ccu6-2f-Icu:
	-$(RM) ./Libraries/iLLD/TC27D/Tricore/Ccu6/Icu/IfxCcu6_Icu.d ./Libraries/iLLD/TC27D/Tricore/Ccu6/Icu/IfxCcu6_Icu.o

.PHONY: clean-Libraries-2f-iLLD-2f-TC27D-2f-Tricore-2f-Ccu6-2f-Icu

