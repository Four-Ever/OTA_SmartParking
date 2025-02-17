################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Libraries/iLLD/TC27D/Tricore/Gtm/Tim/In/IfxGtm_Tim_In.c 

C_DEPS += \
./Libraries/iLLD/TC27D/Tricore/Gtm/Tim/In/IfxGtm_Tim_In.d 

OBJS += \
./Libraries/iLLD/TC27D/Tricore/Gtm/Tim/In/IfxGtm_Tim_In.o 


# Each subdirectory must supply rules for building sources it contributes
Libraries/iLLD/TC27D/Tricore/Gtm/Tim/In/%.o: ../Libraries/iLLD/TC27D/Tricore/Gtm/Tim/In/%.c Libraries/iLLD/TC27D/Tricore/Gtm/Tim/In/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: AURIX GCC Compiler'
	tricore-elf-gcc -std=c99 "@C:/Users/USER/AURIX_PJT/AutoDPJT/TriCore Release (GCC)/AURIX_GCC_Compiler-Include_paths__-I_.opt" -O3 -Wall -c -fmessage-length=0 -fno-common -fstrict-volatile-bitfields -fdata-sections -ffunction-sections -mtc161 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-Libraries-2f-iLLD-2f-TC27D-2f-Tricore-2f-Gtm-2f-Tim-2f-In

clean-Libraries-2f-iLLD-2f-TC27D-2f-Tricore-2f-Gtm-2f-Tim-2f-In:
	-$(RM) ./Libraries/iLLD/TC27D/Tricore/Gtm/Tim/In/IfxGtm_Tim_In.d ./Libraries/iLLD/TC27D/Tricore/Gtm/Tim/In/IfxGtm_Tim_In.o

.PHONY: clean-Libraries-2f-iLLD-2f-TC27D-2f-Tricore-2f-Gtm-2f-Tim-2f-In

