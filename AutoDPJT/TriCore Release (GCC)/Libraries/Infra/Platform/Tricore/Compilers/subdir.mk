################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Libraries/Infra/Platform/Tricore/Compilers/CompilerDcc.c \
../Libraries/Infra/Platform/Tricore/Compilers/CompilerGcc.c \
../Libraries/Infra/Platform/Tricore/Compilers/CompilerGhs.c \
../Libraries/Infra/Platform/Tricore/Compilers/CompilerGnuc.c \
../Libraries/Infra/Platform/Tricore/Compilers/CompilerTasking.c 

C_DEPS += \
./Libraries/Infra/Platform/Tricore/Compilers/CompilerDcc.d \
./Libraries/Infra/Platform/Tricore/Compilers/CompilerGcc.d \
./Libraries/Infra/Platform/Tricore/Compilers/CompilerGhs.d \
./Libraries/Infra/Platform/Tricore/Compilers/CompilerGnuc.d \
./Libraries/Infra/Platform/Tricore/Compilers/CompilerTasking.d 

OBJS += \
./Libraries/Infra/Platform/Tricore/Compilers/CompilerDcc.o \
./Libraries/Infra/Platform/Tricore/Compilers/CompilerGcc.o \
./Libraries/Infra/Platform/Tricore/Compilers/CompilerGhs.o \
./Libraries/Infra/Platform/Tricore/Compilers/CompilerGnuc.o \
./Libraries/Infra/Platform/Tricore/Compilers/CompilerTasking.o 


# Each subdirectory must supply rules for building sources it contributes
Libraries/Infra/Platform/Tricore/Compilers/%.o: ../Libraries/Infra/Platform/Tricore/Compilers/%.c Libraries/Infra/Platform/Tricore/Compilers/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: AURIX GCC Compiler'
	tricore-elf-gcc -std=c99 "@C:/Users/USER/AURIX_PJT/AutoDPJT/TriCore Release (GCC)/AURIX_GCC_Compiler-Include_paths__-I_.opt" -O3 -Wall -c -fmessage-length=0 -fno-common -fstrict-volatile-bitfields -fdata-sections -ffunction-sections -mtc161 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-Libraries-2f-Infra-2f-Platform-2f-Tricore-2f-Compilers

clean-Libraries-2f-Infra-2f-Platform-2f-Tricore-2f-Compilers:
	-$(RM) ./Libraries/Infra/Platform/Tricore/Compilers/CompilerDcc.d ./Libraries/Infra/Platform/Tricore/Compilers/CompilerDcc.o ./Libraries/Infra/Platform/Tricore/Compilers/CompilerGcc.d ./Libraries/Infra/Platform/Tricore/Compilers/CompilerGcc.o ./Libraries/Infra/Platform/Tricore/Compilers/CompilerGhs.d ./Libraries/Infra/Platform/Tricore/Compilers/CompilerGhs.o ./Libraries/Infra/Platform/Tricore/Compilers/CompilerGnuc.d ./Libraries/Infra/Platform/Tricore/Compilers/CompilerGnuc.o ./Libraries/Infra/Platform/Tricore/Compilers/CompilerTasking.d ./Libraries/Infra/Platform/Tricore/Compilers/CompilerTasking.o

.PHONY: clean-Libraries-2f-Infra-2f-Platform-2f-Tricore-2f-Compilers

