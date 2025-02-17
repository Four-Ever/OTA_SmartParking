################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
"../Libraries/iLLD/TC27D/Tricore/Psi5/Std/IfxPsi5.c" 

COMPILED_SRCS += \
"Libraries/iLLD/TC27D/Tricore/Psi5/Std/IfxPsi5.src" 

C_DEPS += \
"./Libraries/iLLD/TC27D/Tricore/Psi5/Std/IfxPsi5.d" 

OBJS += \
"Libraries/iLLD/TC27D/Tricore/Psi5/Std/IfxPsi5.o" 


# Each subdirectory must supply rules for building sources it contributes
"Libraries/iLLD/TC27D/Tricore/Psi5/Std/IfxPsi5.src":"../Libraries/iLLD/TC27D/Tricore/Psi5/Std/IfxPsi5.c" "Libraries/iLLD/TC27D/Tricore/Psi5/Std/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc27xd "-fC:/Users/USER/AURIX_PJT/AutoDPJT/TriCore Debug (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"Libraries/iLLD/TC27D/Tricore/Psi5/Std/IfxPsi5.o":"Libraries/iLLD/TC27D/Tricore/Psi5/Std/IfxPsi5.src" "Libraries/iLLD/TC27D/Tricore/Psi5/Std/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-Libraries-2f-iLLD-2f-TC27D-2f-Tricore-2f-Psi5-2f-Std

clean-Libraries-2f-iLLD-2f-TC27D-2f-Tricore-2f-Psi5-2f-Std:
	-$(RM) ./Libraries/iLLD/TC27D/Tricore/Psi5/Std/IfxPsi5.d ./Libraries/iLLD/TC27D/Tricore/Psi5/Std/IfxPsi5.o ./Libraries/iLLD/TC27D/Tricore/Psi5/Std/IfxPsi5.src

.PHONY: clean-Libraries-2f-iLLD-2f-TC27D-2f-Tricore-2f-Psi5-2f-Std

