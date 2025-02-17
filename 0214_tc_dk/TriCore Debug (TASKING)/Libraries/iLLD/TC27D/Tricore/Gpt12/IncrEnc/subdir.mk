################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
"../Libraries/iLLD/TC27D/Tricore/Gpt12/IncrEnc/IfxGpt12_IncrEnc.c" 

COMPILED_SRCS += \
"Libraries/iLLD/TC27D/Tricore/Gpt12/IncrEnc/IfxGpt12_IncrEnc.src" 

C_DEPS += \
"./Libraries/iLLD/TC27D/Tricore/Gpt12/IncrEnc/IfxGpt12_IncrEnc.d" 

OBJS += \
"Libraries/iLLD/TC27D/Tricore/Gpt12/IncrEnc/IfxGpt12_IncrEnc.o" 


# Each subdirectory must supply rules for building sources it contributes
"Libraries/iLLD/TC27D/Tricore/Gpt12/IncrEnc/IfxGpt12_IncrEnc.src":"../Libraries/iLLD/TC27D/Tricore/Gpt12/IncrEnc/IfxGpt12_IncrEnc.c" "Libraries/iLLD/TC27D/Tricore/Gpt12/IncrEnc/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc27xd "-fC:/Users/USER/AURIX-v1.10.6-workspace/02_10motor/TriCore Debug (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"Libraries/iLLD/TC27D/Tricore/Gpt12/IncrEnc/IfxGpt12_IncrEnc.o":"Libraries/iLLD/TC27D/Tricore/Gpt12/IncrEnc/IfxGpt12_IncrEnc.src" "Libraries/iLLD/TC27D/Tricore/Gpt12/IncrEnc/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-Libraries-2f-iLLD-2f-TC27D-2f-Tricore-2f-Gpt12-2f-IncrEnc

clean-Libraries-2f-iLLD-2f-TC27D-2f-Tricore-2f-Gpt12-2f-IncrEnc:
	-$(RM) ./Libraries/iLLD/TC27D/Tricore/Gpt12/IncrEnc/IfxGpt12_IncrEnc.d ./Libraries/iLLD/TC27D/Tricore/Gpt12/IncrEnc/IfxGpt12_IncrEnc.o ./Libraries/iLLD/TC27D/Tricore/Gpt12/IncrEnc/IfxGpt12_IncrEnc.src

.PHONY: clean-Libraries-2f-iLLD-2f-TC27D-2f-Tricore-2f-Gpt12-2f-IncrEnc

