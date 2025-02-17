################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
"../Libraries/iLLD/TC27D/Tricore/Hssl/Std/IfxHssl.c" 

COMPILED_SRCS += \
"Libraries/iLLD/TC27D/Tricore/Hssl/Std/IfxHssl.src" 

C_DEPS += \
"./Libraries/iLLD/TC27D/Tricore/Hssl/Std/IfxHssl.d" 

OBJS += \
"Libraries/iLLD/TC27D/Tricore/Hssl/Std/IfxHssl.o" 


# Each subdirectory must supply rules for building sources it contributes
"Libraries/iLLD/TC27D/Tricore/Hssl/Std/IfxHssl.src":"../Libraries/iLLD/TC27D/Tricore/Hssl/Std/IfxHssl.c" "Libraries/iLLD/TC27D/Tricore/Hssl/Std/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc27xd "-fC:/Users/USER/AURIX_PJT/AutoDPJT/TriCore Release (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"Libraries/iLLD/TC27D/Tricore/Hssl/Std/IfxHssl.o":"Libraries/iLLD/TC27D/Tricore/Hssl/Std/IfxHssl.src" "Libraries/iLLD/TC27D/Tricore/Hssl/Std/subdir.mk"
	astc -I"/AutoDPJT/Libraries/iLLD/TC27D/Tricore/I2c/I2c" -I"C:\Users\USER\AURIX_PJT\AutoDPJT\Libraries\iLLD\TC27D\Tricore\I2c\I2c" -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-Libraries-2f-iLLD-2f-TC27D-2f-Tricore-2f-Hssl-2f-Std

clean-Libraries-2f-iLLD-2f-TC27D-2f-Tricore-2f-Hssl-2f-Std:
	-$(RM) ./Libraries/iLLD/TC27D/Tricore/Hssl/Std/IfxHssl.d ./Libraries/iLLD/TC27D/Tricore/Hssl/Std/IfxHssl.o ./Libraries/iLLD/TC27D/Tricore/Hssl/Std/IfxHssl.src

.PHONY: clean-Libraries-2f-iLLD-2f-TC27D-2f-Tricore-2f-Hssl-2f-Std

