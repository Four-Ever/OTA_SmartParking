################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
"../Libraries/iLLD/TC27D/Tricore/Cif/Cam/IfxCif_Cam.c" 

COMPILED_SRCS += \
"Libraries/iLLD/TC27D/Tricore/Cif/Cam/IfxCif_Cam.src" 

C_DEPS += \
"./Libraries/iLLD/TC27D/Tricore/Cif/Cam/IfxCif_Cam.d" 

OBJS += \
"Libraries/iLLD/TC27D/Tricore/Cif/Cam/IfxCif_Cam.o" 


# Each subdirectory must supply rules for building sources it contributes
"Libraries/iLLD/TC27D/Tricore/Cif/Cam/IfxCif_Cam.src":"../Libraries/iLLD/TC27D/Tricore/Cif/Cam/IfxCif_Cam.c" "Libraries/iLLD/TC27D/Tricore/Cif/Cam/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc27xd "-fC:/Users/USER/AURIX_PJT/AutoDPJT/TriCore Release (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"Libraries/iLLD/TC27D/Tricore/Cif/Cam/IfxCif_Cam.o":"Libraries/iLLD/TC27D/Tricore/Cif/Cam/IfxCif_Cam.src" "Libraries/iLLD/TC27D/Tricore/Cif/Cam/subdir.mk"
	astc -I"/AutoDPJT/Libraries/iLLD/TC27D/Tricore/I2c/I2c" -I"C:\Users\USER\AURIX_PJT\AutoDPJT\Libraries\iLLD\TC27D\Tricore\I2c\I2c" -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-Libraries-2f-iLLD-2f-TC27D-2f-Tricore-2f-Cif-2f-Cam

clean-Libraries-2f-iLLD-2f-TC27D-2f-Tricore-2f-Cif-2f-Cam:
	-$(RM) ./Libraries/iLLD/TC27D/Tricore/Cif/Cam/IfxCif_Cam.d ./Libraries/iLLD/TC27D/Tricore/Cif/Cam/IfxCif_Cam.o ./Libraries/iLLD/TC27D/Tricore/Cif/Cam/IfxCif_Cam.src

.PHONY: clean-Libraries-2f-iLLD-2f-TC27D-2f-Tricore-2f-Cif-2f-Cam

