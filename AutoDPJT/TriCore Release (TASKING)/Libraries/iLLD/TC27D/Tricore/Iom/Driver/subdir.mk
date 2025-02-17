################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
"../Libraries/iLLD/TC27D/Tricore/Iom/Driver/ASCLIN_Shell_UART.c" \
"../Libraries/iLLD/TC27D/Tricore/Iom/Driver/IMU_Driver.c" \
"../Libraries/iLLD/TC27D/Tricore/Iom/Driver/IfxIom_Driver.c" \
"../Libraries/iLLD/TC27D/Tricore/Iom/Driver/MadgwickAHRS.c" \
"../Libraries/iLLD/TC27D/Tricore/Iom/Driver/Ultra_Driver.c" 

COMPILED_SRCS += \
"Libraries/iLLD/TC27D/Tricore/Iom/Driver/ASCLIN_Shell_UART.src" \
"Libraries/iLLD/TC27D/Tricore/Iom/Driver/IMU_Driver.src" \
"Libraries/iLLD/TC27D/Tricore/Iom/Driver/IfxIom_Driver.src" \
"Libraries/iLLD/TC27D/Tricore/Iom/Driver/MadgwickAHRS.src" \
"Libraries/iLLD/TC27D/Tricore/Iom/Driver/Ultra_Driver.src" 

C_DEPS += \
"./Libraries/iLLD/TC27D/Tricore/Iom/Driver/ASCLIN_Shell_UART.d" \
"./Libraries/iLLD/TC27D/Tricore/Iom/Driver/IMU_Driver.d" \
"./Libraries/iLLD/TC27D/Tricore/Iom/Driver/IfxIom_Driver.d" \
"./Libraries/iLLD/TC27D/Tricore/Iom/Driver/MadgwickAHRS.d" \
"./Libraries/iLLD/TC27D/Tricore/Iom/Driver/Ultra_Driver.d" 

OBJS += \
"Libraries/iLLD/TC27D/Tricore/Iom/Driver/ASCLIN_Shell_UART.o" \
"Libraries/iLLD/TC27D/Tricore/Iom/Driver/IMU_Driver.o" \
"Libraries/iLLD/TC27D/Tricore/Iom/Driver/IfxIom_Driver.o" \
"Libraries/iLLD/TC27D/Tricore/Iom/Driver/MadgwickAHRS.o" \
"Libraries/iLLD/TC27D/Tricore/Iom/Driver/Ultra_Driver.o" 


# Each subdirectory must supply rules for building sources it contributes
"Libraries/iLLD/TC27D/Tricore/Iom/Driver/ASCLIN_Shell_UART.src":"../Libraries/iLLD/TC27D/Tricore/Iom/Driver/ASCLIN_Shell_UART.c" "Libraries/iLLD/TC27D/Tricore/Iom/Driver/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc27xd "-fC:/Users/USER/AURIX_PJT/AutoDPJT/TriCore Release (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"Libraries/iLLD/TC27D/Tricore/Iom/Driver/ASCLIN_Shell_UART.o":"Libraries/iLLD/TC27D/Tricore/Iom/Driver/ASCLIN_Shell_UART.src" "Libraries/iLLD/TC27D/Tricore/Iom/Driver/subdir.mk"
	astc -I"/AutoDPJT/Libraries/iLLD/TC27D/Tricore/I2c/I2c" -I"C:\Users\USER\AURIX_PJT\AutoDPJT\Libraries\iLLD\TC27D\Tricore\I2c\I2c" -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"Libraries/iLLD/TC27D/Tricore/Iom/Driver/IMU_Driver.src":"../Libraries/iLLD/TC27D/Tricore/Iom/Driver/IMU_Driver.c" "Libraries/iLLD/TC27D/Tricore/Iom/Driver/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc27xd "-fC:/Users/USER/AURIX_PJT/AutoDPJT/TriCore Release (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"Libraries/iLLD/TC27D/Tricore/Iom/Driver/IMU_Driver.o":"Libraries/iLLD/TC27D/Tricore/Iom/Driver/IMU_Driver.src" "Libraries/iLLD/TC27D/Tricore/Iom/Driver/subdir.mk"
	astc -I"/AutoDPJT/Libraries/iLLD/TC27D/Tricore/I2c/I2c" -I"C:\Users\USER\AURIX_PJT\AutoDPJT\Libraries\iLLD\TC27D\Tricore\I2c\I2c" -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"Libraries/iLLD/TC27D/Tricore/Iom/Driver/IfxIom_Driver.src":"../Libraries/iLLD/TC27D/Tricore/Iom/Driver/IfxIom_Driver.c" "Libraries/iLLD/TC27D/Tricore/Iom/Driver/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc27xd "-fC:/Users/USER/AURIX_PJT/AutoDPJT/TriCore Release (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"Libraries/iLLD/TC27D/Tricore/Iom/Driver/IfxIom_Driver.o":"Libraries/iLLD/TC27D/Tricore/Iom/Driver/IfxIom_Driver.src" "Libraries/iLLD/TC27D/Tricore/Iom/Driver/subdir.mk"
	astc -I"/AutoDPJT/Libraries/iLLD/TC27D/Tricore/I2c/I2c" -I"C:\Users\USER\AURIX_PJT\AutoDPJT\Libraries\iLLD\TC27D\Tricore\I2c\I2c" -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"Libraries/iLLD/TC27D/Tricore/Iom/Driver/MadgwickAHRS.src":"../Libraries/iLLD/TC27D/Tricore/Iom/Driver/MadgwickAHRS.c" "Libraries/iLLD/TC27D/Tricore/Iom/Driver/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc27xd "-fC:/Users/USER/AURIX_PJT/AutoDPJT/TriCore Release (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"Libraries/iLLD/TC27D/Tricore/Iom/Driver/MadgwickAHRS.o":"Libraries/iLLD/TC27D/Tricore/Iom/Driver/MadgwickAHRS.src" "Libraries/iLLD/TC27D/Tricore/Iom/Driver/subdir.mk"
	astc -I"/AutoDPJT/Libraries/iLLD/TC27D/Tricore/I2c/I2c" -I"C:\Users\USER\AURIX_PJT\AutoDPJT\Libraries\iLLD\TC27D\Tricore\I2c\I2c" -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"Libraries/iLLD/TC27D/Tricore/Iom/Driver/Ultra_Driver.src":"../Libraries/iLLD/TC27D/Tricore/Iom/Driver/Ultra_Driver.c" "Libraries/iLLD/TC27D/Tricore/Iom/Driver/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc27xd "-fC:/Users/USER/AURIX_PJT/AutoDPJT/TriCore Release (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"Libraries/iLLD/TC27D/Tricore/Iom/Driver/Ultra_Driver.o":"Libraries/iLLD/TC27D/Tricore/Iom/Driver/Ultra_Driver.src" "Libraries/iLLD/TC27D/Tricore/Iom/Driver/subdir.mk"
	astc -I"/AutoDPJT/Libraries/iLLD/TC27D/Tricore/I2c/I2c" -I"C:\Users\USER\AURIX_PJT\AutoDPJT\Libraries\iLLD\TC27D\Tricore\I2c\I2c" -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-Libraries-2f-iLLD-2f-TC27D-2f-Tricore-2f-Iom-2f-Driver

clean-Libraries-2f-iLLD-2f-TC27D-2f-Tricore-2f-Iom-2f-Driver:
	-$(RM) ./Libraries/iLLD/TC27D/Tricore/Iom/Driver/ASCLIN_Shell_UART.d ./Libraries/iLLD/TC27D/Tricore/Iom/Driver/ASCLIN_Shell_UART.o ./Libraries/iLLD/TC27D/Tricore/Iom/Driver/ASCLIN_Shell_UART.src ./Libraries/iLLD/TC27D/Tricore/Iom/Driver/IMU_Driver.d ./Libraries/iLLD/TC27D/Tricore/Iom/Driver/IMU_Driver.o ./Libraries/iLLD/TC27D/Tricore/Iom/Driver/IMU_Driver.src ./Libraries/iLLD/TC27D/Tricore/Iom/Driver/IfxIom_Driver.d ./Libraries/iLLD/TC27D/Tricore/Iom/Driver/IfxIom_Driver.o ./Libraries/iLLD/TC27D/Tricore/Iom/Driver/IfxIom_Driver.src ./Libraries/iLLD/TC27D/Tricore/Iom/Driver/MadgwickAHRS.d ./Libraries/iLLD/TC27D/Tricore/Iom/Driver/MadgwickAHRS.o ./Libraries/iLLD/TC27D/Tricore/Iom/Driver/MadgwickAHRS.src ./Libraries/iLLD/TC27D/Tricore/Iom/Driver/Ultra_Driver.d ./Libraries/iLLD/TC27D/Tricore/Iom/Driver/Ultra_Driver.o ./Libraries/iLLD/TC27D/Tricore/Iom/Driver/Ultra_Driver.src

.PHONY: clean-Libraries-2f-iLLD-2f-TC27D-2f-Tricore-2f-Iom-2f-Driver

