################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
"../src/ASCLIN_Shell_UART.c" \
"../src/Driver_Stm.c" \
"../src/EncMotor.c" \
"../src/GTM_ATOM_PWM.c" \
"../src/Logger.c" \
"../src/OurCan.c" \
"../src/OurCan_message.c" \
"../src/PID_CON.c" \
"../src/STM_Interrupt.c" \
"../src/Ultra_Driver.c" \
"../src/UpadateInputs.c" \
"../src/atan2.c" \
"../src/decision_stateflow.c" \
"../src/gitstanley.c" \
"../src/gitstanley_initialize.c" \
"../src/gitstanley_terminate.c" \
"../src/norm.c" \
"../src/rtGetInf.c" \
"../src/rtGetNaN.c" \
"../src/rt_nonfinite.c" \
"../src/servo.c" \
"../src/wrapToPi.c" 

COMPILED_SRCS += \
"src/ASCLIN_Shell_UART.src" \
"src/Driver_Stm.src" \
"src/EncMotor.src" \
"src/GTM_ATOM_PWM.src" \
"src/Logger.src" \
"src/OurCan.src" \
"src/OurCan_message.src" \
"src/PID_CON.src" \
"src/STM_Interrupt.src" \
"src/Ultra_Driver.src" \
"src/UpadateInputs.src" \
"src/atan2.src" \
"src/decision_stateflow.src" \
"src/gitstanley.src" \
"src/gitstanley_initialize.src" \
"src/gitstanley_terminate.src" \
"src/norm.src" \
"src/rtGetInf.src" \
"src/rtGetNaN.src" \
"src/rt_nonfinite.src" \
"src/servo.src" \
"src/wrapToPi.src" 

C_DEPS += \
"./src/ASCLIN_Shell_UART.d" \
"./src/Driver_Stm.d" \
"./src/EncMotor.d" \
"./src/GTM_ATOM_PWM.d" \
"./src/Logger.d" \
"./src/OurCan.d" \
"./src/OurCan_message.d" \
"./src/PID_CON.d" \
"./src/STM_Interrupt.d" \
"./src/Ultra_Driver.d" \
"./src/UpadateInputs.d" \
"./src/atan2.d" \
"./src/decision_stateflow.d" \
"./src/gitstanley.d" \
"./src/gitstanley_initialize.d" \
"./src/gitstanley_terminate.d" \
"./src/norm.d" \
"./src/rtGetInf.d" \
"./src/rtGetNaN.d" \
"./src/rt_nonfinite.d" \
"./src/servo.d" \
"./src/wrapToPi.d" 

OBJS += \
"src/ASCLIN_Shell_UART.o" \
"src/Driver_Stm.o" \
"src/EncMotor.o" \
"src/GTM_ATOM_PWM.o" \
"src/Logger.o" \
"src/OurCan.o" \
"src/OurCan_message.o" \
"src/PID_CON.o" \
"src/STM_Interrupt.o" \
"src/Ultra_Driver.o" \
"src/UpadateInputs.o" \
"src/atan2.o" \
"src/decision_stateflow.o" \
"src/gitstanley.o" \
"src/gitstanley_initialize.o" \
"src/gitstanley_terminate.o" \
"src/norm.o" \
"src/rtGetInf.o" \
"src/rtGetNaN.o" \
"src/rt_nonfinite.o" \
"src/servo.o" \
"src/wrapToPi.o" 


# Each subdirectory must supply rules for building sources it contributes
"src/ASCLIN_Shell_UART.src":"../src/ASCLIN_Shell_UART.c" "src/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc27xd "-fC:/Users/USER/AURIX-v1.10.6-workspace/02_10motor/TriCore Debug (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"src/ASCLIN_Shell_UART.o":"src/ASCLIN_Shell_UART.src" "src/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"src/Driver_Stm.src":"../src/Driver_Stm.c" "src/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc27xd "-fC:/Users/USER/AURIX-v1.10.6-workspace/02_10motor/TriCore Debug (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"src/Driver_Stm.o":"src/Driver_Stm.src" "src/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"src/EncMotor.src":"../src/EncMotor.c" "src/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc27xd "-fC:/Users/USER/AURIX-v1.10.6-workspace/02_10motor/TriCore Debug (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"src/EncMotor.o":"src/EncMotor.src" "src/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"src/GTM_ATOM_PWM.src":"../src/GTM_ATOM_PWM.c" "src/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc27xd "-fC:/Users/USER/AURIX-v1.10.6-workspace/02_10motor/TriCore Debug (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"src/GTM_ATOM_PWM.o":"src/GTM_ATOM_PWM.src" "src/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"src/Logger.src":"../src/Logger.c" "src/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc27xd "-fC:/Users/USER/AURIX-v1.10.6-workspace/02_10motor/TriCore Debug (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"src/Logger.o":"src/Logger.src" "src/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"src/OurCan.src":"../src/OurCan.c" "src/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc27xd "-fC:/Users/USER/AURIX-v1.10.6-workspace/02_10motor/TriCore Debug (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"src/OurCan.o":"src/OurCan.src" "src/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"src/OurCan_message.src":"../src/OurCan_message.c" "src/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc27xd "-fC:/Users/USER/AURIX-v1.10.6-workspace/02_10motor/TriCore Debug (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"src/OurCan_message.o":"src/OurCan_message.src" "src/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"src/PID_CON.src":"../src/PID_CON.c" "src/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc27xd "-fC:/Users/USER/AURIX-v1.10.6-workspace/02_10motor/TriCore Debug (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"src/PID_CON.o":"src/PID_CON.src" "src/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"src/STM_Interrupt.src":"../src/STM_Interrupt.c" "src/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc27xd "-fC:/Users/USER/AURIX-v1.10.6-workspace/02_10motor/TriCore Debug (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"src/STM_Interrupt.o":"src/STM_Interrupt.src" "src/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"src/Ultra_Driver.src":"../src/Ultra_Driver.c" "src/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc27xd "-fC:/Users/USER/AURIX-v1.10.6-workspace/02_10motor/TriCore Debug (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"src/Ultra_Driver.o":"src/Ultra_Driver.src" "src/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"src/UpadateInputs.src":"../src/UpadateInputs.c" "src/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc27xd "-fC:/Users/USER/AURIX-v1.10.6-workspace/02_10motor/TriCore Debug (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"src/UpadateInputs.o":"src/UpadateInputs.src" "src/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"src/atan2.src":"../src/atan2.c" "src/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc27xd "-fC:/Users/USER/AURIX-v1.10.6-workspace/02_10motor/TriCore Debug (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"src/atan2.o":"src/atan2.src" "src/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"src/decision_stateflow.src":"../src/decision_stateflow.c" "src/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc27xd "-fC:/Users/USER/AURIX-v1.10.6-workspace/02_10motor/TriCore Debug (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"src/decision_stateflow.o":"src/decision_stateflow.src" "src/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"src/gitstanley.src":"../src/gitstanley.c" "src/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc27xd "-fC:/Users/USER/AURIX-v1.10.6-workspace/02_10motor/TriCore Debug (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"src/gitstanley.o":"src/gitstanley.src" "src/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"src/gitstanley_initialize.src":"../src/gitstanley_initialize.c" "src/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc27xd "-fC:/Users/USER/AURIX-v1.10.6-workspace/02_10motor/TriCore Debug (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"src/gitstanley_initialize.o":"src/gitstanley_initialize.src" "src/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"src/gitstanley_terminate.src":"../src/gitstanley_terminate.c" "src/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc27xd "-fC:/Users/USER/AURIX-v1.10.6-workspace/02_10motor/TriCore Debug (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"src/gitstanley_terminate.o":"src/gitstanley_terminate.src" "src/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"src/norm.src":"../src/norm.c" "src/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc27xd "-fC:/Users/USER/AURIX-v1.10.6-workspace/02_10motor/TriCore Debug (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"src/norm.o":"src/norm.src" "src/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"src/rtGetInf.src":"../src/rtGetInf.c" "src/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc27xd "-fC:/Users/USER/AURIX-v1.10.6-workspace/02_10motor/TriCore Debug (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"src/rtGetInf.o":"src/rtGetInf.src" "src/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"src/rtGetNaN.src":"../src/rtGetNaN.c" "src/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc27xd "-fC:/Users/USER/AURIX-v1.10.6-workspace/02_10motor/TriCore Debug (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"src/rtGetNaN.o":"src/rtGetNaN.src" "src/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"src/rt_nonfinite.src":"../src/rt_nonfinite.c" "src/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc27xd "-fC:/Users/USER/AURIX-v1.10.6-workspace/02_10motor/TriCore Debug (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"src/rt_nonfinite.o":"src/rt_nonfinite.src" "src/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"src/servo.src":"../src/servo.c" "src/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc27xd "-fC:/Users/USER/AURIX-v1.10.6-workspace/02_10motor/TriCore Debug (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"src/servo.o":"src/servo.src" "src/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"src/wrapToPi.src":"../src/wrapToPi.c" "src/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc27xd "-fC:/Users/USER/AURIX-v1.10.6-workspace/02_10motor/TriCore Debug (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"src/wrapToPi.o":"src/wrapToPi.src" "src/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-src

clean-src:
	-$(RM) ./src/ASCLIN_Shell_UART.d ./src/ASCLIN_Shell_UART.o ./src/ASCLIN_Shell_UART.src ./src/Driver_Stm.d ./src/Driver_Stm.o ./src/Driver_Stm.src ./src/EncMotor.d ./src/EncMotor.o ./src/EncMotor.src ./src/GTM_ATOM_PWM.d ./src/GTM_ATOM_PWM.o ./src/GTM_ATOM_PWM.src ./src/Logger.d ./src/Logger.o ./src/Logger.src ./src/OurCan.d ./src/OurCan.o ./src/OurCan.src ./src/OurCan_message.d ./src/OurCan_message.o ./src/OurCan_message.src ./src/PID_CON.d ./src/PID_CON.o ./src/PID_CON.src ./src/STM_Interrupt.d ./src/STM_Interrupt.o ./src/STM_Interrupt.src ./src/Ultra_Driver.d ./src/Ultra_Driver.o ./src/Ultra_Driver.src ./src/UpadateInputs.d ./src/UpadateInputs.o ./src/UpadateInputs.src ./src/atan2.d ./src/atan2.o ./src/atan2.src ./src/decision_stateflow.d ./src/decision_stateflow.o ./src/decision_stateflow.src ./src/gitstanley.d ./src/gitstanley.o ./src/gitstanley.src ./src/gitstanley_initialize.d ./src/gitstanley_initialize.o ./src/gitstanley_initialize.src ./src/gitstanley_terminate.d ./src/gitstanley_terminate.o ./src/gitstanley_terminate.src ./src/norm.d ./src/norm.o ./src/norm.src ./src/rtGetInf.d ./src/rtGetInf.o ./src/rtGetInf.src ./src/rtGetNaN.d ./src/rtGetNaN.o ./src/rtGetNaN.src ./src/rt_nonfinite.d ./src/rt_nonfinite.o ./src/rt_nonfinite.src ./src/servo.d ./src/servo.o ./src/servo.src ./src/wrapToPi.d ./src/wrapToPi.o ./src/wrapToPi.src

.PHONY: clean-src

