################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Libraries/iLLD/TC27D/Tricore/_PinMap/IfxAsclin_PinMap.c \
../Libraries/iLLD/TC27D/Tricore/_PinMap/IfxCcu6_PinMap.c \
../Libraries/iLLD/TC27D/Tricore/_PinMap/IfxCif_PinMap.c \
../Libraries/iLLD/TC27D/Tricore/_PinMap/IfxDsadc_PinMap.c \
../Libraries/iLLD/TC27D/Tricore/_PinMap/IfxEray_PinMap.c \
../Libraries/iLLD/TC27D/Tricore/_PinMap/IfxEth_PinMap.c \
../Libraries/iLLD/TC27D/Tricore/_PinMap/IfxGpt12_PinMap.c \
../Libraries/iLLD/TC27D/Tricore/_PinMap/IfxGtm_PinMap.c \
../Libraries/iLLD/TC27D/Tricore/_PinMap/IfxI2c_PinMap.c \
../Libraries/iLLD/TC27D/Tricore/_PinMap/IfxMsc_PinMap.c \
../Libraries/iLLD/TC27D/Tricore/_PinMap/IfxMultican_PinMap.c \
../Libraries/iLLD/TC27D/Tricore/_PinMap/IfxPort_PinMap.c \
../Libraries/iLLD/TC27D/Tricore/_PinMap/IfxPsi5_PinMap.c \
../Libraries/iLLD/TC27D/Tricore/_PinMap/IfxPsi5s_PinMap.c \
../Libraries/iLLD/TC27D/Tricore/_PinMap/IfxQspi_PinMap.c \
../Libraries/iLLD/TC27D/Tricore/_PinMap/IfxScu_PinMap.c \
../Libraries/iLLD/TC27D/Tricore/_PinMap/IfxSent_PinMap.c \
../Libraries/iLLD/TC27D/Tricore/_PinMap/IfxSmu_PinMap.c \
../Libraries/iLLD/TC27D/Tricore/_PinMap/IfxVadc_PinMap.c 

C_DEPS += \
./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxAsclin_PinMap.d \
./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxCcu6_PinMap.d \
./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxCif_PinMap.d \
./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxDsadc_PinMap.d \
./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxEray_PinMap.d \
./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxEth_PinMap.d \
./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxGpt12_PinMap.d \
./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxGtm_PinMap.d \
./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxI2c_PinMap.d \
./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxMsc_PinMap.d \
./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxMultican_PinMap.d \
./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxPort_PinMap.d \
./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxPsi5_PinMap.d \
./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxPsi5s_PinMap.d \
./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxQspi_PinMap.d \
./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxScu_PinMap.d \
./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxSent_PinMap.d \
./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxSmu_PinMap.d \
./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxVadc_PinMap.d 

OBJS += \
./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxAsclin_PinMap.o \
./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxCcu6_PinMap.o \
./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxCif_PinMap.o \
./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxDsadc_PinMap.o \
./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxEray_PinMap.o \
./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxEth_PinMap.o \
./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxGpt12_PinMap.o \
./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxGtm_PinMap.o \
./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxI2c_PinMap.o \
./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxMsc_PinMap.o \
./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxMultican_PinMap.o \
./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxPort_PinMap.o \
./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxPsi5_PinMap.o \
./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxPsi5s_PinMap.o \
./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxQspi_PinMap.o \
./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxScu_PinMap.o \
./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxSent_PinMap.o \
./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxSmu_PinMap.o \
./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxVadc_PinMap.o 


# Each subdirectory must supply rules for building sources it contributes
Libraries/iLLD/TC27D/Tricore/_PinMap/%.o: ../Libraries/iLLD/TC27D/Tricore/_PinMap/%.c Libraries/iLLD/TC27D/Tricore/_PinMap/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: AURIX GCC Compiler'
	tricore-elf-gcc -std=c99 "@C:/Users/USER/AURIX_PJT/AutoDPJT/TriCore Release (GCC)/AURIX_GCC_Compiler-Include_paths__-I_.opt" -O3 -Wall -c -fmessage-length=0 -fno-common -fstrict-volatile-bitfields -fdata-sections -ffunction-sections -mtc161 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-Libraries-2f-iLLD-2f-TC27D-2f-Tricore-2f-_PinMap

clean-Libraries-2f-iLLD-2f-TC27D-2f-Tricore-2f-_PinMap:
	-$(RM) ./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxAsclin_PinMap.d ./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxAsclin_PinMap.o ./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxCcu6_PinMap.d ./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxCcu6_PinMap.o ./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxCif_PinMap.d ./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxCif_PinMap.o ./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxDsadc_PinMap.d ./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxDsadc_PinMap.o ./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxEray_PinMap.d ./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxEray_PinMap.o ./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxEth_PinMap.d ./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxEth_PinMap.o ./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxGpt12_PinMap.d ./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxGpt12_PinMap.o ./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxGtm_PinMap.d ./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxGtm_PinMap.o ./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxI2c_PinMap.d ./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxI2c_PinMap.o ./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxMsc_PinMap.d ./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxMsc_PinMap.o ./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxMultican_PinMap.d ./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxMultican_PinMap.o ./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxPort_PinMap.d ./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxPort_PinMap.o ./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxPsi5_PinMap.d ./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxPsi5_PinMap.o ./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxPsi5s_PinMap.d ./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxPsi5s_PinMap.o ./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxQspi_PinMap.d ./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxQspi_PinMap.o ./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxScu_PinMap.d ./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxScu_PinMap.o ./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxSent_PinMap.d ./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxSent_PinMap.o ./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxSmu_PinMap.d ./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxSmu_PinMap.o ./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxVadc_PinMap.d ./Libraries/iLLD/TC27D/Tricore/_PinMap/IfxVadc_PinMap.o

.PHONY: clean-Libraries-2f-iLLD-2f-TC27D-2f-Tricore-2f-_PinMap

