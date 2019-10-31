################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
..\src/Sc_Serial_Library_new/Sc_Serial_Library_new.c 

C_DEPS += \
./src/Sc_Serial_Library_new/Sc_Serial_Library_new.d 

OBJS += \
./src/Sc_Serial_Library_new/Sc_Serial_Library_new.obj 


# Each subdirectory must supply rules for building sources it contributes
src/Sc_Serial_Library_new/%.obj: ../src/Sc_Serial_Library_new/%.c
	@echo 'Scanning and building file: $<'
	@echo 'Invoking: Scanner and Compiler'
	ccrx  -MM -MP -output=dep="$(@:%.obj=%.d)" -MT="$(@:%.obj=%.obj)" -MT="$(@:%.obj=%.d)" -lang=c   -include="C:\PROGRA~2\Renesas\RX\2_6_0/include"  -debug -nomessage -isa=rxv2 -fpu -nologo  -define=__RX=1   "$<"
	ccrx -lang=c -output=obj="$(@:%.d=%.obj)"  -include="C:\PROGRA~2\Renesas\RX\2_6_0/include"  -debug -nomessage -isa=rxv2 -fpu -nologo  -define=__RX=1   "$<"
	@echo 'Finished scanning and building: $<'
	@echo.

