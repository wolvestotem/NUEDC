################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf *.sni *.dbg *.lst *.lis *.lpp *.map *.x *.lbp src/*.src src/*.p src/*.pp src/*.mrc *.bls *.libelf *.lbk *.abs *.fsy

# All of the sources participating in the build are defined here
-include sources.mk
-include src/Bluetooth_Decoder/subdir.mk
-include src/Mavlink/subdir.mk
-include src/OpenMV/subdir.mk
-include src/PathSeeking/subdir.mk
-include src/Safety_Switch/subdir.mk
-include src/Sc_AlgorithmTools/subdir.mk
-include src/Sc_CopterControl/subdir.mk
-include src/Sc_PID/subdir.mk
-include src/Sc_Serial_Library_new/subdir.mk
-include src/Self_Check/subdir.mk
-include src/cg_src/subdir.mk
-include src/subdir.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_UPPER_DEPS)),)
-include $(CC_UPPER_DEPS)
endif
ifneq ($(strip $(C++_UPPER_DEPS)),)
-include $(C++_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_UPPER_DEPS)),)
-include $(CXX_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_UPPER_DEPS)),)
-include $(CPP_UPPER_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CP_DEPS)),)
-include $(CP_DEPS)
endif
ifneq ($(strip $(SRC_UPPER_DEPS)),)
-include $(SRC_UPPER_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(SRC_DEPS)),)
-include $(SRC_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(CP_UPPER_DEPS)),)
-include $(CP_UPPER_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(OBJ_DEPS)),)
-include $(OBJ_DEPS)
endif
ifneq ($(strip $(ABS_DEPS)),)
-include $(ABS_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(O_DEPS)),)
-include $(O_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
endif

ifneq ($(strip $(SRC_UPPER_SRCS)),)
ALL_ASMS += $(SRC_UPPER_SRCS) 
endif
ifneq ($(strip $(SRC_SRCS)),)
ALL_ASMS += $(SRC_SRCS) 
endif
ifneq ($(strip $(S_UPPER_SRCS)),)
ALL_ASMS += $(S_UPPER_SRCS) 
endif
ifneq ($(strip $(S_SRCS)),)
ALL_ASMS += $(S_SRCS) 
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
CONVERTOR_OUTPUT_OUTPUTS += \
Electric_Match_Final.mot \

LINKER_OUTPUT_OUTPUTS += \
Electric_Match_Final.abs \

LIBRARY_GENERATOR_OUTPUTTYPE_OUTPUTS += \
Electric_Match_Final.lib \

# All Target
all: Electric_Match_Final.mot
	@echo 'Build complete.'

# Tool invocations
Electric_Match_Final.mot: $(LINKER_OUTPUT_OUTPUTS)
	@echo 'Invoking: Converter'
	rlink   "E:\Technical events\electric match 2017\Electric_Match_Final\HardwareDebug\Electric_Match_Final.abs" -form=stype  -output="E:\Technical events\electric match 2017\Electric_Match_Final\HardwareDebug\Electric_Match_Final.mot" -nomessage
	@echo 'Finished building target:'
	@echo.
	$(MAKE) --no-print-directory post-build

Electric_Match_Final.abs: $(OBJS) $(LIBRARY_GENERATOR_OUTPUTTYPE_OUTPUTS) $(ALL_ASMS)   \
E:\TECHNI~1\ELECTR~1\ELECTR~1\src\Mavlink\MAVLIN~1.LIB
	@echo 'Invoking: Linker'
	@echo 'Building target:'
	rlink  -subcommand="LinkerSubCommand.tmp"  -library="E:\TECHNI~1\ELECTR~1\ELECTR~1\HARDWA~2\Electric_Match_Final.lib" -library="E:\Technical events\electric match 2017\Electric_Match_Final\src\Mavlink\Mavlink_lib.lib" -noprelink -list="Electric_Match_Final.map" -show -nooptimize -debug -output="E:\Technical events\electric match 2017\Electric_Match_Final\HardwareDebug\Electric_Match_Final.abs" -rom=D=R,D_1=R_1,D_2=R_2 -nocompress -nomessage -nologo
	@echo 'Finished building:'
	@echo.

Electric_Match_Final.lib:  
	@echo 'Invoking: Standard Library'
	lbgrx -head=runtime,math,stdio,stdlib,string,new -isa=rxv2 -fpu -output="Electric_Match_Final.lib"
	@echo 'Finished building:'
	@echo.

# Other Targets
clean:
ifneq ($(strip $(OBJS)),)
	-$(file > clean_OBJS.tmp , $(OBJS))
	-@ xargs -s 6000 -t -a clean_OBJS.tmp $(RM)
	-@$(RM) clean_OBJS.tmp
endif
ifneq ($(strip $(C_DEPS)),)
	-$(file > clean_C_DEPS.tmp , $(C_DEPS))
	-@ xargs -s 6000 -t -a clean_C_DEPS.tmp $(RM)
	-@$(RM) clean_C_DEPS.tmp
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
	-$(file > clean_C_UPPER_DEPS.tmp , $(C_UPPER_DEPS))
	-@ xargs -s 6000 -t -a clean_C_UPPER_DEPS.tmp $(RM)
	-@$(RM) clean_C_UPPER_DEPS.tmp
endif
ifneq ($(strip $(CPP_DEPS)),)
	-$(file > clean_CPP_DEPS.tmp , $(CPP_DEPS))
	-@ xargs -s 6000 -t -a clean_CPP_DEPS.tmp $(RM)
	-@$(RM) clean_CPP_DEPS.tmp
endif
ifneq ($(strip $(CPP_UPPER_DEPS)),)
	-$(file > clean_CPP_UPPER_DEPS.tmp , $(CPP_UPPER_DEPS))
	-@ xargs -s 6000 -t -a clean_CPP_UPPER_DEPS.tmp $(RM)
	-@$(RM) clean_CPP_UPPER_DEPS.tmp
endif
ifneq ($(strip $(C++_DEPS)),)
	-$(file > clean_C++_DEPS.tmp , $(C++_DEPS))
	-@ xargs -s 6000 -t -a clean_C++_DEPS.tmp $(RM)
	-@$(RM) clean_C++_DEPS.tmp
endif
ifneq ($(strip $(C++_UPPER_DEPS)),)
	-$(file > clean_C++_UPPER_DEPS.tmp , $(C++_UPPER_DEPS))
	-@ xargs -s 6000 -t -a clean_C++_UPPER_DEPS.tmp $(RM)
	-@$(RM) clean_C++_UPPER_DEPS.tmp
endif
ifneq ($(strip $(CP_DEPS)),)
	-$(file > clean_CP_DEPS.tmp , $(CP_DEPS))
	-@ xargs -s 6000 -t -a clean_CP_DEPS.tmp $(RM)
	-@$(RM) clean_CP_DEPS.tmp
endif
ifneq ($(strip $(CP_UPPER_DEPS)),)
	-$(file > clean_CP_UPPER_DEPS.tmp , $(CP_UPPER_DEPS))
	-@ xargs -s 6000 -t -a clean_CP_UPPER_DEPS.tmp $(RM)
	-@$(RM) clean_CP_UPPER_DEPS.tmp
endif
ifneq ($(strip $(CXX_DEPS)),)
	-$(file > clean_CXX_DEPS.tmp , $(CXX_DEPS))
	-@ xargs -s 6000 -t -a clean_CXX_DEPS.tmp $(RM)
	-@$(RM) clean_CXX_DEPS.tmp
endif
ifneq ($(strip $(CXX_UPPER_DEPS)),)
	-$(file > clean_CXX_UPPER_DEPS.tmp , $(CXX_UPPER_DEPS))
	-@ xargs -s 6000 -t -a clean_CXX_UPPER_DEPS.tmp $(RM)
	-@$(RM) clean_CXX_UPPER_DEPS.tmp
endif
ifneq ($(strip $(CC_DEPS)),)
	-$(file > clean_CC_DEPS.tmp , $(CC_DEPS))
	-@ xargs -s 6000 -t -a clean_CC_DEPS.tmp $(RM)
	-@$(RM) clean_CC_DEPS.tmp
endif
ifneq ($(strip $(CC_UPPER_DEPS)),)
	-$(file > clean_CC_UPPER_DEPS.tmp , $(CC_UPPER_DEPS))
	-@ xargs -s 6000 -t -a clean_CC_UPPER_DEPS.tmp $(RM)
	-@$(RM) clean_CC_UPPER_DEPS.tmp
endif
ifneq ($(strip $(ASM_PRNS)),)
	-$(file > clean_ASM_PRNS.tmp , $(ASM_PRNS))
	-@ xargs -s 6000 -t -a clean_ASM_PRNS.tmp $(RM)
	-@$(RM) clean_ASM_PRNS.tmp
endif
ifneq ($(strip $(OBJ_DEPS)),)
	-$(file > clean_OBJ_DEPS.tmp , $(OBJ_DEPS))
	-@ xargs -s 6000 -t -a clean_OBJ_DEPS.tmp $(RM)
	-@$(RM) clean_OBJ_DEPS.tmp
endif
ifneq ($(strip $(ABS_DEPS)),)
	-$(file > clean_ABS_DEPS.tmp , $(ABS_DEPS))
	-@ xargs -s 6000 -t -a clean_ABS_DEPS.tmp $(RM)
	-@$(RM) clean_ABS_DEPS.tmp
endif
ifneq ($(strip $(O_DEPS)),)
	-$(file > clean_O_DEPS.tmp , $(O_DEPS))
	-@ xargs -s 6000 -t -a clean_O_DEPS.tmp $(RM)
	-@$(RM) clean_O_DEPS.tmp
endif
ifneq ($(strip $(S_DEPS)),)
	-$(file > clean_S_DEPS.tmp , $(S_DEPS))
	-@ xargs -s 6000 -t -a clean_S_DEPS.tmp $(RM)
	-@$(RM) clean_S_DEPS.tmp
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
	-$(file > clean_S_UPPER_DEPS.tmp , $(S_UPPER_DEPS))
	-@ xargs -s 6000 -t -a clean_S_UPPER_DEPS.tmp $(RM)
	-@$(RM) clean_S_UPPER_DEPS.tmp
endif
ifneq ($(strip $(SRC_DEPS)),)
	-$(file > clean_SRC_DEPS.tmp , $(SRC_DEPS))
	-@ xargs -s 6000 -t -a clean_SRC_DEPS.tmp $(RM)
	-@$(RM) clean_SRC_DEPS.tmp
endif
ifneq ($(strip $(SRC_UPPER_DEPS)),)
	-$(file > clean_SRC_UPPER_DEPS.tmp , $(SRC_UPPER_DEPS))
	-@ xargs -s 6000 -t -a clean_SRC_UPPER_DEPS.tmp $(RM)
	-@$(RM) clean_SRC_UPPER_DEPS.tmp
endif
ifneq ($(strip $(ASM_DEPS)),)
	-$(file > clean_ASM_DEPS.tmp , $(ASM_DEPS))
	-@ xargs -s 6000 -t -a clean_ASM_DEPS.tmp $(RM)
	-@$(RM) clean_ASM_DEPS.tmp
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
	-$(file > clean_ASM_UPPER_DEPS.tmp , $(ASM_UPPER_DEPS))
	-@ xargs -s 6000 -t -a clean_ASM_UPPER_DEPS.tmp $(RM)
	-@$(RM) clean_ASM_UPPER_DEPS.tmp
endif
	-$(RM) $(LIBRARY_GENERATOR_OUTPUTTYPE_OUTPUTS) $(CONVERTOR_OUTPUT_OUTPUTS) 
	-rm -rf "src/Bluetooth_Decoder"/*.obj "src/Bluetooth_Decoder"/*.d "src/Bluetooth_Decoder"/*.src "src/Bluetooth_Decoder"/*.p "src/Bluetooth_Decoder"/*.pp "src/Bluetooth_Decoder"/*.mrc "src/Bluetooth_Decoder"/*.prn "src/Bluetooth_Decoder"/*.asm "src/Bluetooth_Decoder"/*.lst
	-rm -rf "src/Mavlink"/*.obj "src/Mavlink"/*.d "src/Mavlink"/*.src "src/Mavlink"/*.p "src/Mavlink"/*.pp "src/Mavlink"/*.mrc "src/Mavlink"/*.prn "src/Mavlink"/*.asm "src/Mavlink"/*.lst
	-rm -rf "src/OpenMV"/*.obj "src/OpenMV"/*.d "src/OpenMV"/*.src "src/OpenMV"/*.p "src/OpenMV"/*.pp "src/OpenMV"/*.mrc "src/OpenMV"/*.prn "src/OpenMV"/*.asm "src/OpenMV"/*.lst
	-rm -rf "src/PathSeeking"/*.obj "src/PathSeeking"/*.d "src/PathSeeking"/*.src "src/PathSeeking"/*.p "src/PathSeeking"/*.pp "src/PathSeeking"/*.mrc "src/PathSeeking"/*.prn "src/PathSeeking"/*.asm "src/PathSeeking"/*.lst
	-rm -rf "src/Safety_Switch"/*.obj "src/Safety_Switch"/*.d "src/Safety_Switch"/*.src "src/Safety_Switch"/*.p "src/Safety_Switch"/*.pp "src/Safety_Switch"/*.mrc "src/Safety_Switch"/*.prn "src/Safety_Switch"/*.asm "src/Safety_Switch"/*.lst
	-rm -rf "src/Sc_AlgorithmTools"/*.obj "src/Sc_AlgorithmTools"/*.d "src/Sc_AlgorithmTools"/*.src "src/Sc_AlgorithmTools"/*.p "src/Sc_AlgorithmTools"/*.pp "src/Sc_AlgorithmTools"/*.mrc "src/Sc_AlgorithmTools"/*.prn "src/Sc_AlgorithmTools"/*.asm "src/Sc_AlgorithmTools"/*.lst
	-rm -rf "src/Sc_CopterControl"/*.obj "src/Sc_CopterControl"/*.d "src/Sc_CopterControl"/*.src "src/Sc_CopterControl"/*.p "src/Sc_CopterControl"/*.pp "src/Sc_CopterControl"/*.mrc "src/Sc_CopterControl"/*.prn "src/Sc_CopterControl"/*.asm "src/Sc_CopterControl"/*.lst
	-rm -rf "src/Sc_PID"/*.obj "src/Sc_PID"/*.d "src/Sc_PID"/*.src "src/Sc_PID"/*.p "src/Sc_PID"/*.pp "src/Sc_PID"/*.mrc "src/Sc_PID"/*.prn "src/Sc_PID"/*.asm "src/Sc_PID"/*.lst
	-rm -rf "src/Sc_Serial_Library_new"/*.obj "src/Sc_Serial_Library_new"/*.d "src/Sc_Serial_Library_new"/*.src "src/Sc_Serial_Library_new"/*.p "src/Sc_Serial_Library_new"/*.pp "src/Sc_Serial_Library_new"/*.mrc "src/Sc_Serial_Library_new"/*.prn "src/Sc_Serial_Library_new"/*.asm "src/Sc_Serial_Library_new"/*.lst
	-rm -rf "src/Self_Check"/*.obj "src/Self_Check"/*.d "src/Self_Check"/*.src "src/Self_Check"/*.p "src/Self_Check"/*.pp "src/Self_Check"/*.mrc "src/Self_Check"/*.prn "src/Self_Check"/*.asm "src/Self_Check"/*.lst
	-rm -rf "src/cg_src"/*.obj "src/cg_src"/*.d "src/cg_src"/*.src "src/cg_src"/*.p "src/cg_src"/*.pp "src/cg_src"/*.mrc "src/cg_src"/*.prn "src/cg_src"/*.asm "src/cg_src"/*.lst
	-rm -rf "src"/*.obj "src"/*.d "src"/*.src "src"/*.p "src"/*.pp "src"/*.mrc "src"/*.prn "src"/*.asm "src"/*.lst
	@echo.
	@echo 'Clean complete.'

post-build:
	@echo.
	-E:\Renesas\E2_STU~1\DEBUGC~1\RENESA~1.EXE Electric_Match_Final.abs Electric_Match_Final.x 
	@echo.

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
