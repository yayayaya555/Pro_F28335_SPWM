################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := E:/DSP/CCS6.0/compiler6.4/ti-cgt-c2000_6.4.12

ORDERED_OBJS += \
$(GEN_CMDS__FLAG) \
"./source/MainSPWM.obj" \
"./source/DSP2833x_usDelay.obj" \
"./source/DSP2833x_Xintf.obj" \
"./source/DSP2833x_SysCtrl.obj" \
"./source/DSP2833x_Spi.obj" \
"./source/DSP2833x_Sci.obj" \
"./source/DSP2833x_PieVect.obj" \
"./source/DSP2833x_PieCtrl.obj" \
"./source/DSP2833x_MemCopy.obj" \
"./source/DSP2833x_Mcbsp.obj" \
"./source/DSP2833x_I2C.obj" \
"./source/DSP2833x_Gpio.obj" \
"./source/DSP2833x_GlobalVariableDefs.obj" \
"./source/DSP2833x_EQep.obj" \
"./source/DSP2833x_EPwm.obj" \
"./source/DSP2833x_ECap.obj" \
"./source/DSP2833x_ECan.obj" \
"./source/DSP2833x_DisInt.obj" \
"./source/DSP2833x_DefaultIsr.obj" \
"./source/DSP2833x_DMA.obj" \
"./source/DSP2833x_DBGIER.obj" \
"./source/DSP2833x_CpuTimers.obj" \
"./source/DSP2833x_CodeStartBranch.obj" \
"./source/DSP2833x_CSMPasswords.obj" \
"./source/DSP2833x_Adc.obj" \
"./source/DSP2833x_ADC_cal.obj" \
"../lib/SFO_TI_Build.lib" \
"../lib/SFO_TI_Build_V5.lib" \
"../lib/SFO_TI_Build_V5_fpu.lib" \
"../lib/SFO_TI_Build_fpu.lib" \
"../cmd/DSP2833x_Headers_nonBIOS.cmd" \
"../cmd/F28335.cmd" \
-l"libc.a" \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include source/subdir_vars.mk
-include lib/subdir_vars.mk
-include cmd/subdir_vars.mk
-include source/subdir_rules.mk
-include lib/subdir_rules.mk
-include cmd/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
ProDSP_F335.out \

EXE_OUTPUTS__QUOTED += \
"ProDSP_F335.out" \


# All Target
all: ProDSP_F335.out

# Tool invocations
ProDSP_F335.out: $(OBJS) $(CMD_SRCS) $(LIB_SRCS) $(GEN_CMDS)
	@echo 'Building target: $@'
	@echo 'Invoking: C2000 Linker'
	"E:/DSP/CCS6.0/compiler6.4/ti-cgt-c2000_6.4.12/bin/cl2000" -v28 -ml --float_support=fpu32 -g --diag_warning=225 -z -m"ProDSP_F335.map" --stack_size=0x380 --warn_sections -i"E:/DSP/CCS6.0/compiler6.4/ti-cgt-c2000_6.4.12/lib" -i"E:/DSP/CCS6.0/compiler6.4/ti-cgt-c2000_6.4.12/include" --reread_libs --xml_link_info="ProDSP_F335_linkInfo.xml" --rom_model -o "ProDSP_F335.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXE_OUTPUTS__QUOTED)
	-$(RM) "source\DSP2833x_Adc.pp" "source\DSP2833x_CpuTimers.pp" "source\DSP2833x_DMA.pp" "source\DSP2833x_DefaultIsr.pp" "source\DSP2833x_ECan.pp" "source\DSP2833x_ECap.pp" "source\DSP2833x_EPwm.pp" "source\DSP2833x_EQep.pp" "source\DSP2833x_GlobalVariableDefs.pp" "source\DSP2833x_Gpio.pp" "source\DSP2833x_I2C.pp" "source\DSP2833x_Mcbsp.pp" "source\DSP2833x_MemCopy.pp" "source\DSP2833x_PieCtrl.pp" "source\DSP2833x_PieVect.pp" "source\DSP2833x_Sci.pp" "source\DSP2833x_Spi.pp" "source\DSP2833x_SysCtrl.pp" "source\DSP2833x_Xintf.pp" "source\MainSPWM.pp" 
	-$(RM) "source\DSP2833x_ADC_cal.obj" "source\DSP2833x_Adc.obj" "source\DSP2833x_CSMPasswords.obj" "source\DSP2833x_CodeStartBranch.obj" "source\DSP2833x_CpuTimers.obj" "source\DSP2833x_DBGIER.obj" "source\DSP2833x_DMA.obj" "source\DSP2833x_DefaultIsr.obj" "source\DSP2833x_DisInt.obj" "source\DSP2833x_ECan.obj" "source\DSP2833x_ECap.obj" "source\DSP2833x_EPwm.obj" "source\DSP2833x_EQep.obj" "source\DSP2833x_GlobalVariableDefs.obj" "source\DSP2833x_Gpio.obj" "source\DSP2833x_I2C.obj" "source\DSP2833x_Mcbsp.obj" "source\DSP2833x_MemCopy.obj" "source\DSP2833x_PieCtrl.obj" "source\DSP2833x_PieVect.obj" "source\DSP2833x_Sci.obj" "source\DSP2833x_Spi.obj" "source\DSP2833x_SysCtrl.obj" "source\DSP2833x_Xintf.obj" "source\DSP2833x_usDelay.obj" "source\MainSPWM.obj" 
	-$(RM) "source\DSP2833x_ADC_cal.pp" "source\DSP2833x_CSMPasswords.pp" "source\DSP2833x_CodeStartBranch.pp" "source\DSP2833x_DBGIER.pp" "source\DSP2833x_DisInt.pp" "source\DSP2833x_usDelay.pp" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

