################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
DSP2833x_Libraries/DSP2833x_headers/source/%.obj: ../DSP2833x_Libraries/DSP2833x_headers/source/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"D:/ti/ccs1281/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --include_path="F:/CCS_workspace/HelloDSP28335/DSP2833x_Libraries/DSP2833x_common/include" --include_path="F:/CCS_workspace/HelloDSP28335/DSP2833x_Libraries/DSP2833x_common/source" --include_path="F:/CCS_workspace/HelloDSP28335/DSP2833x_Libraries/DSP2833x_common" --include_path="F:/CCS_workspace/HelloDSP28335/DSP2833x_Libraries/DSP2833x_common/lib" --include_path="F:/CCS_workspace/HelloDSP28335" --include_path="D:/ti/ccs1281/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/include" --include_path="F:/CCS_workspace/HelloDSP28335/DSP2833x_Libraries/DSP2833x_headers/include" -g --diag_warning=225 --diag_wrap=off --display_error_number --abi=coffabi --preproc_with_compile --preproc_dependency="DSP2833x_Libraries/DSP2833x_headers/source/$(basename $(<F)).d_raw" --obj_directory="DSP2833x_Libraries/DSP2833x_headers/source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


