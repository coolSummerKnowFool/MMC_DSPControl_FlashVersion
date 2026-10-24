################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
APP/leds/%.obj: ../APP/leds/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1281/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 -O3 --opt_for_speed=5 --fp_mode=relaxed --include_path="D:/CCS_workspace/DSP_Learning/MMC_DSPControl_Flash/DSP2833x_Libraries/common/include" --include_path="D:/CCS_workspace/DSP_Learning/MMC_DSPControl_Flash/APP/leds" --include_path="D:/CCS_workspace/DSP_Learning/MMC_DSPControl_Flash/APP/uart_fifo" --include_path="D:/CCS_workspace/DSP_Learning/MMC_DSPControl_Flash/APP/printf" --include_path="D:/CCS_workspace/DSP_Learning/MMC_DSPControl_Flash/APP/tim" --include_path="D:/CCS_workspace/DSP_Learning/MMC_DSPControl_Flash/APP/exti" --include_path="D:/CCS_workspace/DSP_Learning/MMC_DSPControl_Flash/APP/basic" --include_path="D:/CCS_workspace/DSP_Learning/MMC_DSPControl_Flash/APP/spi_fifo" --include_path="D:/CCS_workspace/DSP_Learning/MMC_DSPControl_Flash/APP/spi_fifo/oled" --include_path="D:/CCS_workspace/DSP_Learning/MMC_DSPControl_Flash/APP/spi_fifo/flash" --include_path="D:/CCS_workspace/DSP_Learning/MMC_DSPControl_Flash/APP/menu" --include_path="D:/CCS_workspace/DSP_Learning/MMC_DSPControl_Flash/APP/epwm" --include_path="D:/CCS_workspace/DSP_Learning/MMC_DSPControl_Flash/APP/control" --include_path="D:/CCS_workspace/DSP_Learning/MMC_DSPControl_Flash/APP/adc" --include_path="D:/CCS_workspace/DSP_Learning/MMC_DSPControl_Flash" --include_path="C:/ti/ccs1281/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/include" --include_path="D:/CCS_workspace/DSP_Learning/MMC_DSPControl_Flash/DSP2833x_Libraries/headers/include" --include_path="C:/ti/ccs1281/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/include" --advice:performance=all -g --c99 --printf_support=full --diag_warning=225 --diag_wrap=off --display_error_number --abi=coffabi --preproc_with_compile --preproc_dependency="APP/leds/$(basename $(<F)).d_raw" --obj_directory="APP/leds" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


