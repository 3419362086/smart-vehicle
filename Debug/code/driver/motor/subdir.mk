################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
"../code/driver/motor/small_driver_uart_control.c" 

COMPILED_SRCS += \
"code/driver/motor/small_driver_uart_control.src" 

C_DEPS += \
"./code/driver/motor/small_driver_uart_control.d" 

OBJS += \
"code/driver/motor/small_driver_uart_control.o" 


# Each subdirectory must supply rules for building sources it contributes
"code/driver/motor/small_driver_uart_control.src":"../code/driver/motor/small_driver_uart_control.c" "code/driver/motor/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc26xb "-fD:/ZhiNengChe/ZiLiao/smart-vehicle/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/driver/motor/small_driver_uart_control.o":"code/driver/motor/small_driver_uart_control.src" "code/driver/motor/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-code-2f-driver-2f-motor

clean-code-2f-driver-2f-motor:
	-$(RM) ./code/driver/motor/small_driver_uart_control.d ./code/driver/motor/small_driver_uart_control.o ./code/driver/motor/small_driver_uart_control.src

.PHONY: clean-code-2f-driver-2f-motor

