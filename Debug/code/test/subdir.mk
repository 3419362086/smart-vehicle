################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
"../code/test/justfloat.c" 

COMPILED_SRCS += \
"code/test/justfloat.src" 

C_DEPS += \
"./code/test/justfloat.d" 

OBJS += \
"code/test/justfloat.o" 


# Each subdirectory must supply rules for building sources it contributes
"code/test/justfloat.src":"../code/test/justfloat.c" "code/test/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc26xb "-fD:/ZhiNengChe/ZiLiao/smart-vehicle/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/test/justfloat.o":"code/test/justfloat.src" "code/test/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-code-2f-test

clean-code-2f-test:
	-$(RM) ./code/test/justfloat.d ./code/test/justfloat.o ./code/test/justfloat.src

.PHONY: clean-code-2f-test

