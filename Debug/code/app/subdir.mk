################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
"../code/app/balance_app.c" \
"../code/app/imu_app.c" \
"../code/app/motor_app.c" \
"../code/app/schedule.c" \
"../code/app/servo_app.c" 

COMPILED_SRCS += \
"code/app/balance_app.src" \
"code/app/imu_app.src" \
"code/app/motor_app.src" \
"code/app/schedule.src" \
"code/app/servo_app.src" 

C_DEPS += \
"./code/app/balance_app.d" \
"./code/app/imu_app.d" \
"./code/app/motor_app.d" \
"./code/app/schedule.d" \
"./code/app/servo_app.d" 

OBJS += \
"code/app/balance_app.o" \
"code/app/imu_app.o" \
"code/app/motor_app.o" \
"code/app/schedule.o" \
"code/app/servo_app.o" 


# Each subdirectory must supply rules for building sources it contributes
"code/app/balance_app.src":"../code/app/balance_app.c" "code/app/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc26xb "-fD:/ZhiNengChe/ZiLiao/smart-vehicle/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/app/balance_app.o":"code/app/balance_app.src" "code/app/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/app/imu_app.src":"../code/app/imu_app.c" "code/app/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc26xb "-fD:/ZhiNengChe/ZiLiao/smart-vehicle/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/app/imu_app.o":"code/app/imu_app.src" "code/app/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/app/motor_app.src":"../code/app/motor_app.c" "code/app/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc26xb "-fD:/ZhiNengChe/ZiLiao/smart-vehicle/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/app/motor_app.o":"code/app/motor_app.src" "code/app/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/app/schedule.src":"../code/app/schedule.c" "code/app/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc26xb "-fD:/ZhiNengChe/ZiLiao/smart-vehicle/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/app/schedule.o":"code/app/schedule.src" "code/app/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/app/servo_app.src":"../code/app/servo_app.c" "code/app/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc26xb "-fD:/ZhiNengChe/ZiLiao/smart-vehicle/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/app/servo_app.o":"code/app/servo_app.src" "code/app/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-code-2f-app

clean-code-2f-app:
	-$(RM) ./code/app/balance_app.d ./code/app/balance_app.o ./code/app/balance_app.src ./code/app/imu_app.d ./code/app/imu_app.o ./code/app/imu_app.src ./code/app/motor_app.d ./code/app/motor_app.o ./code/app/motor_app.src ./code/app/schedule.d ./code/app/schedule.o ./code/app/schedule.src ./code/app/servo_app.d ./code/app/servo_app.o ./code/app/servo_app.src

.PHONY: clean-code-2f-app

