################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Controller.cpp \
../MPCSolver.cpp \
../Main.cpp \
../MyWindow.cpp \
../utils.cpp 

OBJS += \
./Controller.o \
./MPCSolver.o \
./Main.o \
./MyWindow.o \
./utils.o 

CPP_DEPS += \
./Controller.d \
./MPCSolver.d \
./Main.d \
./MyWindow.d \
./utils.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/usr/include/eigen3 -I/usr/local/include -I/usr/local/include/dart -O0 -g3 -Wall -c -fmessage-length=0 -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


