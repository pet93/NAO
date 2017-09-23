################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../qpOASES/BLASReplacement.cpp \
../qpOASES/Bounds.cpp \
../qpOASES/Constraints.cpp \
../qpOASES/Flipper.cpp \
../qpOASES/Indexlist.cpp \
../qpOASES/LAPACKReplacement.cpp \
../qpOASES/Matrices.cpp \
../qpOASES/MessageHandling.cpp \
../qpOASES/OQPinterface.cpp \
../qpOASES/Options.cpp \
../qpOASES/QProblem.cpp \
../qpOASES/QProblemB.cpp \
../qpOASES/SQProblem.cpp \
../qpOASES/SQProblemSchur.cpp \
../qpOASES/SolutionAnalysis.cpp \
../qpOASES/SparseSolver.cpp \
../qpOASES/SubjectTo.cpp \
../qpOASES/Utils.cpp 

OBJS += \
./qpOASES/BLASReplacement.o \
./qpOASES/Bounds.o \
./qpOASES/Constraints.o \
./qpOASES/Flipper.o \
./qpOASES/Indexlist.o \
./qpOASES/LAPACKReplacement.o \
./qpOASES/Matrices.o \
./qpOASES/MessageHandling.o \
./qpOASES/OQPinterface.o \
./qpOASES/Options.o \
./qpOASES/QProblem.o \
./qpOASES/QProblemB.o \
./qpOASES/SQProblem.o \
./qpOASES/SQProblemSchur.o \
./qpOASES/SolutionAnalysis.o \
./qpOASES/SparseSolver.o \
./qpOASES/SubjectTo.o \
./qpOASES/Utils.o 

CPP_DEPS += \
./qpOASES/BLASReplacement.d \
./qpOASES/Bounds.d \
./qpOASES/Constraints.d \
./qpOASES/Flipper.d \
./qpOASES/Indexlist.d \
./qpOASES/LAPACKReplacement.d \
./qpOASES/Matrices.d \
./qpOASES/MessageHandling.d \
./qpOASES/OQPinterface.d \
./qpOASES/Options.d \
./qpOASES/QProblem.d \
./qpOASES/QProblemB.d \
./qpOASES/SQProblem.d \
./qpOASES/SQProblemSchur.d \
./qpOASES/SolutionAnalysis.d \
./qpOASES/SparseSolver.d \
./qpOASES/SubjectTo.d \
./qpOASES/Utils.d 


# Each subdirectory must supply rules for building sources it contributes
qpOASES/%.o: ../qpOASES/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/usr/include/eigen3 -I/usr/local/include -I/usr/local/include/dart -O0 -g3 -Wall -c -fmessage-length=0 -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


