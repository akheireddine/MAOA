################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../CPLEX_AK/Graph_AK.cpp \
../CPLEX_AK/MTZ_VRP.cpp 

OBJS += \
./CPLEX_AK/Graph_AK.o \
./CPLEX_AK/MTZ_VRP.o 

CPP_DEPS += \
./CPLEX_AK/Graph_AK.d \
./CPLEX_AK/MTZ_VRP.d 


# Each subdirectory must supply rules for building sources it contributes
CPLEX_AK/%.o: ../CPLEX_AK/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


