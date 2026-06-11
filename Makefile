MY_ARCH := $(shell uname -m)
$(info $(MY_ARCH))

CXX = g++
CC = gcc
AR = ar
ifeq ($(MY_ARCH), aarch64)
	LIB_PATH = -L/usr/lib/aarch64-linux-gnu
	TARGET = ARMV8
else ifeq ($(MY_ARCH), x86_64)
	LIB_PATH = -L/usr/lib
	TARGET = X86
endif

OPTS = -Wall -W -O3 -g -Wno-deprecated-copy
CXXFLAGS += -std=c++11 -Wextra -fno-strict-aliasing -Wno-unused-parameter $(OPTS) 
# LIBS = -lz -lssl -lcrypto
#LIBS += -lgfortran -larmadillo -lblas -llapack -lrt -ldl -luv
OBJDIR := .libs


all: TEST_MOTOR_TINY
clean :
	rm -f $(TESTS) MAIN* TEST_MOTOR_TINY


# .PHONY:

#-------------------------------MAKE TARGETS-------------------------------

TEST_COMMUNICATION: MotorTestDirect.cpp
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -o $@ $^ $(LIB_PATH)
