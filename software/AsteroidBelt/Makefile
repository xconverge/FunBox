# Project Name
TARGET = asteroidbelt
CPP_STANDARD = -std=c++20
USE_DAISYSP_LGPL = 1

APP_TYPE = BOOT_SRAM

# Compiler options
#OPT = -O1
OPT=-Os


# Sources
CPP_SOURCES = asteroidbelt.cpp funbox_hardware.cpp ImpulseResponse/ImpulseResponse.cpp ImpulseResponse/dsp.cpp

# Library Locations
LIBDAISY_DIR = ../../libDaisy
DAISYSP_DIR = ../../DaisySP

# Core location, and generic Makefile.
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile

# Include funbox.h
C_INCLUDES += -I../../infra/include
C_INCLUDES += -I../../q/q_lib/include
C_INCLUDES += -I../../eigen
C_INCLUDES += -I../../include