# Project Name
TARGET = asteroidbelt
CPP_STANDARD = -std=c++20
USE_DAISYSP_LGPL = 1

APP_TYPE = BOOT_SRAM

# Compiler options
#OPT = -O1
OPT=-Os
# OPT=-Og -g

# Library Locations
LIBDAISY_DIR = ../../libDaisy
DAISYSP_DIR = ../../DaisySP

# Sources
CMSIS_DSP_DIR = ${LIBDAISY_DIR}/Drivers/CMSIS-DSP

CMSIS_DSP_SRC = \
$(CMSIS_DSP_DIR)/Source/FilteringFunctions/arm_fir_f32.c \
$(CMSIS_DSP_DIR)/Source/FilteringFunctions/arm_fir_init_f32.c \
$(CMSIS_DSP_DIR)/Source/SupportFunctions/arm_fill_f32.c \
$(CMSIS_DSP_DIR)/Source/BasicMathFunctions/arm_scale_f32.c
C_DEFS += -DARM_MATH_LOOPUNROLL

CPP_SOURCES = asteroidbelt.cpp funbox_hardware.cpp hothouse_hardware.cpp ImpulseResponse/ImpulseResponse.cpp \
	Dattorro/Dattorro.cpp Dattorro/dsp/delays/InterpDelay.cpp Dattorro/dsp/filters/OnePoleFilters.cpp
C_SOURCES += $(CMSIS_DSP_SRC)

# Core location, and generic Makefile.
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile

# Include funbox.h
C_INCLUDES += -I../../infra/include
C_INCLUDES += -I../../q/q_lib/include
C_INCLUDES += -I../../eigen
C_INCLUDES += -I../../include
# Add CMSIS-DSP include path
C_INCLUDES += -I$(CMSIS_DSP_DIR)/Include