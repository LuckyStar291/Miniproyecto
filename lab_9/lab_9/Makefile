#******************************************************************************
#
# Makefile - Rules for building the project.
#
#******************************************************************************

# Defines the part type that this project uses.
PART=TM4C1294NCPDT

# The base directory for TivaWare.
ROOT=../../../..

# Define the project name as a variable.
NOMBRE_PROJECT=lab_9

# Include the common make definitions.
include ${ROOT}/makedefs

# Where to find source files that do not live in this directory.
VPATH=../../../../utils

# Where to find header files that do not live in the source directory.
IPATH=../../../..

# The default rule, which causes the project to be built.
all: ${COMPILER}
all: ${COMPILER}/${NOMBRE_PROJECT}.axf

# The rule to clean out all the build products.
clean:
	@rm -rf ${COMPILER} ${wildcard *~}

# The rule to create the target directory.
${COMPILER}:
	@mkdir -p ${COMPILER}

# Rules for building the project.
${COMPILER}/${NOMBRE_PROJECT}.axf: ${COMPILER}/startup_${COMPILER}.o
${COMPILER}/${NOMBRE_PROJECT}.axf: ${COMPILER}/${NOMBRE_PROJECT}.o
${COMPILER}/${NOMBRE_PROJECT}.axf: ${COMPILER}/uartstdio.o
${COMPILER}/${NOMBRE_PROJECT}.axf: ${ROOT}/driverlib/${COMPILER}/libdriver.a
${COMPILER}/${NOMBRE_PROJECT}.axf: ${NOMBRE_PROJECT}.ld

SCATTERgcc_${NOMBRE_PROJECT}=${NOMBRE_PROJECT}.ld
ENTRY_${NOMBRE_PROJECT}=ResetISR
CFLAGSgcc=-DTARGET_IS_TM4C129_RA2

# Include the automatically generated dependency files.
ifneq (${MAKECMDGOALS},clean)
-include ${wildcard ${COMPILER}/*.d} __dummy__
endif
