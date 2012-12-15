#******************************************************************************
#
# Makefile - Rules for building the Frequency analyzer using Kentek display.
#
# Copyright (c) 2011 Texas Instruments Incorporated.  All rights reserved.
# TI Information - Selective Disclosure
#
#******************************************************************************

#
# Defines the part type that this project uses.
#
PART=LM4F120H5QR

#
# Set the processor variant.
#
VARIANT=cm4f

#
# The base directory for StellarisWare.
#
ROOT=../../..

#
# Include the common make definitions.
#
include ${ROOT}/makedefs

#
# Where to find source files that do not live in this directory.
#
VPATH=../../../utils

#
# Where to find header files that do not live in the source directory.
#
IPATH=..
IPATH+=../../..
IPATH+=../../../dsplib

#
# The default rule, which causes the Frequency analyzer using Kentek display to be built.
#
all: ${COMPILER}
all: ${COMPILER}/freq_analyzer.axf
##### INTERNAL BEGIN #####
all_comp:
	@for i in `grep COMPILER= ${ROOT}/makedefs | awk -F= '{print $$2}'`; \
	 do                                                                  \
	     ${MAKE} COMPILER=$${i};                                            \
	 done

#
# A rule to regenerate the Makefile if config.in has changed.
#
Makefile: config.in
	@perl ${ROOT}/scripts/buildproj.pl -s -f config.in
##### INTERNAL END #####

#
# The rule to clean out all the build products.
#
clean:
	@rm -rf ${COMPILER} ${wildcard *~}
##### INTERNAL BEGIN #####
	@rm -rf settings ${wildcard *.Bak} ${wildcard *.dep}
clean_comp:
	@for i in `grep COMPILER= ${ROOT}/makedefs | awk -F= '{print $$2}'`; \
	 do                                                                  \
	     ${MAKE} COMPILER=$${i} clean;                                      \
	 done

#
# Cleans out everything, including the Makefile.  Should return this directory
# to fresh from subversion status.
#
realclean:: clean
	@rm -rf ccs codered ewarm gcc rvmdk sourcerygxx ccstempws
	@rm -f ${wildcard *.Uv2} ${wildcard *.Opt}
	@rm -f ${wildcard *.uvopt} ${wildcard *.uvproj}
	@rm -f ${wildcard *.ew?} ${wildcard *.xcl}
	@rm -f ${wildcard *.sgxx}
	@rm -f ccs_proj_args.txt
	@rm -f freq_analyzer_ccs.cmd
	@rm -f freq_analyzer_codered.ld
	@rm -f freq_analyzer.ld
	@rm -f freq_analyzer_sourcerygxx.ld
	@rm -f ${wildcard *-rom.ld}
	@rm -f freq_analyzer.icf
	@rm -f freq_analyzer.sct
	@rm -f ${wildcard *.xml}
	@rm -f ${wildcard startup_*}
	@rm -f readme.txt
	@rm -f Makefile cr_project.xml
##### INTERNAL END #####

#
# The rule to create the target directory.
#
${COMPILER}:
	@mkdir -p ${COMPILER}

#
# Rules for building the Frequency analyzer using Kentek display.
#
${COMPILER}/freq_analyzer.axf: ${COMPILER}/Kentec320x240x16_ssd2119_8bit.o
${COMPILER}/freq_analyzer.axf: ${COMPILER}/dsp.o
${COMPILER}/freq_analyzer.axf: ${COMPILER}/freq_analyzer.o
${COMPILER}/freq_analyzer.axf: ${COMPILER}/gui.o
${COMPILER}/freq_analyzer.axf: ${COMPILER}/images.o
${COMPILER}/freq_analyzer.axf: ${COMPILER}/logoUnc.o
${COMPILER}/freq_analyzer.axf: ${COMPILER}/startup_${COMPILER}.o
${COMPILER}/freq_analyzer.axf: ${COMPILER}/ti_hamming_window_vector.o
${COMPILER}/freq_analyzer.axf: ${COMPILER}/touch.o
${COMPILER}/freq_analyzer.axf: ${COMPILER}/uartstdio.o
${COMPILER}/freq_analyzer.axf: ${COMPILER}/ustdlib.o
${COMPILER}/freq_analyzer.axf: ${ROOT}/grlib/${COMPILER}-cm4f/libgr-cm4f.a
${COMPILER}/freq_analyzer.axf: ${ROOT}/driverlib/${COMPILER}-cm4f/libdriver-cm4f.a
##### INTERNAL BEGIN #####
ifeq (${COMPILER}, ccs)
${COMPILER}/freq_analyzer.axf: freq_analyzer_ccs.cmd
endif
SCATTERccs_freq_analyzer=freq_analyzer_ccs.cmd
ifeq (${COMPILER}, codered)
${COMPILER}/freq_analyzer.axf: freq_analyzer_codered.ld
endif
SCATTERcodered_freq_analyzer=freq_analyzer_codered.ld
ifeq (${COMPILER}, ewarm)
${COMPILER}/freq_analyzer.axf: freq_analyzer.icf
endif
SCATTERewarm_freq_analyzer=freq_analyzer.icf
ifeq (${COMPILER}, gcc)
##### INTERNAL END #####
${COMPILER}/freq_analyzer.axf: freq_analyzer.ld
##### INTERNAL BEGIN #####
endif
##### INTERNAL END #####
SCATTERgcc_freq_analyzer=freq_analyzer.ld
##### INTERNAL BEGIN #####
ifeq (${COMPILER}, rvmdk)
${COMPILER}/freq_analyzer.axf: freq_analyzer.sct
endif
SCATTERrvmdk_freq_analyzer=freq_analyzer.sct
ifeq (${COMPILER}, sourcerygxx)
${COMPILER}/freq_analyzer.axf: freq_analyzer_sourcerygxx.ld
endif
SCATTERsourcerygxx_freq_analyzer=lm4f120h5qr-rom.ld -T freq_analyzer_sourcerygxx.ld
##### INTERNAL END #####
ENTRY_freq_analyzer=ResetISR
##### INTERNAL BEGIN #####
CFLAGSccs=-DTARGET_IS_BLIZZARD_RA1
##### INTERNAL END #####
##### INTERNAL BEGIN #####
CFLAGScodered=-DTARGET_IS_BLIZZARD_RA1
##### INTERNAL END #####
##### INTERNAL BEGIN #####
CFLAGSewarm=-DTARGET_IS_BLIZZARD_RA1
##### INTERNAL END #####
CFLAGSgcc=-DTARGET_IS_BLIZZARD_RA1
##### INTERNAL BEGIN #####
CFLAGSrvmdk=-DTARGET_IS_BLIZZARD_RA1
##### INTERNAL END #####
##### INTERNAL BEGIN #####
CFLAGSsourcerygxx=-DTARGET_IS_BLIZZARD_RA1
##### INTERNAL END #####

#
# Include the automatically generated dependency files.
#
ifneq (${MAKECMDGOALS},clean)
##### INTERNAL BEGIN #####
ifneq (${MAKECMDGOALS},realclean)
##### INTERNAL END #####
-include ${wildcard ${COMPILER}/*.d} __dummy__
##### INTERNAL BEGIN #####
endif
##### INTERNAL END #####
endif
